/* 
 * File:   ahrs_main.c
 * Author: Aaron
 *
 * Created on June 21, 2022, 9:50 AM
 */

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <sys/attribs.h>  //for ISR definitions
#include <proc/p32mx795f512l.h>
#include <xc.h>
#include "Board.h"
#include "ICM_20948.h"  
#include "ICM_20948_registers.h" 
#include "SerialM32.h"
#include "System_timer.h"
/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define CAL_TIME 5000 //bias calculation time in msec
#define MEAS_PERIOD 20 // measurement period in msec
#define THREESEC 3000
#define DT 0.02 
#define MSZ 3

/*******************************************************************************
 * Function Declarations                                                       *
 ******************************************************************************/

/**
 * @function m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
void m_v_mult(double m[MSZ][MSZ], double v[MSZ], double v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
}

/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
void extract_angles(double dcm[MSZ][MSZ], double euler[MSZ]) {
    const double pi_2 = M_PI / 2;
    euler[0] = atan2(dcm[1][0], dcm[0][0]); /* Yaw */
    if (dcm[2][0] > 1.0) {
        euler[1] = -pi_2;
    } else if (dcm[2][0] < -1.0) {
        euler[1] = pi_2;
    } else {
        euler[1] = -asin(dcm[2][0]); /* Pitch */
    }

    euler[2] = atan2(dcm[2][1], dcm[2][2]); /* Roll */

}

void main(void) {

    uint32_t start_time = 0;
    uint32_t current_time = 0;
    uint32_t update_start = 0;
    uint32_t update_end = 0;
    const uint32_t warmup_time = 250; //msec
    int8_t warmed = FALSE;
    uint8_t row;
    uint8_t col;
    double dt = DT; //integration time in sec
    /****** Filter gains  ***************************/
    double kp_a = 2.5; //Accelerometer proportional gain 
    double ki_a = 0.05; // Accelerometer integral gain
    double kp_m = 2.5; //Magnetometer proportional gain 
    double ki_m = 0.05; //Magnetometer integral gain
    /************************************************/
    /*rad <--> deg conversion constants*/
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;

    /********data arrays*******************************************************/
    double gyro_cal[MSZ];
    double acc_raw[MSZ];
    double acc_tmp[MSZ];
    double acc_cal[MSZ];
    double mag_raw[MSZ];
    double mag_cal[MSZ];
    double mag_tmp[MSZ];

    /****** Calibration parameters ********************************************/
    /*gyroscope scaling to rad/sec*/
    double gyro_scale = deg2rad;
    /*Accelerometer calibration matrix*/
    float A_acc[MSZ][MSZ] = {
        6.01180201773358e-05, -6.28352073406424e-07, -3.91326747595870e-07,
        -1.18653342135860e-06, 6.01268083773005e-05, -2.97010157797952e-07,
        -3.19011230800348e-07, -3.62174516629958e-08, 6.04564465269327e-05
    };
    float A_mag[MSZ][MSZ] = {
        0.00351413733554131, -1.74599042407869e-06, -1.62761272908763e-05,
        6.73767225208446e-06, 0.00334531206332366, -1.35302929502152e-05,
        -3.28233797524166e-05, 9.29337701972177e-06, 0.00343350080131375
    };
    float b_acc[MSZ] = {-0.0156750747576770, -0.0118720194488050, -0.0240128301624044};
    float b_mag[MSZ] = {-0.809679246097106, 0.700742334522691, -0.571694648765172};
    /*******************Inertial aiding vectors *******************************/
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    double m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};
    // gravity in inertial frame
    double a_i[MSZ] = {0, 0, 1.0};
    /**************************************************************************/

    // Euler angles
    double euler[MSZ] = {0, 0, 0};

    // attitude DCMs
    double r_minus[MSZ][MSZ] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double r_plus[MSZ][MSZ] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // gyro bias vectors
    double b_minus[MSZ] = {0, 0, 0};
    double b_plus[MSZ] = {0, 0, 0};

        /* gryo, accelerometer, magnetometer data struct */
    struct IMU_out IMU_data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    Board_init();
    Serial_init();
    Sys_timer_init();
    start_time = Sys_timer_get_msec();
    current_time = start_time;
    /* maybe not necessary, but give the chip time to stabilize*/
    while (!warmed) {
        current_time = Sys_timer_get_msec();
        if (current_time - start_time >= warmup_time) warmed = TRUE;
    }
    IMU_init(IMU_SPI_MODE);
    printf("Mahoney DCM Filter from Matlab Coder\r\n");
    /* load calibration matrices */
    IMU_set_mag_cal(A_mag, b_mag);
    IMU_set_acc_cal(A_acc, b_acc);
    start_time = Sys_timer_get_msec();
    while (1) {
        current_time = Sys_timer_get_msec();
        if ((current_time - start_time) >= MEAS_PERIOD) {
            // collect raw gyro data
            IMU_start_data_acq();
            start_time = current_time;
        }
        if ((current_time - start_time) >= MEAS_PERIOD) {
            // collect raw gyro data
            IMU_start_data_acq();
            start_time = current_time;
        }
        if (IMU_is_data_ready() == TRUE) {
            IMU_get_norm_data(&IMU_data);

            acc_cal[0] = (double) IMU_data.acc.x;
            acc_cal[1] = (double) IMU_data.acc.y;
            acc_cal[2] = (double) IMU_data.acc.z;
            mag_cal[0] = (double) IMU_data.mag.x;
            mag_cal[1] = (double) IMU_data.mag.y;
            mag_cal[2] = (double) IMU_data.mag.z;
            /*scale gyro readings into rad/sec */
            gyro_cal[0] = (double) IMU_data.gyro.x * deg2rad;
            gyro_cal[1] = (double) IMU_data.gyro.y * deg2rad;
            gyro_cal[2] = (double) IMU_data.gyro.z * deg2rad;
            update_start = Sys_timer_get_usec();
            ahrs_m_update(r_minus, b_minus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt,
                    kp_a, ki_a, kp_m, ki_m, r_plus, b_plus);
            update_end = Sys_timer_get_usec();
            /*extract euler angles from DCM; */
            extract_angles(r_plus, euler);

            printf("%+3.1f, %+3.1f, %+3.1f, ", euler[0] * rad2deg, euler[1] * rad2deg, euler[2] * rad2deg);
            printf("%+1.3e, %+1.3e, %+1.3e, ", b_plus[0], b_plus[1], b_plus[2]);
            printf("%d \r\n", update_end - update_start);
            // update b_minus and r_minus 
            for (row = 0; row < MSZ; row++) {
                for (col = 0; col < MSZ; col++) {
                    r_minus[row][col] = r_plus[row][col];
                    b_minus[col] = b_plus[col];
                }
            }
        }
    }

}




