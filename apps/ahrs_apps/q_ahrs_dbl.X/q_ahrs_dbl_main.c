/*
 * File:   q_ahrs_dbl_main.c
 * Author: Aaron
 *
 * Created on July 21, 2022, 4:40 PM
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
#define QSZ 4

/*******************************************************************************
 * FUNCTION PROTOTYPES                                                         *
 ******************************************************************************/
void quat2euler(double q[MSZ], double euler[MSZ]);
void lin_alg_q_mult(double q[QSZ], double p[QSZ], double r[QSZ]);
void q_rot_v_q(double v_i[MSZ], double q[QSZ], double v_b[MSZ]);
void v_copy(double m_in[MSZ], double m_out[MSZ]);
void lin_alg_cross(double u[MSZ], double v[MSZ], double w_out[MSZ]);
void lin_alg_v_scale(double s, double v[MSZ]);
void lin_alg_v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]);
void lin_alg_v_v_sub(double v1[MSZ], double v2[MSZ], double v_out[MSZ]);
double lin_alg_q_norm(double q[QSZ]);

/*******************************************************************************
 * FUNCTIONS                                                                    *
 ******************************************************************************/

/**
 * @function quat2euler()
 * @param q A quaternion
 * @param euler a vector of euler angles in [psi, theta, roll] order
 */
void quat2euler(double q[MSZ], double euler[MSZ]) {
    double q00 = q[0] * q[0];
    double q11 = q[1] * q[1];
    double q22 = q[2] * q[2];
    double q33 = q[3] * q[3];

    // psi
    euler[0] = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), ((q00 + q11 - q22 - q33)));
    // theta
    euler[1] = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // phi
    euler[2] = atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), q00 - q11 - q22 + q33);
}

/**
 * @function lin_alg_q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void lin_alg_q_mult(double q[QSZ], double p[QSZ], double r[QSZ]) {
    r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    r[1] = p[1] * q[0] + p[0] * q[1] + p[3] * q[2] - p[2] * q[3];
    r[2] = p[2] * q[0] - p[3] * q[1] + p[0] * q[2] + p[1] * q[3];
    r[3] = p[3] * q[0] + p[2] * q[1] - p[1] * q[2] + p[0] * q[3];
}

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
void q_rot_v_q(double v_i[MSZ], double q[QSZ], double v_b[MSZ]) {
    double q_i[QSZ];
    double q_temp[QSZ];
    double q_conj[QSZ];
    double q_b[QSZ]; // container for inertial vector in body frame as pure quaternion

    // calculate conjugate of q 
    q_conj[0] = q[0];
    q_conj[1] = -q[1];
    q_conj[2] = -q[2];
    q_conj[3] = -q[3];

    //   convert v_i to a pure quaternion --> q_i
    q_i[0] = 0;
    q_i[1] = v_i[0];
    q_i[2] = v_i[1];
    q_i[3] = v_i[2];
    //   first quaternion product q_i by q --> q_temp
    lin_alg_q_mult(q_i, q, q_temp);
    //   second quaternion product q_conj by q_temp -->q_b
    lin_alg_q_mult(q_conj, q_temp, q_b);
    //    set v_b to imaginary part of q_b
    v_b[0] = q_b[1];
    v_b[1] = q_b[2];
    v_b[2] = q_b[3];
}

void v_copy(double m_in[MSZ], double m_out[MSZ]) {
    int row;
    for (row = 0; row < MSZ; row++) {
        m_out[row] = m_in[row];
    }
}

/**
 * @function lin_alg_cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void lin_alg_cross(double u[MSZ], double v[MSZ], double w_out[MSZ]) {
    w_out[0] = u[1] * v[2] - u[2] * v[1];
    w_out[1] = u[2] * v[0] - u[0] * v[2];
    w_out[2] = u[0] * v[1] - u[1] * v[0];
}

/**
 * @function lin_alg_v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void lin_alg_v_scale(double s, double v[MSZ]) {
    int row;
    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void lin_alg_v_v_add(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function lin_alg_v_v_sub()
 * Add a vector value to a vector
 * @param v1 Vector to subtract to another vector
 * @param v2 Vector to have a vector subtracted to it
 * @param v_out Vector as difference of two vectors
 */
void lin_alg_v_v_sub(double v1[MSZ], double v2[MSZ], double v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] - v2[row];
    }
}

/**
 * @function lin_alg_q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
double lin_alg_q_norm(double q[QSZ]) {
    return ((double) sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
}

/**
 * @function m_norm()
 * @param M A matrix
 * @return The magnitude of the M, m_norm
 */
double m_norm(double M[MSZ]) {
    return ((double) sqrt(M[0] * M[0] + M[1] * M[1] + M[2] * M[2]));
}

//void ahrs_update(double q_minus[QSZ], double q_plus[QSZ], double bias_minus[MSZ],
//        double bias_plus[MSZ], double gyros[MSZ], double mags[MSZ], double accels[MSZ],
//        double mag_i[MSZ], double acc_i[MSZ], double dt) 

void ahrs_update(double q_minus[QSZ], double bias_minus[MSZ], double gyros[MSZ],
        double mags[MSZ], double accels[MSZ], double mag_i[MSZ], double acc_i[MSZ],
        double dt, double kp_a, double ki_a, double kp_m, double ki_m,
        double q_plus[QSZ], double bias_plus[MSZ]) {

    double acc_b[MSZ]; //estimated gravity vector in body frame
    double mag_b[MSZ]; //estimated magnetic field vector in body frame

    double gyro_cal[MSZ]; // gyros with bias correction
    double gyro_wfb[MSZ]; // gyro 'rate' after feedback
    double w_meas_ap[MSZ]; // accelerometer proportion correction rate
    double w_meas_mp[MSZ]; // magnetometer proportional correction rate
    double w_meas_ai[MSZ]; // accelerometer integral correction rate
    double w_meas_mi[MSZ]; // magnetometer integral correction rate

    double gyro_q_wfb[QSZ]; // temporary quaternion to hold feedback term
    double q_dot[QSZ]; // quaternion derivative
    double b_dot[MSZ]; // bias vector derivative
    double q_norm;
    double acc_n;
    double mag_n;

    /* normalize inertial measurements */
    acc_n = m_norm(accels);
    accels[0] = accels[0] / acc_n;
    accels[1] = accels[1] / acc_n;
    accels[2] = accels[2] / acc_n;
    
    mag_n = m_norm(mags);
    mags[0] = mags[0] / mag_n;
    mags[1] = mags[1] / mag_n;
    mags[2] = mags[2] / mag_n;

    /*Accelerometer attitude calculations */
    q_rot_v_q(acc_i, q_minus, acc_b); //estimate gravity vector in body frame 
    lin_alg_cross(accels, acc_b, w_meas_ap); // calculate the accelerometer rate term
    v_copy(w_meas_ap, w_meas_ai); // make a copy for the integral term
    lin_alg_v_scale(kp_a, w_meas_ap); // calculate the accelerometer proportional feedback term 
    lin_alg_v_scale(ki_a, w_meas_ai); // calculate the accelerometer integral feedback term 

    /*Magnetometer attitude calculations*/
    q_rot_v_q(mag_i, q_minus, mag_b); //estimate magnetic field vector in body frame
    lin_alg_cross(mags, mag_b, w_meas_mp); // calculate the magnetometer rate term
    v_copy(w_meas_mp, w_meas_mi); //make a copy for the integral term
    lin_alg_v_scale(kp_m, w_meas_mp); // calculate the magnetometer proportional feedback term
    lin_alg_v_scale(ki_m, w_meas_mi); // calculate the magnetometer integral feedback term

    /*Gyro attitude contributions */
    lin_alg_v_v_sub(gyros, bias_minus, gyro_cal); //correct the gyros with the b_minus vector

    /* calculate total rate term gyro_wfb */
    lin_alg_v_v_add(w_meas_ap, w_meas_mp, gyro_wfb);
    lin_alg_v_v_add(gyro_cal, gyro_wfb, gyro_wfb);

    /* convert feedback term to a pure quaternion */
    gyro_q_wfb[0] = 0;
    gyro_q_wfb[1] = gyro_wfb[0];
    gyro_q_wfb[2] = gyro_wfb[1];
    gyro_q_wfb[3] = gyro_wfb[2];

    /* compute the quaternion derivative q_dot */
    lin_alg_q_mult(q_minus, gyro_q_wfb, q_dot);

    /* integrate term by term */
    q_plus[0] = q_minus[0] + 0.5 * q_dot[0] * dt;
    q_plus[1] = q_minus[1] + 0.5 * q_dot[1] * dt;
    q_plus[2] = q_minus[2] + 0.5 * q_dot[2] * dt;
    q_plus[3] = q_minus[3] + 0.5 * q_dot[3] * dt;

    // normalize the quaternion for stability
    q_norm = lin_alg_q_norm(q_plus);
    q_plus[0] = q_plus[0] / q_norm;
    q_plus[1] = q_plus[1] / q_norm;
    q_plus[2] = q_plus[2] / q_norm;
    q_plus[3] = q_plus[3] / q_norm;

    // compute the integral of the bias term by term
    bias_plus[0] = bias_minus[0] - (w_meas_ai[0] + w_meas_mi[0]) * dt;
    bias_plus[1] = bias_minus[1] - (w_meas_ai[1] + w_meas_mi[1]) * dt;
    bias_plus[2] = bias_minus[2] - (w_meas_ai[2] + w_meas_mi[2]) * dt;
}

int main(void) {
    uint32_t start_time = 0;
    const uint32_t warmup_time = 250; //msec
    int8_t warmed = FALSE;
    uint32_t current_time = 0;
    uint32_t update_start = 0;
    uint32_t update_end = 0;
    /*filter gains*/
    double kp_a = 2.5; //accelerometer proportional gain
    double ki_a = 0.05; // accelerometer integral gain
    double kp_m = 2.5; // magnetometer proportional gain
    double ki_m = 0.05; //magnetometer integral gain
    /*timing and conversion*/
    const double dt = DT;
    const double deg2rad = M_PI / 180.0;
    const double rad2deg = 180.0 / M_PI;

    /* NOTE we leave the cal data in floats rather than rewrite the driver
     if there are benefits to be had from the double precision we will update
     the driver or just use raw data and perform calibration inside the ahrs
     algorithm */
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
    // gravity inertial vector
    double a_i[MSZ] = {0, 0, 1.0};
    // Earth's magnetic field inertial vector, normalized 
    // North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
    // converted into ENU format and normalized:
    double m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    // Euler angles
    double euler[MSZ] = {0, 0, 0};

    // attitude quaternions
    double q_minus[QSZ] = {1, 0, 0, 0};
    double q_plus[QSZ] = {1, 0, 0, 0};
    // gyro bias vector
    double b_minus[MSZ] = {0, 0, 0};
    double b_plus[MSZ] = {0, 0, 0};
    /* data arrays */
    double gyro_cal[MSZ] = {0, 0, 0};
    double acc_cal[MSZ] = {0, 0, 0};
    double mag_cal[MSZ] = {0, 0, 0};
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

    printf("Quaternion Mahoney AHRS %s, %s. \r\n", __DATE__, __TIME__);
    /* load calibration matrices */
    IMU_set_mag_cal(A_mag, b_mag);
    IMU_set_acc_cal(A_acc, b_acc);

    while (1) {
        current_time = Sys_timer_get_msec();
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
            //            ahrs_update(q_minus, q_plus, b_minus, b_plus, gyro_cal, mag_cal, acc_cal, m_i, a_i, dt);
            ahrs_update(q_minus, b_minus, gyro_cal, mag_cal, acc_cal, m_i,
                    a_i, dt, kp_a, ki_a, kp_m, ki_m, q_plus, b_plus);
            update_end = Sys_timer_get_usec();
            quat2euler(q_plus, euler);

            printf("%+3.1f, %+3.1f, %+3.1f, ", euler[0] * rad2deg, euler[1] * rad2deg, euler[2] * rad2deg);
            printf("%+1.3e, %+1.3e, %+1.3e, ", b_plus[0], b_plus[1], b_plus[2]);
            printf("%d\r\n", update_end - update_start);
            // update b_minus and q_minus
            b_minus[0] = b_plus[0];
            b_minus[1] = b_plus[1];
            b_minus[2] = b_plus[2];
            q_minus[0] = q_plus[0];
            q_minus[1] = q_plus[1];
            q_minus[2] = q_plus[2];
            q_minus[3] = q_plus[3];
        }
    }
    return 0;
}
