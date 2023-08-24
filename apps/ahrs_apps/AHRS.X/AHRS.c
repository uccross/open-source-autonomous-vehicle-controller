/** Attitude Heading Reference System
 * File:   AHRS.c
 * Author: Aaron Hunter
 * Brief: Implements the complementary filter described by Mahoney 2008
 * Created on 08/05/2022
 * Modified on 

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <xc.h>
#include "AHRS.h"
#include "Board.h"
#include "ICM_20948.h"  
#include "ICM_20948_registers.h" 
#include "lin_alg_float.h"
#include "SerialM32.h"
#include "System_timer.h"


/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE TYPEDEFS                                                            *
 ******************************************************************************/
// attitude quaternions
static float q_minus[QSZ] = {1, 0, 0, 0};
static float q_plus[QSZ] = {1, 0, 0, 0};
// gyro bias vector
static float b_minus[MSZ] = {0, 0, 0};
static float b_plus[MSZ] = {0, 0, 0};

/* data arrays */
static float gyro_cal[MSZ] = {0, 0, 0};
static float acc_cal[MSZ] = {0, 0, 0};
static float mag_cal[MSZ] = {0, 0, 0};

/*filter gains*/
static float kp_a = 2.5; //accelerometer proportional gain
static float ki_a = 0.05; // accelerometer integral gain
static float kp_m = 2.5; // magnetometer proportional gain
static float ki_m = 0.05; //magnetometer integral gain

/* gravity inertial vector */
static float a_i[MSZ] = {0, 0, 1.0};

/* Earth's magnetic field inertial vector at 37 N, 122 W, normalized 
 * North 22,680.8 nT	East 5,217.6 nT	Down 41,324.7 nT, value from NOAA
 * converted into ENU format: */
static float m_i[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
/**
 * @function quat2euler()
 * @param q A quaternion
 * @param euler a vector of euler angles in [psi, theta, roll] order
 */
static void quat2euler(float q[MSZ], float euler[MSZ]);
/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
static void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]);
/**
 * @function v_copy()
 * @param v_in the vector to be copied
 * @param v_out vector to receive the copy
 * @return none
 */
static void v_copy(float v_in[MSZ], float v_out[MSZ]);

/**
 * @function m_norm()
 * @param M A matrix
 * @return The magnitude of the M, m_norm
 */
static float m_norm(float M[MSZ]);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/**
 * @Function 
 * @param none
 * @return current magnetic aiding vector
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_get_mag_inertial(float mag_i[MSZ]) {
    mag_i[0] = m_i[0];
    mag_i[1] = m_i[1];
    mag_i[2] = m_i[2];
}

/**
 * @Function 
 * @param vector with normalized local magnetic field strength (ENU) 
 * @return none
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_set_mag_inertial(float mag_i[MSZ]) {
    m_i[0] = mag_i[0];
    m_i[1] = mag_i[1];
    m_i[2] = mag_i[2];
}

/**
 * @Function 
 * @param none
 * @return current magnetic aiding vector
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_get_filter_gains(float *kp_a_get, float *ki_a_get, float *kp_m_get, float *ki_m_get) {
    *kp_a_get = kp_a;
    *ki_a_get = ki_a;
    *kp_m_get = kp_m;
    *ki_m_get = ki_m;
}

/**
 * @Function 
 * @param none
 * @return current magnetic aiding vector
 * @brief 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified */
void AHRS_set_filter_gains(float kp_a_set, float ki_a_set, float kp_m_set, float ki_m_set) {
    kp_a = kp_a_set;
    ki_a = ki_a_set;
    kp_m = kp_m_set;
    ki_m = ki_m_set;
}

/**
 * @Function AHRS_update
 * @param IMU data in the form of three axis
 * normalized magnetometer and acceleromter arrays, gyro data in rad/sec
 * @param dt, the integration time in seconds
 * @param mag_i normalized inertial aiding vector of magnetic field at location
 * @params kp_a, kp_i, km_p, km_i filter gains
 * @return attitude quaternion and gyro biases vector (x,y,z)
 * @brief implements the complementary filter update step 
 * @note 
 * @author Aaron Hunter, 08/05/2022
 * @modified  11/01/22 to return quaternion attitude and bias values rather
 * than Euler angles*/
void AHRS_update(float accels[MSZ], float mags[MSZ], float gyros[MSZ],
        float dt, float q[QSZ], float bias[MSZ]) {

    float a_b[MSZ]; //estimated gravity vector in body frame
    float m_b[MSZ]; //estimated magnetic field vector in body frame

    float gyro_cal[MSZ]; // gyros with bias correction
    float gyro_wfb[MSZ]; // gyro 'rate' after feedback
    float w_meas_ap[MSZ]; // accelerometer proportion correction rate
    float w_meas_mp[MSZ]; // magnetometer proportional correction rate
    float w_meas_ai[MSZ]; // accelerometer integral correction rate
    float w_meas_mi[MSZ]; // magnetometer integral correction rate

    float gyro_q_wfb[QSZ]; // temporary quaternion to hold feedback term
    float q_dot[QSZ]; // quaternion derivative
    float q_norm;
    float acc_n;
    float mag_n;

    /* normalize inertial measurements */
    acc_n = 1.0 / m_norm(accels);
    accels[0] = accels[0] * acc_n;
    accels[1] = accels[1] * acc_n;
    accels[2] = accels[2] * acc_n;

    mag_n = 1.0 / m_norm(mags);
    mags[0] = mags[0] * mag_n;
    mags[1] = mags[1] * mag_n;
    mags[2] = mags[2] * mag_n;

    /*Accelerometer attitude calculations */
    q_rot_v_q(a_i, q_minus, a_b); //estimate gravity vector in body frame 
    lin_alg_cross(accels, a_b, w_meas_ap); // calculate the accelerometer rate term
    v_copy(w_meas_ap, w_meas_ai); // make a copy for the integral term
    lin_alg_v_scale(kp_a, w_meas_ap); // calculate the accelerometer proportional feedback term 
    lin_alg_v_scale(ki_a, w_meas_ai); // calculate the accelerometer integral feedback term 

    /*Magnetometer attitude calculations*/
    q_rot_v_q(m_i, q_minus, m_b); //estimate magnetic field vector in body frame
    lin_alg_cross(mags, m_b, w_meas_mp); // calculate the magnetometer rate term
    v_copy(w_meas_mp, w_meas_mi); //make a copy for the integral term
    lin_alg_v_scale(kp_m, w_meas_mp); // calculate the magnetometer proportional feedback term
    lin_alg_v_scale(ki_m, w_meas_mi); // calculate the magnetometer integral feedback term

    /*Gyro attitude contributions */
    lin_alg_v_v_sub(gyros, b_minus, gyro_cal); //correct the gyros with the b_minus vector

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
    b_plus[0] = b_minus[0] - (w_meas_ai[0] + w_meas_mi[0]) * dt;
    b_plus[1] = b_minus[1] - (w_meas_ai[1] + w_meas_mi[1]) * dt;
    b_plus[2] = b_minus[2] - (w_meas_ai[2] + w_meas_mi[2]) * dt;

    /* update b_minus and q_minus */
    b_minus[0] = b_plus[0];
    b_minus[1] = b_plus[1];
    b_minus[2] = b_plus[2];
    q_minus[0] = q_plus[0];
    q_minus[1] = q_plus[1];
    q_minus[2] = q_plus[2];
    q_minus[3] = q_plus[3];

    /* set external attitude and bias*/
    bias[0] = b_plus[0];
    bias[1] = b_plus[1];
    bias[2] = b_plus[2];
    q[0] = q_plus[0];
    q[1] = q_plus[1];
    q[2] = q_plus[2];
    q[3] = q_plus[3];
}



/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS                                            *
 ******************************************************************************/

/**
 * @function quat2euler()
 * @param q A quaternion
 * @param euler a vector of euler angles in [psi, theta, roll] order
 */
static void quat2euler(float q[MSZ], float euler[MSZ]) {
    float q00 = q[0] * q[0];
    float q11 = q[1] * q[1];
    float q22 = q[2] * q[2];
    float q33 = q[3] * q[3];

    // psi
    euler[0] = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), ((q00 + q11 - q22 - q33)));
    // theta
    euler[1] = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
    // phi
    euler[2] = atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), q00 - q11 - q22 + q33);
}

/**
 * @function q_rot_v_q()
 * Rotate a vector from the inertial frame to the body frame
 * @param v_i, a 3space vector in the inertial frame
 * @param q an attitude quaternion
 * sets v_b to the rotated inertial vector in the body frame
 */
static void q_rot_v_q(float v_i[MSZ], float q[QSZ], float v_b[MSZ]) {
    float q_i[QSZ];
    float q_temp[QSZ];
    float q_conj[QSZ];
    float q_b[QSZ]; // container for inertial vector in body frame as pure quaternion

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

/**
 * @function v_copy()
 * @param v_in the vector to be copied
 * @param v_out vector to receive the copy
 * @return none
 */
static void v_copy(float v_in[MSZ], float v_out[MSZ]) {
    int row;
    for (row = 0; row < MSZ; row++) {
        v_out[row] = v_in[row];
    }
}

/**
 * @function m_norm()
 * @param M A matrix
 * @return The magnitude of the M, m_norm
 */
static float m_norm(float M[MSZ]) {
    return ((float) sqrt(M[0] * M[0] + M[1] * M[1] + M[2] * M[2]));
}




#ifdef AHRS_TESTING

#define MEAS_PERIOD 20 // measurement period in msec
#define DT 0.02 

int main(void) {
    uint32_t start_time = 0;
    const uint32_t warmup_time = 250; //msec
    int8_t warmed = FALSE;
    uint32_t current_time = 0;
    uint32_t update_start = 0;
    uint32_t update_end = 0;

    /*attitude*/
    float q_test[QSZ] = {1, 0, 0, 0};
    /*gyro bias*/
    float bias_test[MSZ] = {0, 0, 0};
    /*euler angles (yaw, pitch, roll) */
    float euler_test[MSZ] = {0, 0, 0};

    /*filter gains for testing */
    float kpa = 4.0; //accelerometer proportional gain
    float kia = 0.1; // accelerometer integral gain
    float kpm = 4.0; // magnetometer proportional gain
    float kim = 0.1; //magnetometer integral gain

    /*containers for getting current filter gains*/
    float kpa_test;
    float kia_test;
    float kpm_test;
    float kim_test;

    /* test location for magnetic field vector*/
    float mag_i_set[MSZ] = {5174.5, 19386.2, -47948.4}; // ENU
    float mag_norm = lin_alg_v_norm(mag_i_set);
    float mag_norm_inv = 1 / mag_norm;
    lin_alg_v_scale(mag_norm_inv, mag_i_set);
    /* value for santa cruz */
    float mag_i_sc[MSZ] = {0.110011998753301, 0.478219898291142, -0.871322609031072};

    float mag_get[MSZ];

    /*timing and conversion*/
    const float dt = DT;
    const float deg2rad = M_PI / 180.0;
    const float rad2deg = 180.0 / M_PI;

    /*calibration matrices*/

    /* Rover IMU calibration */
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

    // IMU 1 calibration values:
    //    float A_acc[MSZ][MSZ] = {
    //        {6.12649072e-05, -7.69323670e-07,  7.41523511e-07},
    //        {8.86960833e-08,  5.99163592e-05,  4.45763158e-07},
    //        {-8.40898963e-08, -9.22602608e-07,  5.95395979e-05}
    //    };
    //        float A_mag[MSZ][MSZ] = {
    //        {3.36826234e-03, -1.56956478e-05, -1.16520778e-05},
    //        { 4.48554406e-05,  3.54448710e-03,  1.51280264e-05},
    //        {-1.02003198e-04, -3.17665038e-05,  3.48151565e-03}
    //    };
    //    float b_acc[MSZ] = {0.02178154, 0.01720978, 0.01673077};
    //    float b_mag[MSZ] = {-0.46909233, -0.03577201, 0.46432114};



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

    /*test getters and setters*/
    AHRS_get_filter_gains(&kpa_test, &kia_test, &kpm_test, &kim_test);
    printf("filter gains set to kp_i: %f, ki_a: %f, kp_m: %f, ki_m: %f \r\n",
            kpa_test, kia_test, kpm_test, kim_test);
    AHRS_set_filter_gains(kpa, kia, kpm, kim);
    AHRS_get_filter_gains(&kpa_test, &kia_test, &kpm_test, &kim_test);
    printf("filter gains set to kp_i: %f, ki_a: %f, kp_m: %f, ki_m: %f \r\n",
            kpa_test, kia_test, kpm_test, kim_test);

    AHRS_get_mag_inertial(mag_get);
    printf("Magnetic aiding vector: %f, %f, %f \r\n", mag_get[0], mag_get[1], mag_get[2]);

    AHRS_set_mag_inertial(mag_i_set);
    AHRS_get_mag_inertial(mag_get);
    printf("Magnetic aiding vector: %f, %f, %f \r\n", mag_get[0], mag_get[1], mag_get[2]);
    AHRS_set_mag_inertial(mag_i_sc); // reset to santa cruz for testing 

    while (1) {
        current_time = Sys_timer_get_msec();
        if ((current_time - start_time) >= MEAS_PERIOD) {
            // collect raw gyro data
            IMU_start_data_acq();
            start_time = current_time;
        }
        if (IMU_is_data_ready() == TRUE) {
            IMU_get_norm_data(&IMU_data);

            acc_cal[0] = IMU_data.acc.x;
            acc_cal[1] = IMU_data.acc.y;
            acc_cal[2] = IMU_data.acc.z;
            mag_cal[0] = IMU_data.mag.x;
            mag_cal[1] = IMU_data.mag.y;
            mag_cal[2] = IMU_data.mag.z;
            /*scale gyro readings into rad/sec */
            gyro_cal[0] = IMU_data.gyro.x * deg2rad;
            gyro_cal[1] = IMU_data.gyro.y * deg2rad;
            gyro_cal[2] = IMU_data.gyro.z * deg2rad;
            update_start = Sys_timer_get_usec();
            AHRS_update(acc_cal, mag_cal, gyro_cal, dt, q_test, bias_test);
            update_end = Sys_timer_get_usec();
            quat2euler(q_test, euler_test);
            printf("%+3.1f, %+3.1f, %+3.1f, ", euler_test[0] * rad2deg, euler_test[1] * rad2deg, euler_test[2] * rad2deg);
            printf("%+1.3e, %+1.3e, %+1.3e, ", bias_test[0], bias_test[1], bias_test[2]);
            printf("%d\r\n", update_end - update_start);
        }
    }
    return 0;
}

#endif
