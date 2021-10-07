/*
 * File:   cf_ahrs.c
 * Author: Pavlo Vlastos
 *
 * Created on January 28, 2020, 12:47 PM
 */

/******************************************************************************
 * #INCLUDES
 *****************************************************************************/
#include "cf_ahrs.h"
#include "xc.h"

/******************************************************************************
 * PRIVATE #DEFINES
 *****************************************************************************/


/******************************************************************************
 * PRIVATE DATATYPES 
 *****************************************************************************/

/******************************************************************************
 * PRIVATE VARIABLES 
 *****************************************************************************/
/* Accelerometer-related variables */
float g_vi[MSZ] = {0.0}; /* Gravitational inertial reference vector */
float acc_magnitude = 1.0;
float r_acc[MSZ][MSZ];
float kp_acc = 1.0;

/* Magnetometer-related variables */
float mag_vi[MSZ] = {0.0}; /* Magnetic field inertial reference vector */
float mag_magnitude = 1.0;
float r_mag[MSZ][MSZ];
float kp_mag = 1.0;

/* Gyro-related variables */
float gyro_vb[MSZ] = {0.0}; /* Angular rate measurements. Double check signs */
float gyro_bias[MSZ] = {0.0};
float w_meas_sum[MSZ] = {0.0};
float w_meas_i[MSZ] = {0.0};

/* Multi-use variables */
float axis_angle = 0.0;
float axis_v[MSZ] = {0.0};
float r_hat[MSZ][MSZ];
float r_hat_t[MSZ][MSZ];
float vi_hat[MSZ] = {0.0};
float dt = 0.01;

/* Quaternion variables */
float q_gyro[QSZ] = {0.0};
float q_est[QSZ] = {0.0};
float q_est_dot[QSZ] = {0.0};
float q_magnitude = 0.0;

/******************************************************************************
 * PUBLIC FUNCTIONS                                                          *
 *****************************************************************************/
void cf_ahrs_init(float desired_dt, const float exp_gyro_bias[MSZ]) {
    /* Do calibration here? */

    cf_ahrs_set_dt(desired_dt);
    
    cf_ahrs_set_gyro_biases(exp_gyro_bias);
}

void cf_ahrs_set_dt(float desired_dt) {
    dt = desired_dt;
}

void cf_ahrs_set_gyro_biases(const float exp_gyro_bias[MSZ]) {
    gyro_bias[0] = exp_gyro_bias[0];
    gyro_bias[1] = exp_gyro_bias[1];
    gyro_bias[2] = exp_gyro_bias[2];
}

void cf_ahrs_set_kp_acc(float desired_kp_acc) {
    kp_acc = desired_kp_acc;
}

void cf_ahrs_set_kp_mag(float desired_kp_mag) {
    kp_mag = desired_kp_mag;
}

void cf_ahrs_update(float acc_vb[MSZ], float mag_vb[MSZ],
        const float gyro_vb[MSZ], float *yaw, float *pitch, float *roll) {
    /*************************************************************************/
    /* Accelerometers */
    acc_magnitude = lin_alg_v_norm(acc_vb);

    /* Normalize the accelerometer vector*/
    if (acc_magnitude != 0.0) {
        lin_alg_v_scale(1.0 / acc_magnitude, acc_vb);
    }
    axis_angle = lin_alg_angle_from_2vecs(acc_vb, g_vi);

    lin_alg_cross(acc_vb, g_vi, axis_v);
    lin_alg_gen_dcm(axis_angle, axis_v, r_acc);

    /*************************************************************************/
    /* Magnetometers */
    mag_magnitude = lin_alg_v_norm(mag_vb);

    /* Normalize the magnetometer vector*/
    if (mag_magnitude != 0.0) {
        lin_alg_v_scale(1.0 / mag_magnitude, mag_vb);
    }
    axis_angle = lin_alg_angle_from_2vecs(mag_vb, mag_vi);

    lin_alg_cross(mag_vb, mag_vi, axis_v);
    lin_alg_gen_dcm(axis_angle, axis_v, r_mag);

    /*************************************************************************/
    lin_alg_set_v(0.0, 0.0, 0.0, w_meas_sum); /* Clear the correction vector */
    lin_alg_m_transpose(r_hat, r_hat_t);

    /* Feedback with accelerometers */
    lin_alg_m_v_mult(r_hat_t, g_vi, vi_hat);
    lin_alg_cross(acc_vb, vi_hat, w_meas_i);
    lin_alg_v_scale(kp_acc, w_meas_i); /* Weight the acc contribution */

    /* For each sensors that provides a reference vector, begin to sum 
     * up the respective elements of the correction vector */
    lin_alg_set_v(w_meas_i[0], w_meas_i[1], w_meas_i[2], w_meas_sum);

    /* Feedback with magnetometers */
    lin_alg_m_v_mult(r_hat_t, mag_vi, vi_hat);
    lin_alg_cross(mag_vb, vi_hat, w_meas_i);
    lin_alg_v_scale(kp_mag, w_meas_i); /* Weight the mag contribution */

    /* Continue the sum of elements for the correction vector */
    w_meas_sum[0] += w_meas_i[0];
    w_meas_sum[1] += w_meas_i[1];
    w_meas_sum[2] += w_meas_i[2];

    /* Gyros plus the correction vector. Gyros sign is flipped to match
     * the sign of rotation with the reference vectors */

    q_gyro[0] = 0.0;
    q_gyro[1] = -gyro_vb[0] + w_meas_sum[0] - gyro_bias[0];

    q_gyro[2] = -gyro_vb[1] + w_meas_sum[1] - gyro_bias[1];

    q_gyro[3] = -gyro_vb[2] + w_meas_sum[2] - gyro_bias[2];

    lin_alg_q_mult(q_est, q_gyro, q_est_dot);

    lin_alg_scale_q(0.5, q_est_dot);

    /* Primary integration */
    q_est[0] = q_est[0] + q_est_dot[0] * dt;
    q_est[1] = q_est[1] + q_est_dot[1] * dt;
    q_est[2] = q_est[2] + q_est_dot[2] * dt;
    q_est[3] = q_est[3] + q_est_dot[3] * dt;

    q_magnitude = 1.0 / lin_alg_q_norm(q_est);
    lin_alg_scale_q(q_magnitude, q_est);

    lin_alg_q2euler_abs(q_est, yaw, pitch, roll);

    lin_alg_q2dcm(q_est, r_hat);
}

/******************************************************************************
 * PRIVATE FUNCTIONS                                                          *
 *****************************************************************************/

/******************************************************************************
 * MODULE UNIT TESTS                                                          *
 *****************************************************************************/
#ifdef REPORT_ORIENTATION

int main(void) {
    return 0;
}

#endif