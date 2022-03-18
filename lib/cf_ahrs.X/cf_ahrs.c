/*
 * File:   cf_ahrs.c
 * Author: Pavlo Vlastos
 *
 * Created on January 28, 2020, 12:47 PM
 */

/******************************************************************************
 * #INCLUDES
 *****************************************************************************/
#include <math.h>

#include "cf_ahrs.h"
#include "xc.h"

/******************************************************************************
 * PRIVATE #DEFINES
 *****************************************************************************/
#define CF_2_PI (2.0 * M_PI)
#define NORM_ZERO_THRESHOLD 0.01 /* Arbitrarily chosen */
#define QUATERNION_OR_DCM_BASED 0 /* 0 for quaternion, 1 for DCM-based */

/******************************************************************************
 * PRIVATE DATATYPES 
 *****************************************************************************/

/******************************************************************************
 * PRIVATE VARIABLES 
 *****************************************************************************/
/* NOTE: that these are declared here, but are not necessarily set to correct 
 * inertial values until the initialization. */

/* Accelerometer-related variables */
float g_vi[MSZ] = {0.0}; /* Gravitational inertial reference vector */
float acc_magnitude = 1.0;
float r_acc[MSZ][MSZ];
float kp_acc = 1.0;

/* Magnetometer-related variables */
float mag_vi[MSZ] = {0.0}; /* Magnetic field inertial reference vector */
//float mag_vb_common[MSZ] = {0.0}; /* Mag vector rotated into same frame as acc */
float mag_magnitude = 1.0;
float r_mag[MSZ][MSZ];
float kp_mag = 1.0;
float ki_gyro = 0.0001;

/* Gyro-related variables */
float omega_vb[MSZ] = {0.0}; /* Angular rates */
float omega_bias[MSZ] = {0.0};
float w_meas_sum[MSZ] = {0.0};
float b[MSZ] = {0.0};
float w_meas_i[MSZ] = {0.0};
float w_meas_acc[MSZ] = {0.0};
float w_meas_mag[MSZ] = {0.0};

/* Multi-use variables */
float axis_angle = 0.0;
float axis_v[MSZ] = {0.0};
float r_hat[MSZ][MSZ];
float r_hat_t[MSZ][MSZ];
float R_plus[MSZ][MSZ];
float R_minus[MSZ][MSZ];
//float r_align[MSZ][MSZ]; /* Rotation matrix to align mag frame with acc frame */
//float r_align_t[MSZ][MSZ]; /* Rotation matrix to align mag frame with acc frame */
float vi_hat[MSZ] = {0.0};
float dt = 0.01;

/* Euler Angles */
float cf_yaw = 0.0;
float cf_pitch = 0.0;
float cf_roll = 0.0;
float cf_yaw_conj = 0.0;
float cf_pitch_conj = 0.0;
float cf_roll_conj = 0.0;

/* Quaternion variables */
float q_gyro[QSZ] = {0.0};
float q_est[QSZ] = {0.0};
float q_est_conj[QSZ] = {0.0};
float q_est_dot[QSZ] = {0.0};
float q_magnitude = 0.0;
float q_norm = 1.0;

/******************************************************************************
 * PUBLIC FUNCTIONS                                                          *
 *****************************************************************************/
void cf_ahrs_init(float desired_dt, const float exp_gyro_bias[MSZ]) {
    /* Do calibration here? */

    cf_ahrs_set_dt(desired_dt);

    cf_ahrs_set_gyro_biases(exp_gyro_bias);

    /* Set all inertial-frame aiding vectors */
    lin_alg_set_v(0.0, 0.0, 1.0, g_vi);

    /* See https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm 
     * for values (changes by 0.08 degrees each year) 
     *              East,    North,     Down            */
    lin_alg_set_v(5249.8, 22722.9, 41327.4, mag_vi);
    float mag_vi_norm = lin_alg_v_norm(mag_vi);

    /* Kept here more to highlight the fact that it is important to check if 
     * the norm of a vector is zero for other use cases. Not really practical 
     * here, since we know the magnetic field should be non-zero in magnitude 
     */
    if (mag_vi_norm != 0.0) {
        lin_alg_v_scale((1.0 / mag_vi_norm), mag_vi);
    }

    /* Set quaternions */
    lin_alg_set_q(0.0, 0.0, 0.0, q_est);
    lin_alg_set_q(0.0, 0.0, 0.0, q_est_dot);


    /* Explicitly set all matrices to avoid nans */
    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            r_acc);

    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            r_mag);

    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            r_hat);

    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            r_hat_t);

    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            R_plus);

    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            R_minus);

    //    lin_alg_gen_dcm_with_angles(0.0, 0.0, M_PI, r_align);
    //    lin_alg_m_transpose(r_align, r_align_t);
}

void cf_ahrs_set_dt(float desired_dt) {
    dt = desired_dt;
}

void cf_ahrs_set_gyro_biases(const float exp_gyro_bias[MSZ]) {
    omega_bias[0] = exp_gyro_bias[0];
    omega_bias[1] = exp_gyro_bias[1];
    omega_bias[2] = exp_gyro_bias[2];
}

void cf_ahrs_set_kp_acc(float desired_kp_acc) {
    kp_acc = desired_kp_acc;
}

void cf_ahrs_set_kp_mag(float desired_kp_mag) {
    kp_mag = desired_kp_mag;
}

void cf_ahrs_set_ki_gyro(float desired_ki_gyro) {
    ki_gyro = desired_ki_gyro;
}

void cf_ahrs_set_mag_vi(float desired_mag_vi[MSZ]) {
    mag_vi[0] = desired_mag_vi[0];
    mag_vi[1] = desired_mag_vi[1];
    mag_vi[2] = desired_mag_vi[2];

    float mag_vi_norm = lin_alg_v_norm(mag_vi);

    /* Kept here more to highlight the fact that it is important to check if 
     * the norm of a vector is zero for other use cases. Not really practical 
     * here, since we know the magnetic field should be non-zero in magnitude 
     */
    if (mag_vi_norm != 0.0) {
        lin_alg_v_scale((1.0 / mag_vi_norm), mag_vi);
    }
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

    /*************************************************************************/
    /* Magnetometers */
    mag_magnitude = lin_alg_v_norm(mag_vb);

    /* Normalize the magnetometer vector*/
    if (mag_magnitude != 0.0) {
        lin_alg_v_scale(1.0 / mag_magnitude, mag_vb);
    }

    if (QUATERNION_OR_DCM_BASED == 0) {

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
        //    lin_alg_m_v_mult(r_align, mag_vb, mag_vb_common); /* Align mag frame with acc frame */
        lin_alg_m_v_mult(r_hat_t, mag_vi, vi_hat);
        lin_alg_cross(mag_vb, vi_hat, w_meas_i);
        lin_alg_v_scale(kp_mag, w_meas_i); /* Weight the mag contribution */

        /* Continue the sum of elements for the correction vector */
        w_meas_sum[0] += w_meas_i[0];
        w_meas_sum[1] += w_meas_i[1];
        w_meas_sum[2] += w_meas_i[2];

        /* Estimated gyro bias */
        b[0] = -ki_gyro * w_meas_sum[0];
        b[1] = -ki_gyro * w_meas_sum[1];
        b[2] = -ki_gyro * w_meas_sum[2];

        /* Gyros plus the correction vector. Gyros sign is flipped to match
         * the sign of rotation with the reference vectors */

        q_gyro[0] = 0.0;
        q_gyro[1] = gyro_vb[0] - b[0] - w_meas_sum[0] + omega_bias[0];

        q_gyro[2] = gyro_vb[1] - b[1] - w_meas_sum[1] + omega_bias[1];

        q_gyro[3] = gyro_vb[2] - b[2] -w_meas_sum[2] + omega_bias[2];

        lin_alg_q_mult(q_est, q_gyro, q_est_dot);

        lin_alg_scale_q(0.5, q_est_dot);

        /* Primary integration */
        q_est[0] = q_est[0] + q_est_dot[0] * dt;
        q_est[1] = q_est[1] + q_est_dot[1] * dt;
        q_est[2] = q_est[2] + q_est_dot[2] * dt;
        q_est[3] = q_est[3] + q_est_dot[3] * dt;

        q_norm = lin_alg_q_norm(q_est);
        if (q_norm != 0.0) {
            q_magnitude = 1.0 / q_norm;
            
            lin_alg_scale_q(q_magnitude, q_est);

            lin_alg_q2euler_abs(q_est, &cf_yaw, &cf_pitch, &cf_roll);
        }

        *yaw = cf_yaw;
        *pitch = cf_pitch;
        *roll = cf_roll;

        //        lin_alg_q_inv(q_est, q_est_conj);
        //        lin_alg_q2euler_abs(q_est_conj,
        //                &cf_yaw_conj,
        //                &cf_pitch_conj,
        //                &cf_roll_conj);

        lin_alg_q2dcm(q_est, r_hat);
        
    } else {
        lin_alg_set_v(0.0, 0.0, 0.0, w_meas_sum); /* Clear the correction vector */
        
        /* Subtract long-term gyro biases */
        omega_vb[0] = gyro_vb[0] - omega_bias[0];
        omega_vb[1] = gyro_vb[1] - omega_bias[1];
        omega_vb[2] = gyro_vb[2] - omega_bias[2];
                
        /* Calculate rates based on the cross products of the inertial aiding 
         * vectors */
        
        /* Accelerometers */
        lin_alg_m_v_mult(R_minus, g_vi, vi_hat);
        lin_alg_cross(acc_vb, vi_hat, w_meas_acc);
        lin_alg_v_scale(kp_acc, w_meas_acc); /* Weight the mag contribution */
        
        /* Magnetometers */
        lin_alg_m_v_mult(R_minus, mag_vi, vi_hat);
        lin_alg_cross(mag_vi, vi_hat, w_meas_mag);
        lin_alg_v_scale(kp_mag, w_meas_mag); /* Weight the mag contribution */
        
        /* Feedback */
        omega_vb[0] = omega_vb[0] + w_meas_acc[0] + w_meas_mag[0];
        omega_vb[1] = omega_vb[1] + w_meas_acc[1] + w_meas_mag[1];
        omega_vb[2] = omega_vb[2] + w_meas_acc[2] + w_meas_mag[2];
        
        /* Integration using matrix exponential */
        //lin_alg_skew_sym
        //lin_alg_R_exp
    }
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