/*
 * File:   triad_ahrs.c
 * Author: Pavlo Vlastos
 *
 * Created on January 28, 2020, 12:47 PM
 */

/******************************************************************************
 * #INCLUDES
 *****************************************************************************/
#include <math.h>

#include "triad_ahrs.h"
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
/* NOTE: that these are declared here, but are not necessarily set to correct 
 * inertial values until the initialization. */

float dt = 0.01;

float w1[MSZ] = {0.0}; /* acc intermediate vector */
float w2[MSZ] = {0.0}; /* mag intermediate vector */
float v1[MSZ] = {0.0}; /* inertial intermediate vector */
float v2[MSZ] = {0.0}; /* inertial intermediate vector */
float o1[MSZ] = {0.0}; /* observation vector 1*/
float o2[MSZ] = {0.0}; /* observation vector 2*/
float o3[MSZ] = {0.0}; /* observation vector 3*/
float r1[MSZ] = {0.0}; /* reference vector 1*/
float r2[MSZ] = {0.0}; /* reference vector 2*/
float r3[MSZ] = {0.0}; /* reference vector 3*/

float v1Xv2[MSZ];
float v1Xv1Xv2[MSZ];

float v1Xv2_norm = 0.0;

float m_o[MSZ][MSZ];
float m_r[MSZ][MSZ];
float m_r_t[MSZ][MSZ];
float a_hat[MSZ][MSZ];

/* Euler Angles */
float cf_yaw = 0.0;
float cf_pitch = 0.0;
float cf_roll = 0.0;

/******************************************************************************
 * PUBLIC FUNCTIONS                                                          *
 *****************************************************************************/

void triad_ahrs_init(float desired_dt) {
    float mag_vi_norm = 1.0;
    
    dt = desired_dt;
    
    /* Set all inertial-frame aiding vectors */
    lin_alg_set_v(0.0, 0.0, 1.0, v1);
    
    
    /* See https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm 
     * for values (changes by 0.08 degrees each year) 
     *              East,    North,     Down            */
    lin_alg_set_v(5249.8, 22722.9, 41327.4, v2);
    
    /* Kept here more to highlight the fact that it is important to check if 
     * the norm of a vector is zero for other use cases. Not really practical 
     * here, since we know the magnetic field should be non-zero in magnitude 
     */
    mag_vi_norm = lin_alg_v_norm(v2);
    if (mag_vi_norm != 0.0) {
        lin_alg_v_scale((1.0 / mag_vi_norm), v2);
    }
    
    r1[0] = v1[0];
    r1[1] = v1[1];
    r1[2] = v1[2];
    
    lin_alg_cross(v1, v2, v1Xv2);
    
    v1Xv2_norm = lin_alg_v_norm(v1Xv2);
            
    r2[0] = v1Xv2[0];
    r2[1] = v1Xv2[1];
    r2[2] = v1Xv2[2];
    
    if (v1Xv2_norm != 0.0) {
        lin_alg_v_scale((1.0 / v1Xv2_norm), r2);
    }
    
    lin_alg_cross(v1, v1Xv2, v1Xv1Xv2);
            
    r3[0] = v1Xv1Xv2[0];
    r3[1] = v1Xv1Xv2[1];
    r3[2] = v1Xv1Xv2[2];
    
    if (v1Xv2_norm != 0.0) {
        lin_alg_v_scale((1.0 / v1Xv2_norm), r3);
    }
    
    /* Explicitly set all matrices to avoid nans */
    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            m_o);

    lin_alg_set_m(
            r1[0], r2[0], r3[0],
            r1[1], r2[1], r3[1],
            r1[2], r2[2], r3[2],
            m_r);
    
    lin_alg_m_transpose(m_r, m_r_t);
    
    lin_alg_set_m(
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            a_hat);
}

void triad_ahrs_update(float w1[MSZ], float w2[MSZ], float *yaw, 
        float *pitch, float *roll) {
    float wi_norm = 0.0;
    float w1Xw2[MSZ];
    float w1Xw1Xw2[MSZ];
    
    lin_alg_set_v(0.0, 0.0, 0.0, o1);
    lin_alg_set_v(0.0, 0.0, 0.0, o2);
    lin_alg_set_v(0.0, 0.0, 0.0, o3);
    
    lin_alg_set_v(0.0, 0.0, 0.0, r1);
    lin_alg_set_v(0.0, 0.0, 0.0, r2);
    lin_alg_set_v(0.0, 0.0, 0.0, r3);
    
    lin_alg_set_v(0.0, 0.0, 0.0, w1Xw2);
    lin_alg_set_v(0.0, 0.0, 0.0, w1Xw1Xw2);
    
    lin_alg_set_v(0.0, 0.0, 0.0, v1Xv2);
    lin_alg_set_v(0.0, 0.0, 0.0, v1Xv1Xv2);
    
    wi_norm = lin_alg_v_norm(w1);
    if (wi_norm != 0.0) {
        lin_alg_v_scale((1.0 / wi_norm), w1);
    }
    
    wi_norm = lin_alg_v_norm(w2);
    if (wi_norm != 0.0) {
        lin_alg_v_scale((1.0 / wi_norm), w2);
    }
    
    o1[0] = w1[0];
    o1[1] = w1[1];
    o1[2] = w1[2];
    
    lin_alg_cross(w1, w2, w1Xw2);
    
    o2[0] = w1Xw2[0];
    o2[1] = w1Xw2[1];
    o2[2] = w1Xw2[2];
    
    wi_norm = lin_alg_v_norm(w1Xw2);
    
    if (wi_norm != 0.0) {
        lin_alg_v_scale((1.0 / wi_norm), o2);
    }
    
    lin_alg_cross(w1, w1Xw2, w1Xw1Xw2);
    
    o3[0] = w1Xw1Xw2[0];
    o3[1] = w1Xw1Xw2[1];
    o3[2] = w1Xw1Xw2[2];
    
    if (wi_norm != 0.0) {
        lin_alg_v_scale((1.0 / wi_norm), o3);
    }
    
    lin_alg_set_m(
            o1[0], o2[0], o3[0],
            o1[1], o2[1], o3[1],
            o1[2], o2[2], o3[2],
            m_o);
    
    lin_alg_m_m_mult(m_o, m_r_t, a_hat);
    
    lin_alg_extract_angles(a_hat, yaw, pitch, roll);
}

/******************************************************************************
 * UNIT TEST(S)
 *****************************************************************************/
#ifdef TEST_TRIAD
#include "Board.h"

int main(void) {
//    Board_init();
    
    while(1);
    return 0;
}

#endif
