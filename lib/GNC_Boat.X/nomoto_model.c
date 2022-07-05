/*
 * File:   nomoto_model.c
 * Author: Pavlo Vlastos
 * Brief:  Nomoto (Augmented) ship steering model
 * Created on March 11, 2021, 1:48 PM
 */

/******************************************************************************
 * #INCLUDES
 *****************************************************************************/
#include "nomoto_model.h"
#include "Lin_alg_float.h"
#include "Board.h"
#include <math.h>

/******************************************************************************
 * PRIVATE #DEFINES
 *****************************************************************************/

/******************************************************************************
 * PRIVATE DATATYPES 
 *****************************************************************************/
static float A[SSZ][SSZ] = {
    {1.0, DT, 0.0, 0.0, 0.0},
    {0.0, T_D_YAW, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0}
};

static float B[SSZ] = {
    0.0,
    K_D_YAW,
    0.0,
    0.0,
    0.0
};

static float x_partial[SSZ] = {0.0};
static float u_partial[SSZ] = {0.0};

static float dt = 0.0;

/******************************************************************************
 * Public Function Implementations
 *****************************************************************************/

/**
 * @function nomoto_init()
 * @param dt_desired The desired sample time in seconds
 */
void nomoto_init(float dt_desired) {
    dt = dt_desired;
}

/**
 * @function nomoto_update()
 * Multiplies a matrix with a vector
 * @param x_in The state vector at time step T(k)
 * @param u The control input
 * @param x_out The state vector at time step T(k+1)
 * @return SUCCESS or ERROR
 */
char nomoto_update(float x_in[SSZ], float u, float x_out[SSZ]) {
    float psi;
    
    /* Angle wrap heading angle */
    x_in[0] = fmod((x_in[0] + M_PI), TWO_PI) ;
    if (x_in[0] < 0.0) {
        x_in[0] += TWO_PI;
    }
    x_in[0] -= M_PI;
    
    psi = x_in[0];
    
    /* Update A matrix since it is non-linear */
    A[2][4] = dt*sin(psi);
    A[3][4] = dt*cos(psi);
    
    /* Ax_{k} */
    nomoto_mult_m_v(A, x_in, x_partial);
    
    /* Bu_{k} */
    nomoto_s_v_mult(u, B, u_partial);
    
    /* x_{k+1} = Ax_{k} + Bu_{k}*/
    nomoto_v_v_add(x_partial, u_partial, x_out);
    
    /* Angle wrap heading angle */
    x_out[0] = fmod((x_out[0] + M_PI), TWO_PI) ;
    if (x_out[0] < 0.0) {
        x_out[0] += TWO_PI;
    }
    x_out[0] -= M_PI;
    return SUCCESS;
}

/**
 * @function nomoto_mult_m_v()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
char nomoto_mult_m_v(float m[SSZ][SSZ], float v[SSZ], float v_out[SSZ]) {
    int row;
    int col;

    for (row = 0; row < SSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < SSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
    return SUCCESS;
}


/**
 * @function nomoto_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void nomoto_v_v_add(float v1[SSZ], float v2[SSZ], float v_out[SSZ]) {
    int row;

    for (row = 0; row < SSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}

/**
 * @function nomoto_s_v_mult()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out
 */
void nomoto_s_v_mult(float s, float v[SSZ], float v_out[SSZ]) {
    int row;

    for (row = 0; row < SSZ; row++) {
        v_out[row] = s * v[row];
    }
}