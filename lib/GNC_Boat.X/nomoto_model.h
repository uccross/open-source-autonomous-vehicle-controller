/*
 * File:   nomoto_model.h
 * Author: Pavlo Vlastos
 * Brief:  Nomoto (Augmented) ship steering model
 * Created on March 11, 2021, 1:48 PM
 */

#ifndef NOMOTO_MODEL_H
#define	NOMOTO_MODEL_H
/*******************************************************************************
 * #DEFINES
 ******************************************************************************/
#define DT 0.01 // Sample time
#define SSZ 5 // Maximum size of state and statically allocated arrays 
#define T_D_YAW -0.01 // Yaw-rate time-constant
#define K_D_YAW 0.9 // Yaw-rate gain 

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @function nomoto_init()
 * @param dt_desired The desired sample time in seconds
 */
void nomoto_init(float dt_desired);

/**
 * @function nomoto_update()
 * Multiplies a matrix with a vector
 * @param x_in The state vector at time step T(k)
 * @param u The control input
 * @param x_out The state vector at time step T(k+1)
 * @return SUCCESS or ERROR
 */
char nomoto_update(float x_in[SSZ], float u, float x_out[SSZ]);
/**
 * @function nomoto_mult_m_v()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
char nomoto_mult_m_v(float m[SSZ][SSZ], float v[SSZ], float v_out[SSZ]);

/**
 * @function nomoto_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void nomoto_v_v_add(float v1[SSZ], float v2[SSZ], float v_out[SSZ]);

/**
 * @function nomoto_s_v_mult()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out
 */
void nomoto_s_v_mult(float s, float v[SSZ], float v_out[SSZ]);

#endif	/* NOMOTO_MODEL_H */