/*
 * File:   nomoto_model.c
 * Author: Pavlo Vlastos
 * Brief:  Nomoto (Augmented) ship steering model
 * Created on March 11, 2021, 1:48 PM
 */

#include "nomoto_model.h"
#include "Lin_alg_float.h"
#include "Board.h"

/**
 * @function nomoto_update()
 * Multiplies a matrix with a vector
 * @param x_in The state vector at time step T(k)
 * @param u The control input
 * @param x_out The state vector at time step T(k+1)
 * @return SUCCESS or ERROR
 */
char nomoto_update(float x_in[SSZ], float u, float x_out[SSZ]) {
    
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
void lin_alg_v_v_add(float v1[SSZ], float v2[SSZ], float v_out[SSZ]) {
    int row;

    for (row = 0; row < SSZ; row++) {
        v_out[row] = v1[row] + v2[row];
    }
}
    