/*
 * File:   lin_alg_float.c
 * Author: Pavlo Vlastos
 * Acknowledgements:
 *     Some code is based on CMPE13 (now CSE/E 13) and Chris Seruge's Matrix 
 *     libraries
 * Created on February 6, 2020, 11:28 AM
 */

/*******************************************************************************
 * #INCLUDES
 ******************************************************************************/
#include "Lin_alg_float.h"
#include "Board.h"


#include <math.h>
#include <stdio.h>

/*******************************************************************************
 * PRIVATE #DEFINES
 ******************************************************************************/
#define RES 0.0001

/*******************************************************************************
 * PRIVATE DATATYPES 
 ******************************************************************************/

/*******************************************************************************
 * PRIVATE VARIABLES 
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

///**
// * @function correct()
// * Correct scaling and offset of a 3-component measurement by the accelerometer
// * or magnetometer
// * @param v_measure A vector array to hold accelerometer or magnetometer 
// *        [x y z]^T  measurement
// * @param a_est The scale factor matrix correction from tumble test. See 
// *        CalibrateEllipsoidData3D.m in repo
// * @param b_est The offset vector correction from tumble test. See 
// *        CalibrateEllipsoidData3D.m in repo
// * @param v_correct The corrected vector measurement
// * @return SUCCESS or ERROR
// */
//char correct(float v_measure[MSZ], float a_est[MSZ][MSZ], float b_est[MSZ],
//        float v_correct[MSZ]) {
//
//    lin_alg_m_v_mult(a_est, v_measure, v_correct);
//    lin_alg_v_v_add(v_correct, b_est, v_correct);
//
//}

/**
 * @function lin_alg_is_m_equal()
 * Compares to see if two matrices are equal to within the resolution (RES)
 * @param m1 A 3x3 matrix
 * @param m2 A 3x3 matrix
 * @return TRUE if matrices are equal, FALSE otherwise
 */
char lin_alg_is_m_equal(float m1[MSZ][MSZ], float m2[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            if (fabs(m1[row][col] - m2[row][col]) > RES) {
                return FALSE;
            }
        }
    }

    return TRUE;
}

/**
 * @function lin_alg_m_m_mult()
 * Multiplies two matricies together
 * @param m1 First matrix to be multiplied with second
 * @param m2 Second matrix to be multiplied with first
 * @param m_out The product of the two matrices.
 */
char lin_alg_m_m_mult(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {
    int row;
    int col;
    int i_sum;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = 0;
            for (i_sum = 0; i_sum < MSZ; i_sum++) {
                m_out[row][col] += (m1[row][i_sum] * m2[i_sum][col]);
            }
        }
    }
    return SUCCESS;
}

/**
 * @function lin_alg_m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
char lin_alg_m_v_mult(float m[MSZ][MSZ], float v[MSZ], float v_out[MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = 0;
        for (col = 0; col < MSZ; col++) {
            v_out[row] += m[row][col] * v[col];
        }
    }
    return SUCCESS;
}

/**
 * @function lin_alg_is_v_equal()
 * Compares to see if two vectors are equal to within the resolution (RES)
 * @param v1
 * @param v2
 * @return TRUE if equal and FALSE otherwise
 */
char lin_alg_is_v_equal(float v1[MSZ], float v2[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        if (fabs(v1[row] - v2[row]) > RES) {
            return FALSE;
        }
    }

    return TRUE;
}

/**
 * @function lin_alg_set_m()
 * @param matrixXX all respective entries to a 3x3 matrix
 * @param m The matrix for which each entry is set.
 */
void lin_alg_set_m(float matrix11, float matrix12, float matrix13,
        float matrix21, float matrix22, float matrix23, float matrix31,
        float matrix32, float matrix33, float m[MSZ][MSZ]) {

    m[0][0] = matrix11;
    m[0][1] = matrix12;
    m[0][2] = matrix13;

    m[1][0] = matrix21;
    m[1][1] = matrix22;
    m[1][2] = matrix23;

    m[2][0] = matrix31;
    m[2][1] = matrix32;
    m[2][2] = matrix33;
}

/**
 * @function lin_alg_set_v()
 * @param vectorX all respective entries to a 3x1 vector
 * @param v The vector for which each entry is set.
 */
void lin_alg_set_v(float vector1, float vector2, float vector3, float v[MSZ]) {
    v[0] = vector1;
    v[1] = vector2;
    v[2] = vector3;
}

/**
 * @function lin_alg_s_m_mult()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 * @param m_out Scaled matrix
 */
void lin_alg_s_m_mult(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = s * m[row][col];
        }
    }
}

/**
 * @function lin_alg_m_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
void lin_alg_m_scale(float s, float m[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m[row][col] *= s;
        }
    }
}

/**
 * @function lin_alg_s_v_mult()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out
 */
void lin_alg_s_v_mult(float s, float v[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = s * v[row];
    }
}

/**
 * @function lin_alg_v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void lin_alg_v_scale(float s, float v[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v[row] *= s;
    }
}

/**
 * @function lin_alg_s_m_mult()
 * Add a scalar value to a matrix
 * @param s Scalar to add to a matrix
 * @param m Matrix to have a scalar added to each element
 * @param m_out Matrix with added scalar
 */
void lin_alg_s_m_add(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = s + m[row][col];
        }
    }
}

/**
 * @function lin_alg_s_v_add()
 * Add a scalar value to a vector
 * @param s Scalar to add to a vector
 * @param v Vector to have a scalar added to each element
 * @param v_out Vector with added scalar
 */
void lin_alg_s_v_add(float s, float v[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = s + v[row];
    }
}

/**
 * @function lin_alg_m_m_add()
 * Add a matrix to a matrix
 * @param m1 Matrix to add to another matrix
 * @param m2 Matrix to have a matrix added to it
 * @param m_out Matrix as sum of two matrices
 */
void lin_alg_m_m_add(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {

    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m1[row][col] + m2[row][col];
        }
    }
}

/**
 * @function lin_alg_m_m_sub()
 * Add a matrix to a matrix
 * @param m1 Matrix to subtract to another matrix
 * @param m2 Matrix to have a matrix subtracted to it
 * @param m_out Matrix as difference of two matrices
 */
void lin_alg_m_m_sub(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]) {

    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m1[row][col] - m2[row][col];
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
void lin_alg_v_v_add(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
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
void lin_alg_v_v_sub(float v1[MSZ], float v2[MSZ], float v_out[MSZ]) {
    int row;

    for (row = 0; row < MSZ; row++) {
        v_out[row] = v1[row] - v2[row];
    }
}

/**
 * @function lin_alg_m_trace()
 * Sums the diagonals of a matrix. This sum is known as the trace.
 * @param m The matrix to have its trace calculated
 * @return The trace of the matrix.
 */
float lin_alg_m_trace(float m[MSZ][MSZ]) {
    float trace = 0;
    int i;

    for (i = 0; i < MSZ; i++) {
        trace += m[i][i];
    }

    return trace;
}

/**
 * @function lin_alg_m_det()
 * @param m The matrix for which a determinant is to be calculated
 * @return The determinant of the matrix
 */
float lin_alg_m_det(float m[MSZ][MSZ]) {
    float det = 0;
    det = m[0][0] * ((m[1][1] * m[2][2]) - (m[1][2] * m[2][1]))
            - m[0][1] * ((m[1][0] * m[2][2]) - (m[1][2] * m[2][0]))
            + m[0][2] * ((m[1][0] * m[2][1]) - (m[1][1] * m[2][0]));
    return det;
}

/**
 * @function lin_alg_m_transpose()
 * Generates the transpose of a matrix
 * @param m The matrix to be transposed
 * @param m_out The transpose of m
 */
void lin_alg_m_transpose(float m[MSZ][MSZ], float m_out[MSZ][MSZ]) {
    int row;
    int col;

    for (row = 0; row < MSZ; row++) {
        for (col = 0; col < MSZ; col++) {
            m_out[row][col] = m[col][row];
        }
    }
}

/**
 * @function lin_alg_skew_sym()
 * Generate the skew symmetric matrix of a (3x1) vector 
 * @param v A vector
 * @param m_out The resulting (3x3) skew matrix of v
 */
void lin_alg_skew_sym(float v[MSZ], float m_out[MSZ][MSZ]) {
    m_out[0][0] = 0.0;
    m_out[1][1] = 0.0;
    m_out[2][2] = 0.0;

    m_out[2][1] = v[0];
    m_out[2][0] = -v[1];
    m_out[1][0] = v[2];

    m_out[1][2] = -v[0];
    m_out[0][2] = v[1];
    m_out[0][1] = -v[2];
}

/**
 * @function lin_alg_anti_skew_pro()
 * Forms the antisymmetric projection matrix
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param dcm_t The transpose of the dcm
 * @param p_out The antisymmetric projection matrix
 */
void lin_alg_anti_sym_pro(float dcm[MSZ][MSZ], float dcm_t[MSZ][MSZ],
        float p_out[MSZ][MSZ]) {

    lin_alg_m_m_sub(dcm, dcm_t, p_out);

    lin_alg_m_scale(0.5, p_out);
}

/**
 * @function lin_alg_vex()
 * Change a skew matrix to a vector
 * @param m A skew matrix
 * @param v_out The corresponding vector
 */
void lin_alg_vex(float m[MSZ][MSZ], float v_out[MSZ]) {
    v_out[0] = m[2][1];
    v_out[1] = m[0][2];
    v_out[2] = m[1][0];
}

/**
 * @function lin_alg_v_norm()
 * Calculate the 2-norm, also known as the Euclidean length, or vector magnitude
 * @param v The vector for which the norm is to be calculated
 * @return The norm of v
 */
float lin_alg_v_norm(float v[MSZ]) {
    return ((float) sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2])));
}

/**
 * @function lin_alg_dot()
 * @param u A vector
 * @param v A vector
 * @return The dot product (sometimes called the inner product) of u and v
 */
float lin_alg_dot(float u[MSZ], float v[MSZ]) {
    return (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
}

/**
 * @function lin_alg_cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void lin_alg_cross(float u[MSZ], float v[MSZ], float w_out[MSZ]) {
    w_out[0] = u[1] * v[2] - u[2] * v[1];
    w_out[1] = u[2] * v[0] - u[0] * v[2];
    w_out[2] = u[0] * v[1] - u[1] * v[0];
}

/**
 * @function lin_alg_angle_from_2vecs()
 * @param u A vector
 * @param v A vector
 * @return The angle between the two vectors
 */
float lin_alg_angle_from_2vecs(float u[MSZ], float v[MSZ]) {
    return acos(lin_alg_dot(u, v) / (lin_alg_v_norm(u) * lin_alg_v_norm(v)));
}

/**
 * @function lin_alg_gen_dcm();
 * Generates a Direction Cosine Matrix for a rotation about a given axis and a
 * given angle. See Jack B. Kuibers book: "Quaternions and Rotation Sequences"
 * @param angle The angle of rotation about the axis of rotation unit-vector
 * @param v The unit-vector representing the axis of rotation
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @return SUCCESS or FAIL
 */
char lin_alg_gen_dcm(float angle, float v[MSZ], float dcm[MSZ][MSZ]) {

    // Check if the axis vector is a unit-vector
    if (fabs(lin_alg_v_norm(v) - 1.0) > RES) {
        return ERROR;
    }

    float ca = cos(angle);
    float sa = sin(angle);

    dcm[0][0] = v[0] * v[0] + (v[1] * v[1] + v[2] * v[2]) * ca;
    dcm[0][1] = v[0] * v[1]*(1 - ca) - v[2] * sa;
    dcm[0][2] = v[0] * v[2]*(1 - ca) + v[1] * sa;

    dcm[1][0] = v[0] * v[1]*(1 - ca) + v[2] * sa;
    dcm[1][1] = v[1] * v[1] + (v[2] * v[2] + v[0] * v[0]) * ca;
    dcm[1][2] = v[1] * v[2]*(1 - ca) - v[0] * sa;

    dcm[2][0] = v[2] * v[0]*(1 - ca) - v[1] * sa;
    dcm[2][1] = v[1] * v[2]*(1 - ca) + v[0] * sa;
    dcm[2][2] = v[2] * v[2] + (v[0] * v[0] + v[1] * v[1]) * ca;

    return SUCCESS;
}

/**
 * @function lin_alg_gen_dcm_with_angles();
 * Generates a Direction Cosine Matrix for a rotation about a given axis and a
 * given angle. See Jack B. Kuibers book: "Quaternions and Rotation Sequences"
 * @param psi Yaw angle in radians from -pi to pi
 * @param theta Pitch angle in radians from -pi to pi
 * @param phi Roll angle in radians from -pi to pi
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 */
void lin_alg_gen_dcm_with_angles(float psi, float theta, float phi,
        float dcm[MSZ][MSZ]) {
    float c_psi = cos(psi);
    float c_theta = cos(theta);
    float c_phi = cos(phi);

    float s_psi = sin(psi);
    float s_theta = sin(theta);
    float s_phi = sin(phi);

    dcm[0][0] = c_psi*c_theta;
    dcm[0][1] = s_psi*c_theta;
    dcm[0][2] = -s_theta;

    dcm[1][0] = c_psi * s_theta * s_phi - s_psi * c_phi;
    dcm[1][1] = s_psi * s_theta * s_phi + c_psi * c_phi;
    dcm[1][2] = c_theta * s_phi;

    dcm[2][0] = c_psi * s_theta * c_phi + s_psi * s_phi;
    dcm[2][1] = s_psi * s_theta * c_phi - c_psi * s_phi;
    dcm[2][2] = c_theta * c_phi;
}

/**
 * @function lin_alg_extract_angles();
 * Extract Euler angles from the DCM
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param psi A pointer to return the Yaw angle in radians from -pi to pi
 * @param theta A pointer to return the Pitch angle in radians from -pi to pi
 * @param phi A pointer to return the Roll angle in radians from -pi to pi
 * @return SUCCESS or FAIL
 */
char lin_alg_extract_angles(float dcm[MSZ][MSZ], float *psi, float *theta,
        float *phi) {
    *psi = atan2(dcm[1][0], dcm[0][0]); /* Yaw */

    *theta = asin(-dcm[2][0]); /* Pitch */

    *phi = atan2(dcm[2][1], dcm[2][2]); /* Roll */

    return SUCCESS;
}

/**
 * @function lin_alg_set_q()
 * @param psi Yaw angle in radians from -pi to pi
 * @param theta Pitch angle in radians from -pi to pi
 * @param phi Roll angle in radians from -pi to pi
 * @param q_out
 */
void lin_alg_set_q(float psi, float theta, float phi, float q_out[QSZ]) {
    float c_psi_2 = cos(psi / 2.0);
    float c_theta_2 = cos(theta / 2.0);
    float c_phi_2 = cos(phi / 2.0);

    float s_psi_2 = sin(psi / 2.0);
    float s_theta_2 = sin(theta / 2.0);
    float s_phi_2 = sin(phi / 2.0);

    q_out[0] = (c_psi_2 * c_theta_2 * c_phi_2 + s_psi_2 * s_theta_2 * s_phi_2);
    q_out[1] = (c_psi_2 * c_theta_2 * s_phi_2 - s_psi_2 * s_theta_2 * c_phi_2);
    q_out[2] = (c_psi_2 * s_theta_2 * c_phi_2 + s_psi_2 * c_theta_2 * s_phi_2);
    q_out[3] = (s_psi_2 * c_theta_2 * c_phi_2 - c_psi_2 * s_theta_2 * s_phi_2);
}

/**
 * @function lin_alg_q_inv()
 * Form the complex conjugate 
 * @param q_in A quaternion
 * @param q_out The complex conjugate, or inverse of q_in
 */
void lin_alg_q_inv(float q_in[QSZ], float q_out[QSZ]) {
    q_out[0] = q_in[0];
    q_out[1] = -q_in[1];
    q_out[2] = -q_in[2];
    q_out[3] = -q_in[3];
}

/**
 * @function lin_alg_scale_q()
 * Scale a quaternion
 * @param s A scalar
 * @param q A quaternion
 */
void lin_alg_scale_q(float s, float q[QSZ]) {
    q[0] *= s;
    q[1] *= s;
    q[2] *= s;
    q[3] *= s;
}

/**
 * @function lin_alg_q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
float lin_alg_q_norm(float q[QSZ]) {
    return ((float) sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]));
}

/**
 * @function lin_alg_q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void lin_alg_q_mult(float q[QSZ], float p[QSZ], float r[QSZ]) {
    r[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
    r[1] = p[1] * q[0] + p[0] * q[1] + p[3] * q[2] - p[2] * q[3];
    r[2] = p[2] * q[0] - p[3] * q[1] + p[0] * q[2] + p[1] * q[3];
    r[3] = p[3] * q[0] + p[2] * q[1] - p[1] * q[2] + p[0] * q[3];
}

/**
 * @function lin_alg_q2dcm()
 * Form a DCM from a quaternion
 * @param q A quaternion
 * @param dcm_out A direction cosine matrix (DCM)
 */
void lin_alg_q2dcm(float q[QSZ], float dcm_out[MSZ][MSZ]) {
    dcm_out[0][0] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[1] * q[1];
    dcm_out[0][1] = 2.0 * q[1] * q[2] + 2.0 * q[0] * q[3];
    dcm_out[0][2] = 2.0 * q[1] * q[3] - 2.0 * q[0] * q[2];

    dcm_out[1][0] = 2.0 * q[1] * q[2] - 2.0 * q[0] * q[3];
    dcm_out[1][1] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[2] * q[2];
    dcm_out[1][2] = 2.0 * q[2] * q[3] + 2.0 * q[0] * q[1];

    dcm_out[2][0] = 2.0 * q[1] * q[3] + 2.0 * q[0] * q[2];
    dcm_out[2][1] = 2.0 * q[2] * q[3] - 2.0 * q[0] * q[1];
    dcm_out[2][2] = 2.0 * q[0] * q[0] - 1.0 + 2.0 * q[3] * q[3];
}

/**
 * @function lin_alg_q2euler()
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler(float q[QSZ], float *psi, float *theta, float *phi) {
    *psi = atan((2.0 * q[1] * q[2] + 2.0 * q[0] * q[3]) /
            ((2.0 * q[0] * q[0] + 2.0 * q[1] * q[1] - 1.0)));

    *theta = asin(-(2.0 * q[1] * q[3] - 2.0 * q[0] * q[2]));

    *phi = atan((2.0 * q[2] * q[3] + 2.0 * q[0] * q[1]) /
            (2.0 * q[0] * q[0] + 2.0 * q[3] * q[3] - 1.0));
}

/**
 * @function lin_alg_q2euler_abs()
 * Return absolute Euler angles
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler_abs(float q[QSZ], float *psi, float *theta, float *phi) {
    *psi = atan2((2.0 * q[1] * q[2] + 2.0 * q[0] * q[3]),
            ((2.0 * q[0] * q[0] + 2.0 * q[1] * q[1] - 1.0)));

    *theta = asin(-(2.0 * q[1] * q[3] - 2.0 * q[0] * q[2]));

    *phi = atan2((2.0 * q[2] * q[3] + 2.0 * q[0] * q[1]),
            (2.0 * q[0] * q[0] + 2.0 * q[3] * q[3] - 1.0));
}

/**
 * @function lin_alg_rot_v_q()
 * Rotate a vector using quaternions
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param v_new
 */
void lin_alg_rot_v_q(float v[MSZ], float psi, float theta, float phi,
        float v_new[MSZ]) {
    float v_pure[QSZ];
    float v_temp[MSZ];
    float q[MSZ];
    float q_conj[MSZ];

    v_pure[0] = 0.0;
    v_pure[1] = v[0];
    v_pure[2] = v[1];
    v_pure[3] = v[2];

    lin_alg_set_q(psi, theta, phi, q);
    lin_alg_q_inv(q, q_conj);
    lin_alg_q_mult(q_conj, v_pure, v_temp);
    lin_alg_q_mult(v_temp, q, v_pure);

    v_new[0] = v_pure[1];
    v_new[1] = v_pure[2];
    v_new[2] = v_pure[3];
}

/**
 * @function lin_alg_m_print()
 * Print a matrix
 * @param matrix A matrix to print out
 */
void lin_alg_m_print(float matrix[MSZ][MSZ]) {
    int row;
    int col;

    printf("\r\n");
    for (row = 0; row < MSZ; row++) {
        printf("\r\n");
        for (col = 0; col < MSZ; col++) {
            printf("    % -9.4f", (double) matrix[row][col]);
        }
    }
    printf("\r\n");
}

/**
 * @function lin_alg_v_print()
 * Print a vector
 * @param vector A vector to print out
 */
void lin_alg_v_print(float vector[MSZ]) {
    int row;

    printf("\r\n");
    for (row = 0; row < MSZ; row++) {
        printf("\r\n    % -9.4f", (double) vector[row]);
    }
    printf("\r\n");
}

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/


/*******************************************************************************
 * MODULE UNIT TESTS
 ******************************************************************************/
#ifdef TEST_PACK
#define PAUSE 200000
#include "SerialM32.h"


int test_num_print(int i, int k);

int main(void) {
    Board_init();
    Serial_init();
    
    printf("\r\nLinear Algebra Test Harness %s. %s",__DATE__,__TIME__);

    int pass_count = 0;

    int count_lin_alg_is_m_equal = 0;
    int count_lin_alg_m_m_mult = 0;
    int count_lin_alg_is_v_equal = 0;
    int count_lin_alg_m_v_mult = 0;
    int count_lin_alg_s_m_mult = 0;
    int count_lin_alg_s_v_mult = 0;
    int count_lin_alg_s_m_add = 0;
    int count_lin_alg_s_v_add = 0;
    int count_lin_alg_m_m_add = 0;
    int count_lin_alg_v_v_add = 0;
    int count_lin_alg_m_trace = 0;
    int count_lin_alg_m_det = 0;
    int count_lin_alg_m_transpose = 0;
    int count_lin_alg_skew_sym = 0;
    int count_lin_alg_v_norm = 0;
    int count_lin_alg_gen_dcm = 0;
    int count_lin_alg_q2euler = 0;
    int count_lin_alg_gen_dcm_with_angles = 0;

    int pass_lin_alg_is_m_equal = 0;
    int pass_lin_alg_m_m_mult = 0;
    int pass_lin_alg_is_v_equal = 0;
    int pass_lin_alg_m_v_mult = 0;
    int pass_lin_alg_s_m_mult = 0;
    int pass_lin_alg_s_v_mult = 0;
    int pass_lin_alg_s_m_add = 0;
    int pass_lin_alg_s_v_add = 0;
    int pass_lin_alg_m_m_add = 0;
    int pass_lin_alg_v_v_add = 0;
    int pass_lin_alg_m_trace = 0;
    int pass_lin_alg_m_det = 0;
    int pass_lin_alg_m_transpose = 0;
    int pass_lin_alg_skew_sym = 0;
    int pass_lin_alg_v_norm = 0;
    int pass_lin_alg_gen_dcm = 0;
    int pass_lin_alg_q2euler = 0;
    int pass_lin_alg_gen_dcm_with_angles = 0;

    int total = 0;
    char correct_answer;

    int i;
    int j = 0;

    float m1[MSZ][MSZ] = {
        {-1.1480, 2.5855, -0.0825},
        {0.1049, -0.6669, -1.9330},
        {0.7223, 0.1873, -0.4390}
    };
    float m2[MSZ][MSZ] = {
        {-1.7947, 0.1001, -0.6003},
        {0.8404, -0.5445, 0.4900},
        {-0.8880, 0.3035, 0.7394}
    };
    float m_answer[MSZ][MSZ] = {
        {1.6372, -4.8194, 0.2181},
        {-0.6680, 2.6278, 0.7681},
        {1.5853, -2.3598, -0.8380}
    };
    float m_result[MSZ][MSZ];
    /***************************************************************************
     * TEST: lin_alg_is_m_equal()
     **************************************************************************/
    count_lin_alg_is_m_equal++;
    total++;
    correct_answer = TRUE;
    if (lin_alg_is_m_equal(m1, m1) == correct_answer) {
        pass_lin_alg_is_m_equal++;
        printf("\r\nSUCCESS: Test %d lin_alg_is_m_equal() correctly passes",
                total);
    } else {
        printf("\r\n         Test %d lin_alg_is_m_equal() correctly passes",
                total);
    }
    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    for (i = 0; i < PAUSE; i++);
    /**************************************************************************/
    count_lin_alg_is_m_equal++;
    total++;
    correct_answer = FALSE;
    if (lin_alg_is_m_equal(m1, m2) == correct_answer) {
        pass_lin_alg_is_m_equal++;
        printf("\r\nSUCCESS: Test %d lin_alg_is_m_equal() correctly fails", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_is_m_equal() correctly fails", total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);
    printf("\r\n    Second matrix:");
    lin_alg_m_print(m2);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_m_mult() 
     **************************************************************************/
    count_lin_alg_m_m_mult++;
    total++;
    lin_alg_set_m(-1.7947, 0.1001, -0.6003,
            0.8404, -0.5445, 0.4900,
            -0.8880, 0.3035, 0.7394, m1);
    lin_alg_set_m(-1.1480, 2.5855, -0.0825,
            0.1049, -0.6669, -1.9330,
            0.7223, 0.1873, -0.4390, m2);
    lin_alg_set_m(1.6372, -4.8194, 0.2181,
            -0.6680, 2.6278, 0.7681,
            1.5853, -2.3598, -0.8380, m_answer);
    lin_alg_m_m_mult(m1, m2, m_result);
    correct_answer = TRUE;
    if (lin_alg_is_m_equal(m_result, m_answer) == correct_answer) {
        pass_lin_alg_m_m_mult++;
        printf("\r\nSUCCESS: Test %d lin_alg_is_m_equal() correctly fails", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_is_m_equal() correctly fails", total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    Second matrix:");
    lin_alg_m_print(m2);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_is_v_equal()
     **************************************************************************/
    count_lin_alg_is_v_equal++;
    total++;
    float v1[] = {1, 2, 3};
    float v2[] = {2.0, 4.0, 6.0};
    float v_result[MSZ];
    float v_answer[] = {2.0, 4.0, 6.0};
    correct_answer = TRUE;
    if (lin_alg_is_v_equal(v1, v1) == correct_answer) {
        pass_lin_alg_is_v_equal++;
        printf("\r\nSUCCESS: Test %d lin_alg_is_v_equal() correctly passes",
                total);
    } else {
        printf("\r\n         Test %d lin_alg_is_v_equal() correctly passes",
                total);
    }
    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    for (i = 0; i < PAUSE; i++);
    /**************************************************************************/
    count_lin_alg_is_v_equal++;
    total++;
    correct_answer = FALSE;
    if (lin_alg_is_v_equal(v1, v2) == correct_answer) {
        pass_lin_alg_is_v_equal++;
        printf("\r\nSUCCESS: Test %d lin_alg_is_v_equal() correctly fails", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_is_v_equal() correctly fails", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);
    printf("\r\n    Second vector:");
    lin_alg_v_print(v2);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_v_mult() 
     **************************************************************************/
    count_lin_alg_m_v_mult++;
    total++;
    lin_alg_set_m(1.7119, -0.8396, 0.9610,
            -0.1941, 1.3546, 0.1240,
            -2.1384, -1.0722, 1.4367, m1);
    lin_alg_set_v(-1.9609, -0.1977, -1.2078, v1);
    lin_alg_set_v(-4.3516, -0.0370, 2.6699, v_answer);
    lin_alg_m_v_mult(m1, v1, v_result);
    correct_answer = TRUE;
    if (lin_alg_is_v_equal(v_result, v_answer) == correct_answer) {
        pass_lin_alg_m_v_mult++;
        printf("\r\nSUCCESS: Test %d lin_alg_m_v_mult() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_m_v_mult() correctly passes",
                total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Expected Answer:");
    lin_alg_v_print(v_answer);

    printf("\r\n    Result:");
    lin_alg_v_print(v_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_s_m_mult() 
     **************************************************************************/
    count_lin_alg_s_m_mult++;
    total++;

    lin_alg_set_m(-0.2620, -0.8314, -0.5336,
            -1.7502, -0.9792, -2.0026,
            -0.2857, -1.1564, 0.9642, m1);
    float scalar = 2.0;
    lin_alg_set_m(-0.5240, -1.6628, -1.0672,
            -3.5004, -1.9584, -4.0052,
            -0.5714, -2.3128, 1.9284, m_answer);

    lin_alg_s_m_mult(scalar, m1, m_result);

    if (lin_alg_is_m_equal(m_result, m_answer) == TRUE) {
        pass_lin_alg_s_m_mult++;
        printf("\r\nSUCCESS: Test %d lin_alg_s_m_mult() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_s_m_mult() correctly passes",
                total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    Scalar: %f", (double) scalar);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);

    /***************************************************************************
     * TEST: lin_alg_s_v_mult() 
     **************************************************************************/
    count_lin_alg_s_v_mult++;
    total++;

    lin_alg_set_v(2.9080, 0.8252, 1.3790, v1);
    scalar = 2.0;
    lin_alg_set_v(5.8160, 1.6504, 2.7580, v_answer);

    lin_alg_s_v_mult(scalar, v1, v_result);

    if (lin_alg_is_v_equal(v_result, v_answer) == TRUE) {
        pass_lin_alg_s_v_mult++;
        printf("\r\nSUCCESS: Test %d lin_alg_s_v_mult() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_s_v_mult() correctly passes",
                total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Scalar: %f", (double) scalar);

    printf("\r\n    Expected Answer:");
    lin_alg_v_print(v_answer);

    printf("\r\n    Result:");
    lin_alg_v_print(v_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_s_m_add() 
     **************************************************************************/
    count_lin_alg_s_m_add++;
    total++;
    lin_alg_set_m(2.7694, 0.7254, -0.2050,
            -1.3499, -0.0631, -0.1241,
            3.0349, 0.7147, 1.4897, m1);
    lin_alg_set_m(4.7694, 2.7254, 1.7950,
            0.6501, 1.9369, 1.8759,
            5.0349, 2.7147, 3.4897, m_answer);
    lin_alg_set_m(0, 0, 0, 0, 0, 0, 0, 0, 0, m_result);

    lin_alg_s_m_add(scalar, m1, m_result);

    correct_answer = TRUE;
    if (lin_alg_is_m_equal(m_result, m_answer) == correct_answer) {
        pass_lin_alg_s_m_add++;
        printf("\r\nSUCCESS: Test %d lin_alg_s_m_add() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_s_m_add() correctly passes",
                total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    Scalar: %f", (double) scalar);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_s_v_add() 
     **************************************************************************/
    count_lin_alg_s_v_add++;
    total++;
    lin_alg_set_v(1.4090, 1.4172, 0.6715, v1);
    lin_alg_set_v(4.4090, 4.4172, 3.6715, v_answer);

    scalar = 3.0;

    lin_alg_s_v_add(scalar, v1, v_result);

    correct_answer = TRUE;
    if (lin_alg_is_v_equal(v_result, v_answer) == correct_answer) {
        pass_lin_alg_s_v_add++;
        printf("\r\nSUCCESS: Test %d lin_alg_s_v_add() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_s_v_add() correctly passes",
                total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Scalar: %f\r\n", (double) scalar);

    printf("\r\n    Expected Answer:");
    lin_alg_v_print(v_answer);

    printf("\r\n    Result:");
    lin_alg_v_print(v_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_m_add() 
     **************************************************************************/
    count_lin_alg_m_m_add++;
    total++;
    lin_alg_set_m(-1.7947, 0.1001, -0.6003,
            0.8404, -0.5445, 0.4900,
            -0.8880, 0.3035, 0.7394, m1);
    lin_alg_set_m(-1.1480, 2.5855, -0.0825,
            0.1049, -0.6669, -1.9330,
            0.7223, 0.1873, -0.4390, m2);
    lin_alg_set_m(-2.9427, 2.6856, -0.6828,
            0.9453, -1.2114, -1.4430,
            -0.1657, 0.4908, 0.3004, m_answer);

    lin_alg_m_m_add(m1, m2, m_result);

    correct_answer = TRUE;
    if (lin_alg_is_m_equal(m_result, m_answer) == correct_answer) {
        pass_lin_alg_m_m_add++;
        printf("\r\nSUCCESS: Test %d lin_alg_m_m_add() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_m_m_add() correctly passes", total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    Second matrix:");
    lin_alg_m_print(m2);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_v_v_add() 
     **************************************************************************/
    count_lin_alg_v_v_add++;
    total++;
    lin_alg_set_v(-1.2075, 0.4889, -0.3034, v1);
    lin_alg_set_v(0.8884, -0.8095, 0.3252, v2);
    lin_alg_set_v(-0.3191, -0.3206, 0.0217, v_answer);

    lin_alg_v_v_add(v1, v2, v_result);

    correct_answer = TRUE;
    if (lin_alg_is_v_equal(v_result, v_answer) == correct_answer) {
        pass_lin_alg_v_v_add++;
        printf("\r\nSUCCESS: Test %d lin_alg_v_v_add() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_v_v_add() correctly passes", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Second vector:");
    lin_alg_v_print(v2);

    printf("\r\n    Expected Answer:");
    lin_alg_v_print(v_answer);

    printf("\r\n    Result:");
    lin_alg_v_print(v_result);

    /**************************************************************************/
    count_lin_alg_v_v_add++;
    total++;
    lin_alg_set_v(-1.2075, 0.4889, -0.3034, v1);
    lin_alg_set_v(0.8884, -0.8095, 0.3252, v2);
    lin_alg_set_v(-0.3191, -0.3206, 0.0217, v_answer);

    lin_alg_v_v_add(v1, v2, v_result);

    correct_answer = TRUE;
    if (lin_alg_is_v_equal(v_result, v_answer) == correct_answer) {
        pass_lin_alg_v_v_add++;
        printf("\r\nSUCCESS: Test %d lin_alg_v_v_add() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_v_v_add() correctly passes", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Second vector:");
    lin_alg_v_print(v2);

    printf("\r\n    Expected Answer:");
    lin_alg_v_print(v_answer);

    printf("\r\n    Result:");
    lin_alg_v_print(v_result);


    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_trace() 
     **************************************************************************/
    count_lin_alg_m_trace++;
    total++;
    lin_alg_set_m(-1.7115, 0.3192, -0.0301,
            -0.1022, 0.3129, -0.1649,
            -0.2414, -0.8649, 0.6277, m1);
    float calc_answer = -0.7710;
    float calc_result = 0;
    calc_result = lin_alg_m_trace(m1);

    if (fabs(calc_result - calc_answer) <= RES) {
        pass_lin_alg_m_trace++;
        printf("\r\nSUCCESS: Test %d lin_alg_m_trace() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_m_trace() correctly passes", total);
    }
    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);
    printf("\r\n    Expected Answer: %f", (double) calc_answer);
    printf("\r\n    Result: %f\r\n", (double) calc_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_det() 
     **************************************************************************/
    count_lin_alg_m_det++;
    total++;
    lin_alg_set_m(1.0933, 0.0774, -0.0068,
            1.1093, -1.2141, 1.5326,
            -0.8637, -1.1135, -0.7697, m1);
    calc_answer = 2.8667;

    calc_result = lin_alg_m_det(m1);

    if (fabs(calc_result - calc_answer) <= RES) {
        pass_lin_alg_m_det++;
        printf("\r\nSUCCESS: Test %d lin_alg_m_det() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_m_det() correctly passes",
                total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);
    printf("\r\n    Expected Answer: %f", (double) calc_answer);
    printf("\r\n    Result: %f\r\n", (double) calc_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_m_transpose() 
     **************************************************************************/
    count_lin_alg_m_transpose++;
    total++;
    lin_alg_set_m(0.3714, -1.0891, 1.1006,
            -0.2256, 0.0326, 1.5442,
            1.1174, 0.5525, 0.0859, m1);
    lin_alg_set_m(0.3714, -0.2256, 1.1174,
            -1.0891, 0.0326, 0.5525,
            1.1006, 1.5442, 0.0859, m_answer);

    lin_alg_m_transpose(m1, m_result);

    if (lin_alg_is_m_equal(m_result, m_answer) == TRUE) {
        pass_lin_alg_m_transpose++;
        printf("\r\nSUCCESS: Test %d lin_alg_m_transpose() correctly passes",
                total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_m_transpose() correctly passes",
                total);
    }

    printf("\r\n    First matrix:");
    lin_alg_m_print(m1);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_skew_sym() 
     **************************************************************************/
    count_lin_alg_skew_sym++;
    total++;
    lin_alg_set_v(-1.4023, -1.4224, 0.4882, v1);
    lin_alg_set_m(0, -0.4882, -1.4224,
            0.4882, 0, 1.4023,
            1.4224, -1.4023, 0, m_answer);

    lin_alg_skew_sym(v1, m_result);

    if (lin_alg_is_m_equal(m_result, m_answer) == TRUE) {
        pass_lin_alg_skew_sym++;
        printf("\r\nSUCCESS: Test %d lin_alg_skew_sym() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_skew_sym() correctly passes", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_v_norm() 
     **************************************************************************/
    count_lin_alg_v_norm++;
    total++;
    lin_alg_set_v(-0.1774, -0.1961, 1.4193, v1);
    calc_answer = 1.4437;

    calc_result = lin_alg_v_norm(v1);

    if (fabs(calc_result - calc_answer) <= RES) {
        pass_lin_alg_v_norm++;
        printf("\r\nSUCCESS: Test %d lin_alg_v_norm() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_v_norm() correctly passes", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v1);

    printf("\r\n    Expected Answer: %f", (double) calc_answer);
    printf("\r\n    Result: %f\r\n", (double) calc_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_gen_dcm()
     **************************************************************************/
    count_lin_alg_gen_dcm++;
    total++;

    float angle = (M_PI / 3.0); // 60 degrees

    lin_alg_set_v(1.0, 2.0, 3.0, v1);
    lin_alg_set_m(0.5357, -0.6229, 0.5700,
            0.7658, 0.6428, -0.0172,
            -0.3557, 0.4458, 0.8214, m_answer);
    scalar = (1.0 / lin_alg_v_norm(v1));
    lin_alg_s_v_mult(scalar, v1, v_result);

    lin_alg_gen_dcm(angle, v_result, m_result);

    if (lin_alg_is_m_equal(m_result, m_answer) == TRUE) {
        pass_lin_alg_gen_dcm++;
        printf("\r\nSUCCESS: Test %d lin_alg_gen_dcm() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_gen_dcm() correctly passes", total);
    }


    printf("\r\n    First vector:");
    lin_alg_v_print(v_result);

    printf("\r\n    First angle: %f\r\n", (double) angle);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    /**************************************************************************/
    count_lin_alg_gen_dcm++;
    total++;

    angle = (M_PI / 2.0); // 90 degrees

    lin_alg_set_m(0.0714, -0.6589, 0.7488,
            0.9447, 0.2857, 0.1613,
            -0.3202, 0.6959, 0.6429, m_answer);
    lin_alg_gen_dcm(angle, v_result, m_result);

    if (lin_alg_is_m_equal(m_result, m_answer) == TRUE) {
        pass_lin_alg_gen_dcm++;
        printf("\r\nSUCCESS: Test %d lin_alg_gen_dcm() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_gen_dcm() correctly passes", total);
    }


    printf("\r\n    First vector:");
    lin_alg_v_print(v_result);

    printf("\r\n    First angle: %f\r\n", (double) angle);

    printf("\r\n    Expected Answer:");
    lin_alg_m_print(m_answer);

    printf("\r\n    Result:");
    lin_alg_m_print(m_result);

    for (i = 0; i < PAUSE; i++);


    /**************************************************************************/
    count_lin_alg_gen_dcm++;
    total++;

    // Test to see  if lin_alg_gen_dcm() errors out when a non-unit-vector is given
    lin_alg_s_v_mult(1.5, v1, v_result);

    if (lin_alg_gen_dcm(angle, v_result, m_result) == ERROR) {
        pass_lin_alg_gen_dcm++;
        printf("\r\nSUCCESS: Test %d lin_alg_gen_dcm() correctly fails", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_gen_dcm() correctly fails", total);
    }

    printf("\r\n    First vector:");
    lin_alg_v_print(v_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_q2euler()
     **************************************************************************/
    count_lin_alg_q2euler++;
    total++;

    float psi = (60.0 * M_PI / 180.0);
    float theta = (40.0 * M_PI / 180.0);
    float phi = (20.0 * M_PI / 180.0);

    float psi_result = 0.0;
    float theta_result = 0.0;
    float phi_result = 0.0;

    float dcm[MSZ][MSZ];
    lin_alg_set_m(0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, dcm);

    float q[QSZ] = {0.0};
    lin_alg_set_q(psi, theta, phi, q);
    lin_alg_q2euler(q, &psi_result, &theta_result, &phi_result);

    if ((psi - psi_result < RES) && (theta - theta_result < RES) &&
            (phi - phi_result < RES)) {
        pass_lin_alg_q2euler++;
        printf("\r\nSUCCESS: Test %d lin_alg_q2euler() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_q2euler() correctly passes", total);
    }

    printf("\r\n    Expected Answer:");
    printf("\r\n    psi: %f", (double) psi);
    printf("\r\n    theta: %f", (double) theta);
    printf("\r\n    phi: %f\r\n", (double) phi);

    printf("\r\n    Result:");
    printf("\r\n    psi_result: %f", (double) psi_result);
    printf("\r\n    theta_result: %f", (double) theta_result);
    printf("\r\n    phi_result: %f", (double) phi_result);

    for (i = 0; i < PAUSE; i++);
    /***************************************************************************
     * TEST: lin_alg_q2euler()
     **************************************************************************/
    count_lin_alg_gen_dcm_with_angles++;
    total++;
    float dcma[MSZ][MSZ];
    lin_alg_set_m(0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, dcma);
    lin_alg_q2dcm(q, dcma);
    
    lin_alg_set_m(0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, dcm);

    lin_alg_gen_dcm_with_angles(psi, theta, phi, dcm);
    
    if (lin_alg_is_m_equal(dcma, dcm) == TRUE){
        pass_lin_alg_gen_dcm_with_angles++;
        printf("\r\nSUCCESS: Test %d lin_alg_gen_dcm_with_angles() correctly passes", total);
    } else {
        printf("\r\nFAIL:    Test %d lin_alg_gen_dcm_with_angles() correctly passes", total);
    }
    
    printf("\r\n     DCM from quaternion");
    lin_alg_m_print(dcma);
    printf("\r\n     DCM from angles");
    lin_alg_m_print(dcm);
    
    

    for (i = 0; i < PAUSE; i++);

    /***************************************************************************
     * Test scores
     **************************************************************************/
    i = 0;
    i += test_num_print(i, pass_lin_alg_is_m_equal);
    printf("%d / %d Passed: lin_alg_is_m_equal", pass_lin_alg_is_m_equal,
            count_lin_alg_is_m_equal);
    i += test_num_print(i, pass_lin_alg_m_m_mult);
    printf("%d / %d Passed: lin_alg_m_m_mult", pass_lin_alg_m_m_mult,
            count_lin_alg_m_m_mult);
    i += test_num_print(i, pass_lin_alg_is_v_equal);
    printf("%d / %d Passed: lin_alg_is_v_equal", pass_lin_alg_is_v_equal,
            count_lin_alg_is_v_equal);
    i += test_num_print(i, pass_lin_alg_m_v_mult);
    printf("%d / %d Passed: lin_alg_m_v_mult", pass_lin_alg_m_v_mult,
            count_lin_alg_m_v_mult);
    i += test_num_print(i, pass_lin_alg_s_m_mult);
    printf("%d / %d Passed: lin_alg_s_m_mult", pass_lin_alg_s_m_mult,
            count_lin_alg_s_m_mult);
    i += test_num_print(i, pass_lin_alg_s_v_mult);
    printf("%d / %d Passed: lin_alg_s_v_mult", pass_lin_alg_s_v_mult,
            count_lin_alg_s_v_mult);
    i += test_num_print(i, pass_lin_alg_s_m_add);
    printf("%d / %d Passed: lin_alg_s_m_add", pass_lin_alg_s_m_add,
            count_lin_alg_s_m_add);
    i += test_num_print(i, pass_lin_alg_s_v_add);
    printf("%d / %d Passed: lin_alg_s_v_add", pass_lin_alg_s_v_add,
            count_lin_alg_s_v_add);
    i += test_num_print(i, pass_lin_alg_m_m_add);
    printf("%d / %d Passed: lin_alg_m_m_add", pass_lin_alg_m_m_add,
            count_lin_alg_m_m_add);
    for (j = 0; j < PAUSE; j++);
    i += test_num_print(i, pass_lin_alg_v_v_add);
    printf("%d / %d Passed: lin_alg_v_v_add", pass_lin_alg_v_v_add,
            count_lin_alg_v_v_add);
    i += test_num_print(i, pass_lin_alg_m_trace);
    printf("%d / %d Passed: lin_alg_m_trace", pass_lin_alg_m_trace, count_lin_alg_m_trace);
    i += test_num_print(i, pass_lin_alg_m_det);
    printf("%d / %d Passed: lin_alg_m_det", pass_lin_alg_m_det,
            count_lin_alg_m_det);
    i += test_num_print(i, pass_lin_alg_m_transpose);
    printf("%d / %d Passed: lin_alg_m_transpose", pass_lin_alg_m_transpose,
            count_lin_alg_m_transpose);
    i += test_num_print(i, pass_lin_alg_skew_sym);
    printf("%d / %d Passed: lin_alg_skew_sym", pass_lin_alg_skew_sym, count_lin_alg_skew_sym);
    i += test_num_print(i, pass_lin_alg_v_norm);
    printf("%d / %d Passed: lin_alg_v_norm", pass_lin_alg_v_norm, count_lin_alg_v_norm);
    i += test_num_print(i, pass_lin_alg_gen_dcm);
    printf("%d / %d Passed: lin_alg_gen_dcm", pass_lin_alg_gen_dcm, count_lin_alg_gen_dcm);
    i += test_num_print(i, pass_lin_alg_gen_dcm);
    printf("%d / %d Passed: lin_alg_q2euler", pass_lin_alg_q2euler, count_lin_alg_q2euler);
    i += test_num_print(i, pass_lin_alg_gen_dcm_with_angles);
    printf("%d / %d Passed: lin_alg_gen_dcm_with_angles", pass_lin_alg_gen_dcm_with_angles, count_lin_alg_gen_dcm_with_angles);

    pass_count = (pass_lin_alg_is_m_equal +
            pass_lin_alg_m_m_mult +
            pass_lin_alg_is_v_equal +
            pass_lin_alg_m_v_mult +
            pass_lin_alg_s_m_mult +
            pass_lin_alg_s_v_mult +
            pass_lin_alg_s_m_add +
            pass_lin_alg_s_v_add +
            pass_lin_alg_m_m_add +
            pass_lin_alg_v_v_add +
            pass_lin_alg_m_trace +
            pass_lin_alg_m_det +
            pass_lin_alg_m_transpose +
            pass_lin_alg_skew_sym +
            pass_lin_alg_v_norm +
            pass_lin_alg_gen_dcm +
            pass_lin_alg_q2euler +
            pass_lin_alg_gen_dcm_with_angles);

    printf("\r\n\r\n%d / %d Tests passed\r\n", pass_count, total);

    while (1);

    return 1;
}

int test_num_print(int i, int k) {
    if (k > 1) {
        i++;
        printf("\r\nTest %02d - %02d :: ", i, (i + k - 1));
    } else {
        printf("\r\nTest %02d      :: ", (k + i));
    }
    return k;
}

#endif