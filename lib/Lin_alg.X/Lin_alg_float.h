/* 
 * File:   lin_alg_float.h
 * Author: Pavlo Vlastos, Aaron Hunter
 *
 * Created on February 6, 2020, 11:31 AM
 * Modified on June 16,2021, 3;14 PM
 */

#ifndef LIN_ALG_FLOAT_H
#define	LIN_ALG_FLOAT_H

/*******************************************************************************
 * #DEFINES
 ******************************************************************************/
#define MSZ 3 // Maximum size of statically allocated arrays 
#define QSZ 4

/*******************************************************************************
 * PUBLIC DATATYPES 
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC VARIABLES 
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
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
//        float v_correct[MSZ]);

/**
 * @function lin_alg_is_m_equal()
 * Compares to see if two matrices are equal to within the resolution (RES)
 * @param m1 A 3x3 matrix
 * @param m2 A 3x3 matrix
 * @return TRUE if matrices are equal, FALSE otherwise
 */
char lin_alg_is_m_equal(float m1[MSZ][MSZ], float m2[MSZ][MSZ]);

/**
 * @function lin_alg_m_m_mult()
 * Multiplies two matricies together
 * @param m1 First matrix to be multiplied with second
 * @param m2 Second matrix to be multiplied with first
 * @param m_out The product of the two matrices.
 */
char lin_alg_m_m_mult(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_m_v_mult()
 * Multiplies a matrix with a vector
 * @param m A matrix to be multiplied with a vector
 * @param v A vector to be multiplied with a matrix
 * @param v_out The product of the matrix and vector
 * @return SUCCESS or ERROR
 */
char lin_alg_m_v_mult(float m[MSZ][MSZ], float v[MSZ], float v_out[MSZ]);


/**
 * @function lin_alg_is_v_equal()
 * Compares to see if two vectors are equal to within the resolution (RES)
 * @param v1
 * @param v2
 * @return TRUE if equal and FALSE otherwise
 */
char lin_alg_is_v_equal(float v1[MSZ], float v2[MSZ]);

/**
 * @function lin_alg_set_m()
 * @param matrixXX all respective entries to a 3x3 matrix
 * @param m The matrix for which each entry is set.
 */
void lin_alg_set_m(float matrix11, float matrix12, float matrix13,
        float matrix21, float matrix22, float matrix23, float matrix31,
        float matrix32, float matrix33, float m[MSZ][MSZ]);

/**
 * @function lin_alg_set_v()
 * @param vectorX all respective entries to a 3x1 vector
 * @param v The vector for which each entry is set.
 */
void lin_alg_set_v(float vector1, float vector2, float vector3, float v[MSZ]);

/**
 * @function lin_alg_s_m_mult()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 * @param m_out Scaled matrix
 */
void lin_alg_s_m_mult(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_m_scale()
 * Scales matrix
 * @param s Scalar to scale matrix with
 * @param m Matrix to be scaled
 */
void lin_alg_m_scale(float s, float m[MSZ][MSZ]);

/**
 * @function lin_alg_s_v_mult()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out
 */
void lin_alg_s_v_mult(float s, float v[MSZ], float v_out[MSZ]);

/**
 * @function lin_alg_v_scale()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 */
void lin_alg_v_scale(float s, float v[MSZ]);

/**
 * @function lin_alg_s_m_mult()
 * Add a scalar value to a matrix
 * @param s Scalar to add to a matrix
 * @param m Matrix to have a scalar added to each element
 * @param m_out Matrix with added scalar
 */
void lin_alg_s_m_add(float s, float m[MSZ][MSZ], float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_s_v_add()
 * Scales vector
 * @param s Scalar to scale vector with
 * @param v Vector to be scaled
 * @param v_out Vector with added scalar
 */
void lin_alg_s_v_add(float s, float v[MSZ], float v_out[MSZ]);

/**
 * @function lin_alg_m_m_add()
 * Add a matrix to a matrix
 * @param m1 Matrix to add to another matrix
 * @param m2 Matrix to have a matrix added to it
 * @param m_out Matrix as sum of two matrices
 */
void lin_alg_m_m_add(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_m_m_sub()
 * Add a matrix to a matrix
 * @param m1 Matrix to subtract to another matrix
 * @param m2 Matrix to have a matrix subtracted to it
 * @param m_out Matrix as difference of two matrices
 */
void lin_alg_m_m_sub(float m1[MSZ][MSZ], float m2[MSZ][MSZ],
        float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_v_v_add()
 * Add a vector value to a vector
 * @param v1 Vector to add to another vector
 * @param v2 Vector to have a vector added to it
 * @param v_out Vector as sum of two vectors
 */
void lin_alg_v_v_add(float v1[MSZ], float v2[MSZ], float v_out[MSZ]);

/**
 * @function lin_alg_v_v_sub()
 * Add a vector value to a vector
 * @param v1 Vector to subtract to another vector
 * @param v2 Vector to have a vector subtracted to it
 * @param v_out Vector as difference of two vectors
 */
void lin_alg_v_v_sub(float v1[MSZ], float v2[MSZ], float v_out[MSZ]);

/**
 * @function lin_alg_m_trace()
 * Sums the diagonals of a matrix. This sum is known as the trace.
 * @param m The matrix to have its trace calculated
 * @return The trace of the matrix.
 */
float lin_alg_m_trace(float m[MSZ][MSZ]);

/**
 * @function lin_alg_m_det()
 * @param m The matrix for which a determinant is to be calculated
 * @return The determinant of the matrix, if it exists. 
 */
float lin_alg_m_det(float m[MSZ][MSZ]);

/**
 * @function lin_alg_m_transpose()
 * Generates the transpose of a matrix
 * @param m The matrix to be transposed
 * @param m_out The transpose of m
 */
void lin_alg_m_transpose(float m[MSZ][MSZ], float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_skew_sym()
 * Generate the skew symmetric matrix of a (3x1) vector 
 * @param v A vector
 * @param m_out The resulting (3x3) skew matrix of v
 */
void lin_alg_skew_sym(float v[MSZ], float m_out[MSZ][MSZ]);

/**
 * @function lin_alg_anti_sym_pro()
 * Forms the antisymmetric projection matrix
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @param dcm The transpose of the dcm
 * @param p_out The antisymmetric projection matrix
 */
void lin_alg_anti_sym_pro(float dcm[MSZ][MSZ], float dcm_t[MSZ][MSZ],
        float p_out[MSZ][MSZ]);

/**
 * @function lin_alg_vex()
 * Change a skew matrix to a vector
 * @param m A skew matrix
 * @param v_out The corresponding vector
 */
void lin_alg_vex(float m[MSZ][MSZ], float v_out[MSZ]);

/**
 * @function lin_alg_v_norm()
 * Calculate the 2-norm, also known as the Euclidean length, or vector magnitude
 * @param v The vector for which the norm is to be calculated
 * @return The norm of v
 */
float lin_alg_v_norm(float v[MSZ]);

/**
 * @function lin_alg_dot()
 * @param u A vector
 * @param v A vector
 * @return The dot product (sometimes called the inner product) of u and v
 */
float lin_alg_dot(float u[MSZ], float v[MSZ]);

/**
 * @function lin_alg_cross()
 * @param u A vector
 * @param v A vector
 * @return The cross product (sometimes called the outter product) of u and v
 */
void lin_alg_cross(float u[MSZ], float v[MSZ], float w_out[MSZ]);

/**
 * @function lin_alg_angle_from_2vecs()
 * @param u A vector
 * @param v A vector
 * @return The angle between the two vectors
 */
float lin_alg_angle_from_2vecs(float u[MSZ], float v[MSZ]);

/**
 * @function lin_alg_gen_dcm();
 * Generates a Direction Cosine Matrix for a rotation about a given axis and a
 * given angle. See Jack B. Kuibers book: "Quaternions and Rotation Sequences"
 * @param angle The angle of rotation about the axis of rotation unit-vector
 * @param v The unit-vector representing the axis of rotation
 * @param dcm The Direction Cosine Matrix, a rotation matrix
 * @return SUCCESS or FAIL
 */
char lin_alg_gen_dcm(float angle, float v[MSZ], float dcm[MSZ][MSZ]);

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
        float dcm[MSZ][MSZ]);

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
        float *phi);

/**
 * @function lin_alg_set_q()
 * Set a quaternion with Euler angles
 * @param psi Yaw angle in radians from -pi to pi
 * @param theta Pitch angle in radians from -pi to pi
 * @param phi Roll angle in radians from -pi to pi
 * @param q_out A quaternion
 */
void lin_alg_set_q(float psi, float theta, float phi, float q_out[QSZ]);

/**
 * @function lin_alg_q_inv()
 * Form the complex conjugate 
 * @param q_in A quaternion
 * @param q_out The complex conjugate, or inverse of q_in
 */
void lin_alg_q_inv(float q_in[QSZ], float q_out[QSZ]);

/**
 * @function lin_alg_scale_q()
 * Scale a quaternion
 * @param s A scalar
 * @param q A quaternion
 */
void lin_alg_scale_q(float s, float q[QSZ]);

/**
 * @function lin_alg_q_norm()
 * @param q A quaternion
 * @return The magnitude of the quaternion, q
 */
float lin_alg_q_norm(float q[QSZ]);

/**
 * @function lin_alg_q_mult()
 * Multiply two quaternions together
 * @param p A quaternion
 * @param q A quaternion
 * @param r The resulting quaternion
 */
void lin_alg_q_mult(float p[QSZ], float q[QSZ], float r[QSZ]);

/**
 * @function lin_alg_q2dcm()
 * Form a DCM from a quaternion
 * @param q A quaternion
 * @param dcm_out A direction cosine matrix (DCM)
 */
void lin_alg_q2dcm(float q[QSZ], float dcm_out[MSZ][MSZ]);

/**
 * @function lin_alg_q2euler()
 * Return Euler angles
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler(float q[QSZ], float *psi, float *theta, float *phi);

/**
 * @function lin_alg_q2euler_abs()
 * Return absolute Euler angles
 * @param v
 * @param psi
 * @param theta
 * @param phi
 * @param q A quaternion
 */
void lin_alg_q2euler_abs(float q[QSZ], float *psi, float *theta, float *phi);

/**
 * @function lin_alg_m_print()
 * Print a matrix
 * @param matrix A matrix to print out
 */
void lin_alg_m_print(float matrix[MSZ][MSZ]);

/**
 * @function lin_alg_v_print()
 * Print a vector
 * @param vector A vector to print out
 */
void lin_alg_v_print(float vector[MSZ]);

#endif	/* LIN_ALG_FLOAT_H */

