/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ahrs_m_update.h
 *
 * Code generation for function 'ahrs_m_update'
 *
 */

#ifndef AHRS_M_UPDATE_H
#define AHRS_M_UPDATE_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>
#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  extern void ahrs_m_update(const double Rminus[9], const double Bminus[3],
    const double gyros[3], double mags[3], double accels[3], double magInertial
    [3], double accelInertial[3], double deltaT, double Kp_a, double Ki_a,
    double Kp_m, double Ki_m, double Rplus[9], double Bplus[3]);

#ifdef __cplusplus

}
#endif
#endif

/* End of code generation (ahrs_m_update.h) */
