/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ahrs_m_update.c
 *
 * Code generation for function 'ahrs_m_update'
 *
 */

/* Include files */
#include "ahrs_m_update.h"
#include <math.h>

/* Function Definitions */
void ahrs_m_update(const double Rminus[9], const double Bminus[3], const double
                   gyros[3], double mags[3], double accels[3], double
                   magInertial[3], double accelInertial[3], double deltaT,
                   double Kp_a, double Ki_a, double Kp_m, double Ki_m, double
                   Rplus[9], double Bplus[3])
{
  static const signed char iv[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double dv[9];
  double wx[9];
  double gyroInputWithFeedback[3];
  double wmeas_m[3];
  double absxk;
  double b_y;
  double d;
  double d1;
  double d2;
  double scale;
  double t;
  double wnorm;
  double y;
  int i;
  int i1;
  int k;

  /*  function [Rplus, Bplus] = IntegrateClosedLoop(Rminus, Bminus, gyros, mags,... */
  /*  accels, magInertial, accelInertial, deltaT, Kp_a, Ki_a, Kp_m, Ki_m) */
  /*  */
  /*  Function to implement the full complementary estimate and integration of */
  /*  gyros for full attitude estimation using an accelerometer and */
  /*  magnetometer feedback. */
  /*  */
  /*  Inputs: Previous attitute DCM (Rminus) */
  /*          Previous bias estimate (Bminus) */
  /*          Body Fixed Rotation rates ([p;q;r]) in rad/s (gyros) */
  /*          Magnetometer Readings in body coordinates (mags) */
  /*          Accelerometer Readings in body coordinates (accels) */
  /*          Inertial reference magnetic field (magInertial) */
  /*          Inertial reference gravity field (accelInertial) */
  /*          Time between samples (deltaT) in seconds */
  /*          Accelerometer proportional gain (Kp_a) */
  /*          Accelerometer integral gain (Ki_a) */
  /*          Magnetometer proportional gain (Kp_m) */
  /*          Magnetometer integral gain (Ki_m) */
  /*  */
  /*  Outputs: New DCM (Rplus) */
  /*           New Gyro Bias (Bplus) */
  /*  */
  /*  Note: This code implements a full complementary filter on the DCM using */
  /*  the matrix exponential integration of the gyros.  */
  scale = 3.3121686421112381E-170;
  absxk = fabs(accels[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(accels[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(accels[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);

  /*  set mags and accels to unit vectors */
  scale = 3.3121686421112381E-170;
  accels[0] /= y;
  absxk = fabs(mags[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  accels[1] /= y;
  absxk = fabs(mags[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  accels[2] /= y;
  absxk = fabs(mags[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  b_y = scale * sqrt(b_y);
  scale = 3.3121686421112381E-170;
  mags[0] /= b_y;
  absxk = fabs(magInertial[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  mags[1] /= b_y;
  absxk = fabs(magInertial[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  mags[2] /= b_y;
  absxk = fabs(magInertial[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);

  /*  set inertial reference vectors to unit vectors */
  scale = 3.3121686421112381E-170;
  magInertial[0] /= y;
  absxk = fabs(accelInertial[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_y = t * t;
  }

  magInertial[1] /= y;
  absxk = fabs(accelInertial[1]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  magInertial[2] /= y;
  absxk = fabs(accelInertial[2]);
  if (absxk > scale) {
    t = scale / absxk;
    b_y = b_y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    b_y += t * t;
  }

  b_y = scale * sqrt(b_y);
  accelInertial[0] /= b_y;
  accelInertial[1] /= b_y;
  accelInertial[2] /= b_y;

  /*  function rx = rcross(r) */
  /*  forms the skew symmetric x-product matrix of a 3x1 vector */
  dv[0] = 0.0;
  dv[3] = -accels[2];
  dv[6] = accels[1];
  dv[1] = accels[2];
  dv[4] = 0.0;
  dv[7] = -accels[0];
  dv[2] = -accels[1];
  dv[5] = accels[0];
  dv[8] = 0.0;
  d = accelInertial[0];
  d1 = accelInertial[1];
  d2 = accelInertial[2];
  for (k = 0; k < 3; k++) {
    wmeas_m[k] = (Rminus[k] * d + Rminus[k + 3] * d1) + Rminus[k + 6] * d2;
  }

  d = wmeas_m[0];
  d1 = wmeas_m[1];
  d2 = wmeas_m[2];
  for (k = 0; k < 3; k++) {
    Bplus[k] = (dv[k] * d + dv[k + 3] * d1) + dv[k + 6] * d2;
  }

  /*  accelerometer correction in the body frame */
  /*  function rx = rcross(r) */
  /*  forms the skew symmetric x-product matrix of a 3x1 vector */
  dv[0] = 0.0;
  dv[3] = -mags[2];
  dv[6] = mags[1];
  dv[1] = mags[2];
  dv[4] = 0.0;
  dv[7] = -mags[0];
  dv[2] = -mags[1];
  dv[5] = mags[0];
  dv[8] = 0.0;
  d = magInertial[0];
  d1 = magInertial[1];
  d2 = magInertial[2];
  for (k = 0; k < 3; k++) {
    wmeas_m[k] = (Rminus[k] * d + Rminus[k + 3] * d1) + Rminus[k + 6] * d2;
  }

  /*  magnetometer correction in the body frame */
  /*  function R_exp = Rexp(w, deltaT) */
  /*  */
  /*  returns the exponential Rodrigues parameter form of the integration that */
  /*  keeps R on SO(3). See Park and Chung paper. Requires a time step and the Rotation */
  /*  rate (omega). */
  /*  */
  wnorm = 0.0;
  scale = 3.3121686421112381E-170;
  d = wmeas_m[0];
  d1 = wmeas_m[1];
  d2 = wmeas_m[2];
  for (k = 0; k < 3; k++) {
    y = (dv[k] * d + dv[k + 3] * d1) + dv[k + 6] * d2;
    wmeas_m[k] = y;
    y = ((gyros[k] - Bminus[k]) + Kp_a * Bplus[k]) + Kp_m * y;
    gyroInputWithFeedback[k] = y;
    absxk = fabs(y);
    if (absxk > scale) {
      t = scale / absxk;
      wnorm = wnorm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      wnorm += t * t;
    }
  }

  wnorm = scale * sqrt(wnorm);

  /*  function rx = rcross(r) */
  /*  forms the skew symmetric x-product matrix of a 3x1 vector */
  wx[0] = 0.0;
  wx[3] = -gyroInputWithFeedback[2];
  wx[6] = gyroInputWithFeedback[1];
  wx[1] = gyroInputWithFeedback[2];
  wx[4] = 0.0;
  wx[7] = -gyroInputWithFeedback[0];
  wx[2] = -gyroInputWithFeedback[1];
  wx[5] = gyroInputWithFeedback[0];
  wx[8] = 0.0;
  d = wnorm * deltaT;
  if (d < 0.0001) {
    /*  just plucked this one out of the air, need a better number here */
    /* sincW = deltaT - (deltaT^3 * wnorm^2)/6.0 + (deltaT^5 * wnorm^4)/120.0; */
    b_y = 1.0;

    /* oneMinusCosW = (deltaT^2)/2.0 - (deltaT^4 * wnorm^2)/24.0 + (deltaT^6 * wnorm^4)/720.0; */
    y = 0.5;
  } else {
    b_y = sin(d) / wnorm;
    y = (1.0 - cos(d)) / (wnorm * wnorm);
  }

  /*  Rplus = Rexp(gyroInputWithFeedback, deltaT) * Rminus; */
  for (k = 0; k < 3; k++) {
    for (i = 0; i < 3; i++) {
      i1 = k + 3 * i;
      dv[i1] = ((double)iv[i1] - b_y * wx[i1]) + ((y * wx[k] * wx[3 * i] + y *
        wx[k + 3] * wx[3 * i + 1]) + y * wx[k + 6] * wx[3 * i + 2]);
    }

    d = dv[k];
    d1 = dv[k + 3];
    d2 = dv[k + 6];
    for (i = 0; i < 3; i++) {
      Rplus[k + 3 * i] = (d * Rminus[3 * i] + d1 * Rminus[3 * i + 1]) + d2 *
        Rminus[3 * i + 2];
    }

    Bplus[k] = Bminus[k] + (-Ki_a * Bplus[k] - Ki_m * wmeas_m[k]) * deltaT;
  }
}

/* End of code generation (ahrs_m_update.c) */
