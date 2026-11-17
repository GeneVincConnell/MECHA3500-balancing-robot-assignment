/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Subsystem1.c
 *
 * Code generated for Simulink model 'Subsystem1'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Tue Nov 17 09:37:10 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Subsystem1.h"
#include <stdint.h>

/* Block signals and states (default storage) */
DW_Subsystem1_T Subsystem1_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Subsystem1_T Subsystem1_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Subsystem1_T Subsystem1_Y;

/* Real-time model */
static RT_MODEL_Subsystem1_T Subsystem1_M_;
RT_MODEL_Subsystem1_T *const Subsystem1_M = &Subsystem1_M_;

/* Model step function */
void Subsystem1_step(void)
{
  double tmp[5];
  double rateLimiterRate;
  int32_t i;

  /* Outputs for Atomic SubSystem: '<Root>/Subsystem1' */
  /* DiscreteStateSpace: '<S2>/Kalman filter' incorporates:
   *  Outport: '<Root>/xhat'
   */
  {
    Subsystem1_Y.xhat[0] = 1.0*Subsystem1_DW.Kalmanfilter_DSTATE[0];
    Subsystem1_Y.xhat[1] = 1.0*Subsystem1_DW.Kalmanfilter_DSTATE[1];
    Subsystem1_Y.xhat[2] = 1.0*Subsystem1_DW.Kalmanfilter_DSTATE[2];
  }

  /* Outport: '<Root>/z' incorporates:
   *  DiscreteIntegrator: '<S2>/Integral of phi error'
   */
  Subsystem1_Y.z = Subsystem1_DW.Integralofphierror_DSTATE;

  /* RateLimiter: '<S3>/Acceleration limit' incorporates:
   *  DiscreteIntegrator: '<S2>/Wheel speed'
   */
  rateLimiterRate = Subsystem1_DW.Wheelspeed_DSTATE - Subsystem1_DW.PrevY;
  if (rateLimiterRate > 0.05730704967773811) {
    /* Gain: '<S2>/select phi' */
    Subsystem1_DW.PrevY += 0.05730704967773811;
  } else if (rateLimiterRate < -0.05730704967773811) {
    /* Gain: '<S2>/select phi' */
    Subsystem1_DW.PrevY -= 0.05730704967773811;
  } else {
    /* Gain: '<S2>/select phi' */
    Subsystem1_DW.PrevY = Subsystem1_DW.Wheelspeed_DSTATE;
  }

  /* End of RateLimiter: '<S3>/Acceleration limit' */

  /* Saturate: '<S3>/Speed limit' */
  if (Subsystem1_DW.PrevY > 27.11817869840188) {
    /* Saturate: '<S3>/Speed limit' */
    Subsystem1_Y.omega = 27.11817869840188;
  } else if (Subsystem1_DW.PrevY < -27.11817869840188) {
    /* Saturate: '<S3>/Speed limit' */
    Subsystem1_Y.omega = -27.11817869840188;
  } else {
    /* Saturate: '<S3>/Speed limit' */
    Subsystem1_Y.omega = Subsystem1_DW.PrevY;
  }

  /* End of Saturate: '<S3>/Speed limit' */

  /* Update for DiscreteStateSpace: '<S2>/Kalman filter' incorporates:
   *  Inport: '<Root>/y'
   */
  {
    double xnew[3];
    xnew[0] = 0.42580814429331149*Subsystem1_DW.Kalmanfilter_DSTATE[0] +
      0.0034254187505727627*Subsystem1_DW.Kalmanfilter_DSTATE[1]
      + 0.0035069803417830852*Subsystem1_DW.Kalmanfilter_DSTATE[2];
    xnew[0] += 0.0019500256187815652*Subsystem1_Y.omega + 0.57419185570668851*
      Subsystem1_U.y[0]
      + (-0.0034254187505727627)*Subsystem1_U.y[1]
      + (-0.00014086721350048276)*Subsystem1_U.y[2];
    xnew[1] = 0.20301584795335928*Subsystem1_DW.Kalmanfilter_DSTATE[0] +
      0.99687064049688467*Subsystem1_DW.Kalmanfilter_DSTATE[1]
      + (-0.015347599087999625)*Subsystem1_DW.Kalmanfilter_DSTATE[2];
    xnew[1] += 0.00021870289903633473*Subsystem1_Y.omega + (-0.20301584795335928)*
      Subsystem1_U.y[0]
      + 0.0031853946815428558*Subsystem1_U.y[1]
      + 0.0026165145839891291*Subsystem1_U.y[2];
    xnew[2] = 0.00079221550305442448*Subsystem1_DW.Kalmanfilter_DSTATE[0] +
      0.0021584737526385476*Subsystem1_DW.Kalmanfilter_DSTATE[1]
      + 0.75209774700908516*Subsystem1_DW.Kalmanfilter_DSTATE[2];
    xnew[2] += 0.0035326008384688966*Subsystem1_Y.omega +
      (-0.00079221550305442448)*Subsystem1_U.y[0]
      + 9.2349574581397821E-5*Subsystem1_U.y[1]
      + 0.00995990559245782*Subsystem1_U.y[2];
    (void) memcpy(Subsystem1_DW.Kalmanfilter_DSTATE, xnew,
                  sizeof(double)*3);
  }

  /* SignalConversion generated from: '<S2>/LQR gain' incorporates:
   *  Inport: '<Root>/phi_ref'
   *  Outport: '<Root>/xhat'
   *  Sum: '<S2>/state error'
   */
  tmp[0] = Subsystem1_Y.xhat[0] - Subsystem1_U.phi_ref;

  /* Sum: '<S2>/state error' incorporates:
   *  Gain: '<S2>/x_ref'
   *  Inport: '<Root>/phi_ref'
   */
  rateLimiterRate = 0.0 * Subsystem1_U.phi_ref;

  /* SignalConversion generated from: '<S2>/LQR gain' incorporates:
   *  DiscreteIntegrator: '<S2>/Integral of phi error'
   *  DiscreteIntegrator: '<S2>/Wheel speed'
   *  Gain: '<S2>/x_ref'
   *  Outport: '<Root>/xhat'
   *  Sum: '<S2>/state error'
   */
  tmp[1] = Subsystem1_Y.xhat[1] - rateLimiterRate;
  tmp[2] = Subsystem1_Y.xhat[2] - rateLimiterRate;
  tmp[3] = Subsystem1_DW.Wheelspeed_DSTATE;
  tmp[4] = Subsystem1_DW.Integralofphierror_DSTATE;

  /* Gain: '<S2>/LQR gain' */
  rateLimiterRate = 0.0;
  for (i = 0; i < 5; i++) {
    rateLimiterRate += Subsystem1_ConstP.LQRgain_Gain[i] * tmp[i];
  }

  /* End of Gain: '<S2>/LQR gain' */

  /* Saturate: '<S2>/Acceleration limit' */
  if (rateLimiterRate > 28.653524838869053) {
    rateLimiterRate = 28.653524838869053;
  } else if (rateLimiterRate < -28.653524838869053) {
    rateLimiterRate = -28.653524838869053;
  }

  /* Update for DiscreteIntegrator: '<S2>/Wheel speed' incorporates:
   *  Saturate: '<S2>/Acceleration limit'
   */
  Subsystem1_DW.Wheelspeed_DSTATE += 0.002 * rateLimiterRate;
  if (Subsystem1_DW.Wheelspeed_DSTATE > 27.11817869840188) {
    Subsystem1_DW.Wheelspeed_DSTATE = 27.11817869840188;
  } else if (Subsystem1_DW.Wheelspeed_DSTATE < -27.11817869840188) {
    Subsystem1_DW.Wheelspeed_DSTATE = -27.11817869840188;
  }

  /* End of Update for DiscreteIntegrator: '<S2>/Wheel speed' */

  /* Update for DiscreteIntegrator: '<S2>/Integral of phi error' incorporates:
   *  Gain: '<S2>/select phi'
   *  Inport: '<Root>/phi_ref'
   *  Inport: '<Root>/y'
   *  Sum: '<S2>/phi error'
   */
  Subsystem1_DW.Integralofphierror_DSTATE += (((0.0 * Subsystem1_U.y[1] +
    Subsystem1_U.y[0]) + 0.0 * Subsystem1_U.y[2]) - Subsystem1_U.phi_ref) *
    0.002;
  if (Subsystem1_DW.Integralofphierror_DSTATE > 0.5109340404094733) {
    Subsystem1_DW.Integralofphierror_DSTATE = 0.5109340404094733;
  } else if (Subsystem1_DW.Integralofphierror_DSTATE < -0.5109340404094733) {
    Subsystem1_DW.Integralofphierror_DSTATE = -0.5109340404094733;
  }

  /* End of Update for DiscreteIntegrator: '<S2>/Integral of phi error' */
  /* End of Outputs for SubSystem: '<Root>/Subsystem1' */
}

/* Model initialize function */
void Subsystem1_initialize(void)
{
  /* SystemInitialize for Atomic SubSystem: '<Root>/Subsystem1' */
  /* InitializeConditions for DiscreteStateSpace: '<S2>/Kalman filter' */
  Subsystem1_DW.Kalmanfilter_DSTATE[0] = 0.0;
  Subsystem1_DW.Kalmanfilter_DSTATE[1] = 0.11344640137963141;
  Subsystem1_DW.Kalmanfilter_DSTATE[2] = 0.0;

  /* End of SystemInitialize for SubSystem: '<Root>/Subsystem1' */
}

/* Model terminate function */
void Subsystem1_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
