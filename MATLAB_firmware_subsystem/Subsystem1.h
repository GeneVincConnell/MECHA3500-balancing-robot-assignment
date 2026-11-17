/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Subsystem1.h
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

#ifndef Subsystem1_h_
#define Subsystem1_h_
#ifndef Subsystem1_COMMON_INCLUDES_
#define Subsystem1_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "math.h"
#endif                                 /* Subsystem1_COMMON_INCLUDES_ */

#include "Subsystem1_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  double Kalmanfilter_DSTATE[3];       /* '<S2>/Kalman filter' */
  double Wheelspeed_DSTATE;            /* '<S2>/Wheel speed' */
  double Integralofphierror_DSTATE;    /* '<S2>/Integral of phi error' */
  double PrevY;                        /* '<S3>/Acceleration limit' */
} DW_Subsystem1_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: -K
   * Referenced by: '<S2>/LQR gain'
   */
  double LQRgain_Gain[5];
} ConstP_Subsystem1_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  double phi_ref;                      /* '<Root>/phi_ref' */
  double y[3];                         /* '<Root>/y' */
} ExtU_Subsystem1_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  double xhat[3];                      /* '<Root>/xhat' */
  double omega;                        /* '<Root>/omega' */
  double z;                            /* '<Root>/z' */
} ExtY_Subsystem1_T;

/* Real-time Model Data Structure */
struct tag_RTM_Subsystem1_T {
  const char * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW_Subsystem1_T Subsystem1_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Subsystem1_T Subsystem1_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Subsystem1_T Subsystem1_Y;

/* Constant parameters (default storage) */
extern const ConstP_Subsystem1_T Subsystem1_ConstP;

/* Model entry point functions */
extern void Subsystem1_initialize(void);
extern void Subsystem1_step(void);
extern void Subsystem1_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Subsystem1_T *const Subsystem1_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('robotLQG_sim/Subsystem1')    - opens subsystem robotLQG_sim/Subsystem1
 * hilite_system('robotLQG_sim/Subsystem1/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'robotLQG_sim'
 * '<S1>'   : 'robotLQG_sim/Subsystem1'
 * '<S2>'   : 'robotLQG_sim/Subsystem1/Control'
 * '<S3>'   : 'robotLQG_sim/Subsystem1/Stepper motors'
 */
#endif                                 /* Subsystem1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
