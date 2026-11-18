/**
 * @file    Robot_Foward_AIgen.h
 * @brief   Drive the balancing robot a set distance, with a smooth parabolic
 *          acceleration curve. See Robot_Foward_AIgen.c.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 18 Nov 2026, for
 * Gene Connell's MCHA3500 balancing robot, from the request copied at the
 * bottom of Robot_Foward_AIgen.c. Not copied from any outside source.
 *
 *   Robot_Foward(distance (m), average velocity (m/s) or 'N',
 *                peak acceleration (m/s^2), acceleration gradient (m/s^3));
 *
 * Call it after Balance_Start(seconds) in main.c, so the robot is already
 * balancing: the move starts at once. (Called before the robot is balancing,
 * it waits until it has balanced for RF_START_DELAY_S.) Or type it on the
 * serial port while it balances:   fwd 0.5 0.1 0.2 0.4    fwd 0.5 N 0.1 0.2
 */
#ifndef ROBOT_FOWARD_AIGEN_H
#define ROBOT_FOWARD_AIGEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================= */
/* Settings                                                                  */
/* ========================================================================= */
/* Put this in the velocity argument to drive from the acceleration curve
   alone. Robot_Foward(1.0f, 'N', 0.1f, 0.2f) works as it is: C turns the
   character 'N' into the number 78, which is never a real speed. */
#define RF_NO_VELOCITY          ((float)'N')

/* A move asked for before the robot is balancing waits this long after the
   controller engages, so you can let go and step back first. One asked for
   while it balances (after Balance_Start(seconds)) starts at once. */
#define RF_START_DELAY_S        3.0f

/* A move is refused (with a message) if it needs more than these. Raise
   them once the robot follows the gentle moves well.
   For scale: the controller's own limits are omegaMax x r = 1.5 m/s and
   alphaMax x r = 1.6 m/s^2, and it needs most of that for balancing. */
#define RF_MAX_DISTANCE_M       10.0f
#define RF_MAX_SPEED_MPS        0.25f     /* top (cruise) speed                */
#define RF_MAX_ACCEL_MPS2       0.30f     /* peak of the acceleration curve    */
#define RF_MIN_PULSE_S          0.50f     /* shortest speed-up / slow-down     */

/* The move is stopped early if the encoders get this far behind (or ahead
   of) where the move says the robot should be: it is blocked, or the move
   is too fast for it. The robot then slows to a halt and holds position. */
#define RF_FOLLOW_ERR_M         0.30f

/* Arrived = within RF_DONE_TOL_M of the distance and slower than
   RF_DONE_SPEED_MPS for RF_DONE_HOLD_S. Gives up waiting after
   RF_SETTLE_TIMEOUT_S (the robot still holds the target position). */
#define RF_DONE_TOL_M           0.020f
#define RF_DONE_SPEED_MPS       0.020f
#define RF_DONE_HOLD_S          0.30f
#define RF_SETTLE_TIMEOUT_S     4.0f

/* ========================================================================= */
/* State, for Live Expressions (add "rf")                                    */
/* ========================================================================= */
typedef enum {
    RF_IDLE = 0,       /* no move: holding position                          */
    RF_QUEUED,         /* waiting for the robot to be balancing              */
    RF_ACCEL,          /* speeding up   (parabolic acceleration pulse)       */
    RF_CRUISE,         /* constant speed                                     */
    RF_DECEL,          /* slowing down  (the same pulse, upside down)        */
    RF_SETTLE,         /* at the distance, waiting for the robot to settle   */
    RF_BRAKE,          /* stopping early: slowing smoothly to a halt         */
    RF_DONE,           /* arrived: normal balancing at the new position      */
    RF_STOPPED         /* stopped early and holding position (see note)      */
} RF_State;

typedef struct {
    RF_State    state;
    const char *note;            /* why the last move was refused or stopped  */

    /* The move as planned (after fitting the curve to the velocity) */
    float distance_m;            /* signed: + forwards, - backwards           */
    float peak_accel;            /* m/s^2, peak of the curve as used          */
    float gradient;              /* m/s^3, slope of the curve at a = 0        */
    float cruise_speed;          /* m/s, top speed                            */
    float t_accel;               /* s, one acceleration pulse                 */
    float t_cruise;              /* s                                         */
    float t_total;               /* s = 2 x t_accel + t_cruise                */
    float avg_velocity;          /* m/s = distance / t_total                  */

    /* Now: where the move says the robot should be (magnitudes along the
       move) and where the encoders say it is */
    float t;                     /* s since the move started                  */
    float dist_ref;              /* m                                         */
    float vel_ref;               /* m/s                                       */
    float acc_ref;               /* m/s^2                                     */
    float dist_meas;             /* m, from Encoder_Distance_m()              */
    float follow_err;            /* m = dist_ref - dist_meas                  */
    float follow_err_max;        /* largest |follow_err| this move            */

    /* Result of the last finished move */
    float dist_at_end;           /* m, encoder distance at the planned finish */
    float time_taken;            /* s, start until settled at the distance    */
    float dist_final;            /* m, encoder distance when settled          */

    /* For the balance controller (Balance_AIgen.c), wheel units, signed:
       wheel angle relative to where the controller engaged, and its rates */
    float phi_ref;               /* rad                                       */
    float omega_ref;             /* rad/s                                     */
    float alpha_ref;             /* rad/s^2                                   */
} RF_Status;

extern RF_Status rf;

/* ========================================================================= */
/* Functions                                                                 */
/* ========================================================================= */
/* Robot_Foward(distance (m), average velocity (m/s) or 'N',
 *              peak acceleration (m/s^2), acceleration gradient (m/s^3))
 * Returns 1 if the move was accepted (started or queued), 0 if refused
 * (rf.note and the serial port say why). Full description above the
 * function in Robot_Foward_AIgen.c. */
uint8_t Robot_Foward(float distance_m, float avg_velocity_mps,
                     float peak_accel_mps2, float accel_gradient_mps3);

void    Robot_Foward_Stop(void);     /* stop the move: slow to a halt and hold */
uint8_t Robot_Foward_Busy(void);     /* 1 while a move is queued or running    */
void    Robot_Foward_Cmd(int argc, char *argv[]);   /* "fwd" (Lab5.c's table)  */

/* Called by Balance_AIgen.c, not by main.c */
void    Robot_Foward_Engage(void);                  /* controller engaged      */
void    Robot_Foward_Update(uint32_t run_samples);  /* every 2 ms while balancing */
void    Robot_Foward_BalanceStopped(void);          /* controller stopped      */

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_FOWARD_AIGEN_H */
