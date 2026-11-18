/**
 * @file    Robot_Foward_AIgen.c
 * @brief   Drive the balancing robot a set distance and stop, balanced, at
 *          the end. The acceleration follows a smooth parabolic curve.
 *
 * PROVENANCE: AI-generated. Written by Claude (Anthropic), 18 Nov 2026, for
 * Gene Connell's MCHA3500 balancing robot, from the request copied at the
 * bottom of this file. Not copied from any outside source. It calls code
 * that is already in the project:
 *   Encoder.c         Encoder_Distance_m(), Encoder_Speed_mps() (added for this)
 *   Balance_AIgen.c   calls Robot_Foward_Update() every 2 ms while balancing
 *                     and feeds rf.phi_ref (and the speed and acceleration
 *                     references) to the generated controller
 *   Lab5.c            Lab5_printf() and the "fwd" serial command
 *
 * How the move is built
 *   The controller holds the wheel angle phi at phi_ref. This file moves
 *   phi_ref along a planned path, distance against time, and the controller
 *   makes the robot follow it. The path has three parts:
 *
 *     acceleration
 *       ^      _                                    1. speed up:  parabola
 *     A |    /   \                                  2. cruise:    a = 0
 *       |   /     \                                 3. slow down: the same
 *     0 +--+-------+----------------+-------+--> t     parabola upside down
 *       |  0       Ta               :\     /:
 *    -A |                           :  \_/  :
 *          |<- Ta ->|<---- Tc ---->|<- Ta ->|      total time T = 2 Ta + Tc
 *
 *   One pulse (t measured from the start of the pulse, 0 <= t <= Ta):
 *       a(t) = G t (1 - t/Ta)          acceleration: 0 at t = 0 and t = Ta
 *            = A - k (t - Ta/2)^2      the same parabola in vertex form
 *       A  = peak acceleration (the "y offset" of the parabola)   [m/s^2]
 *       G  = acceleration gradient: the slope da/dt where the curve leaves
 *            zero, which is also the largest jerk                 [m/s^3]
 *       Ta = 4 A / G                   pulse length
 *       k  = G^2 / (4 A)               t^2 coefficient
 *       speed gained      vc = (2/3) A Ta = G Ta^2 / 6
 *       distance covered  vc Ta / 2
 *   The slow-down is positioned so the speed reaches zero exactly at the
 *   distance: it starts when the remaining distance equals what the pulse
 *   needs (vc Ta / 2). After it, phi_ref stays at the target and the robot
 *   is back in normal balancing, at the new spot.
 *
 * Fitting the curve to the velocity (the velocity wins over the curve)
 *   Average velocity V = distance / total time, so T = D / V is fixed. With
 *   a cruise in the middle, D = vc (T - Ta), so for a pulse length Ta the
 *   peak has to be  A' = 1.5 D / (Ta (T - Ta)),  and G' = 4 A' / Ta.
 *   The curve is changed as little as it can be, and the peak (which sets
 *   how far the robot leans) is never raised unless it has to be:
 *   1. No time for two pulses as long as yours (T < 2 Ta): each pulse is
 *      shortened to T/2, with no cruise (vc = 2 V, A' = 6 D / T^2).
 *   2. The velocity needs less than your peak: same pulse length, lower
 *      curve (peak and gradient scaled down together).
 *   3. It needs more than your peak: the peak is kept and the pulses are
 *      made longer, which lowers the gradient. Ta is the smaller root of
 *          Ta^2 - T Ta + 1.5 D / A = 0
 *   4. Not even pulses filling the whole time reach it at your peak
 *      (T^2 < 6 D / A): no cruise, Ta = T/2, and the peak is raised to
 *      A' = 6 D / T^2.
 *   The new curve still has to pass the RF_MAX_ limits, or it is refused.
 *   With 'N' for the velocity the curve is used as given: vc = (2/3) A Ta,
 *   and the cruise lasts as long as the distance needs. Only if the two
 *   pulses alone would overshoot the distance is the curve scaled down
 *   (same Ta) until they cover exactly the distance.
 *   The values really used are printed and are in rf (Live Expressions).
 *
 * Where the encoders come in
 *   Encoder_Distance_m() is the distance travelled. Every 2 ms it is
 *   compared with the planned distance:
 *   - the controller drives the difference to zero (through phi_ref), so
 *     the robot ends where the encoders say the distance is reached;
 *   - if the difference passes RF_FOLLOW_ERR_M the move is stopped early:
 *     the robot slows to a halt from where it is and holds position there
 *     (it is blocked, or the move is too fast for it);
 *   - the encoder distance at the planned finish time is reported, which
 *     shows how well the robot kept to the velocity;
 *   - the move is finished when the encoder distance is within
 *     RF_DONE_TOL_M of the target and the wheels are nearly still.
 *   Distance = wheel radius x wheel angle, and the wheel angle is measured
 *   against the chassis, so leaning 1 deg reads as 1 mm. It cancels over a
 *   move, because the robot is upright at both ends.
 */
#include "Robot.h"                /* Encoder.h, Lab5_printf(), IMU_SAMPLE_US */
#include "Robot_Foward_AIgen.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RF_TS_S          ((float)IMU_SAMPLE_US * 1.0e-6f)     /* 2 ms */
#define RF_SAMPLES(s)    ((uint32_t)((s) / RF_TS_S + 0.5f))
#define RF_MM(x)         ((long)lroundf((x) * 1000.0f))       /* m -> mm, for printing */

RF_Status rf = { .state = RF_IDLE, .note = "no move yet" };

static uint8_t  balancing;        /* the balance controller is running        */
static uint32_t run_samples_now;  /* samples since it engaged                 */
static uint32_t start_after;      /* a queued move starts once run_samples_now
                                     reaches this (0 = at once)               */
static float    enc_at_engage;    /* Encoder_Distance_m() when it engaged, m  */
static float    x_hold;           /* position to hold when no move runs, m
                                     from the engage point                    */
static float    x_start;          /* x_hold when this move started            */
static float    enc_at_start;     /* Encoder_Distance_m() when it started, m  */
static float    dir;              /* +1 forwards, -1 backwards                */
static uint32_t move_samples;     /* samples since this move started          */
static uint32_t still_samples;    /* samples inside the "arrived" window      */

/* Early stop (RF_BRAKE): the reference slows to a halt over brake_T */
static float    brake_x;          /* reference position, m from engage        */
static float    brake_v;          /* reference speed, m/s, signed             */
static float    brake_a0;         /* acceleration the stop began with, signed */
static float    brake_dv;         /* speed the slow-down pulse removes, signed */
static float    brake_T;          /* how long the stop takes, s               */
static uint32_t brake_samples;

/* Lab5_printf() waits when its buffer is full. Inside the 2 ms loop, print
   only when the line is sure to fit. Whole numbers only (mm, ms): newlib's
   float printf is too slow to run while the robot is balancing. */
#define RF_PRINT(...)                                           \
    do {                                                        \
        if (Lab5_TxFree() >= LAB5_PRINTF_MAX) {                 \
            Lab5_printf(__VA_ARGS__);                           \
        }                                                       \
    } while (0)

/* ========================================================================= */
/* The curve                                                                 */
/* ========================================================================= */
/* One acceleration pulse at time t (0 <= t <= Ta) from its start:
   acceleration, speed gained so far and distance covered so far. */
static void rf_pulse(float t, float G, float Ta, float *acc, float *vel, float *dist)
{
    float u = t / Ta;                                      /* 0 .. 1 through the pulse */
    *acc  = G * t * (1.0f - u);
    *vel  = G * t * t * (0.5f - u / 3.0f);
    *dist = G * t * t * t * (1.0f / 6.0f - u / 12.0f);
}

/* Planned distance, speed and acceleration t seconds into the move
   (magnitudes; the direction is applied afterwards) */
static void rf_path(float t, float *dist, float *vel, float *acc)
{
    float G = rf.gradient, Ta = rf.t_accel, Tc = rf.t_cruise, vc = rf.cruise_speed;
    float pa, pv, pd;

    if (t < Ta) {                                  /* 1. speed up */
        rf_pulse(t, G, Ta, acc, vel, dist);
    } else if (t < Ta + Tc) {                      /* 2. cruise */
        *acc  = 0.0f;
        *vel  = vc;
        *dist = 0.5f * vc * Ta + vc * (t - Ta);
    } else if (t < rf.t_total) {                   /* 3. slow down */
        float td = t - Ta - Tc;
        rf_pulse(td, G, Ta, &pa, &pv, &pd);
        *acc  = -pa;
        *vel  = vc - pv;
        *dist = vc * (0.5f * Ta + Tc) + vc * td - pd;
    } else {                                       /* there */
        *acc  = 0.0f;
        *vel  = 0.0f;
        *dist = fabsf(rf.distance_m);
    }
}

/* Work out the move: fills the plan in rf. Returns 0 (and sets rf.note) if
   the numbers make no sense or the move would pass a limit. */
static uint8_t rf_plan(float D, uint8_t use_velocity, float V, float A, float G)
{
    const char *fit = "";

    if (!(D > 0.0f) || (D > RF_MAX_DISTANCE_M)) {
        rf.note = "distance must be more than 0 and no more than RF_MAX_DISTANCE_M";
        return 0U;
    }
    if (!(A > 0.0f) || !(G > 0.0f)) {
        rf.note = "peak acceleration and gradient must be more than 0";
        return 0U;
    }
    if (use_velocity && !(V > 0.0f)) {
        rf.note = "velocity must be more than 0, or 'N'";
        return 0U;
    }

    float Ta = 4.0f * A / G;          /* pulse length of the curve as given */
    float vc;                         /* cruise speed                       */
    float Tc;                         /* cruise time                        */

    if (use_velocity) {
        float T = D / V;              /* the velocity fixes the total time  */
        /* Peak needed with the pulse length as given: vc = D / (T - Ta),
           A = 1.5 vc / Ta (only possible if two pulses fit: T >= 2 Ta) */
        float A_same_Ta = (T >= 2.0f * Ta) ? 1.5f * D / (Ta * (T - Ta)) : 0.0f;
        if (T < 2.0f * Ta) {
            Ta = 0.5f * T;            /* no time for two pulses that long   */
            vc = 2.0f * V;
            Tc = 0.0f;
            fit = "pulses shortened to half the time each (no cruise) to fit the velocity";
        } else if (A_same_Ta <= A) {
            vc = D / (T - Ta);        /* same pulse length, lower curve     */
            Tc = T - 2.0f * Ta;
            fit = "curve lowered (same pulse length) to fit the velocity";
        } else {
            /* The curve as given is too weak for the velocity. Keep the peak
               and make the pulses longer instead (gentler gradient):
               A = 1.5 D / (Ta (T - Ta))  ->  Ta^2 - T Ta + 1.5 D / A = 0 */
            float disc = T * T - 6.0f * D / A;
            if (disc >= 0.0f) {
                Ta = 0.5f * (T - sqrtf(disc));   /* the root with Ta <= T/2  */
                vc = D / (T - Ta);
                Tc = T - 2.0f * Ta;
                fit = "peak kept, pulses lengthened (gentler gradient) to reach the velocity";
            } else {
                Ta = 0.5f * T;        /* even the longest pulses need more  */
                vc = 2.0f * V;
                Tc = 0.0f;
                fit = "peak raised: the velocity needs more than your peak, even with no cruise";
            }
        }
    } else {
        vc = (2.0f / 3.0f) * A * Ta;  /* the curve as given                 */
        if (vc * Ta > D) {
            vc = D / Ta;              /* too short for it: scale it down    */
            Tc = 0.0f;
            fit = "curve lowered (same pulse length): the distance is too short for it";
        } else {
            Tc = D / vc - Ta;
        }
    }

    float A_used = 1.5f * vc / Ta;    /* peak that gives vc in Ta           */
    float G_used = 4.0f * A_used / Ta;

    /* Report what it needs before refusing, so the numbers can be changed */
    if ((vc > RF_MAX_SPEED_MPS) || (A_used > RF_MAX_ACCEL_MPS2) || (Ta < RF_MIN_PULSE_S)) {
        Lab5_printf("fwd: refused. It needs top speed %ld mm/s (limit %ld), peak accel "
                    "%ld mm/s2 (limit %ld), pulse %ld ms (at least %ld).\n",
                    RF_MM(vc), RF_MM(RF_MAX_SPEED_MPS), RF_MM(A_used),
                    RF_MM(RF_MAX_ACCEL_MPS2), RF_MM(Ta), RF_MM(RF_MIN_PULSE_S));
        rf.note = "needs more speed or acceleration than the RF_MAX_ limits "
                  "(Robot_Foward_AIgen.h), or a shorter pulse than RF_MIN_PULSE_S";
        return 0U;
    }

    rf.peak_accel   = A_used;
    rf.gradient     = G_used;
    rf.cruise_speed = vc;
    rf.t_accel      = Ta;
    rf.t_cruise     = Tc;
    rf.t_total      = 2.0f * Ta + Tc;
    rf.avg_velocity = D / rf.t_total;

    Lab5_printf("fwd: %ld mm in %ld ms (average %ld mm/s). Curve used: peak %ld mm/s2, "
                "gradient %ld mm/s3 (you gave %ld, %ld)\n",
                RF_MM(D), RF_MM(rf.t_total), RF_MM(rf.avg_velocity),
                RF_MM(A_used), RF_MM(G_used), RF_MM(A), RF_MM(G));
    if (fit[0] != '\0') {
        Lab5_printf("     %s\n", fit);
    }
    Lab5_printf("     speed up %ld ms, cruise at %ld mm/s for %ld ms, slow down %ld ms\n",
                RF_MM(Ta), RF_MM(vc), RF_MM(Tc), RF_MM(Ta));
    return 1U;
}

/* ========================================================================= */
/* Running the move                                                          */
/* ========================================================================= */
static uint8_t rf_moving(void)
{
    return ((rf.state == RF_ACCEL) || (rf.state == RF_CRUISE) ||
            (rf.state == RF_DECEL) || (rf.state == RF_SETTLE)) ? 1U : 0U;
}

/* Hand the controller a fixed position to hold (no move running) */
static void rf_hold(void)
{
    rf.phi_ref   = x_hold / ENC_WHEEL_RADIUS_M;
    rf.omega_ref = 0.0f;
    rf.alpha_ref = 0.0f;
    rf.vel_ref   = 0.0f;
    rf.acc_ref   = 0.0f;
}

/* Stop the move early, from position x0, speed v0 and acceleration a0
   (signed, m from the engage point). Freezing phi_ref while the robot is
   rolling would jolt it: it has to lean back before it can slow down. So
   the reference slows to a halt over one pulse length Ta instead:
       a(t) = a0 (1 - t/Ta)^2  -  pulse(t)
   The first term fades out whatever acceleration the move had, so there is
   no step in acceleration. The second is the usual parabola, sized to take
   away all the speed (v0 plus what the first term still adds, a0 Ta / 3).
   Stopped from the cruise, this is exactly the move's own slow-down. */
static void rf_brake(float x0, float v0, float a0, const char *note)
{
    brake_x       = x0;
    brake_v       = v0;
    brake_a0      = a0;
    brake_T       = rf.t_accel;
    brake_dv      = v0 + a0 * brake_T / 3.0f;
    brake_samples = 0U;
    rf.state = RF_BRAKE;
    rf.note  = note;
    RF_PRINT("fwd: stopping after %ld mm of %ld mm: %s\n",
             RF_MM(rf.dist_meas), RF_MM(fabsf(rf.distance_m)), note);
}

/* One sample of the early stop: step the position and speed on by one
   sample, then hand them to the controller */
static void rf_brake_step(void)
{
    brake_samples++;
    float t = (float)brake_samples * RF_TS_S;
    if (t >= brake_T) {
        x_hold   = brake_x;                         /* hold where it ended */
        rf.state = RF_STOPPED;
        rf_hold();
        return;
    }

    float fade = (1.0f - t / brake_T) * (1.0f - t / brake_T);
    float G    = 6.0f * brake_dv / (brake_T * brake_T);   /* pulse that removes brake_dv */
    float pa, pv, pd;
    rf_pulse(t, G, brake_T, &pa, &pv, &pd);
    float acc = brake_a0 * fade - pa;

    brake_x += brake_v * RF_TS_S;
    brake_v += acc * RF_TS_S;

    rf.phi_ref   = brake_x / ENC_WHEEL_RADIUS_M;
    rf.omega_ref = brake_v / ENC_WHEEL_RADIUS_M;
    rf.alpha_ref = acc / ENC_WHEEL_RADIUS_M;
    rf.vel_ref   = fabsf(brake_v);
    rf.acc_ref   = acc * dir;
}

/* Balance_AIgen.c: the controller has just engaged. This spot is zero. */
void Robot_Foward_Engage(void)
{
    balancing       = 1U;
    run_samples_now = 0U;
    enc_at_engage   = Encoder_Distance_m();
    x_hold          = 0.0f;
    if (rf.state != RF_QUEUED) {
        rf.state = RF_IDLE;
    }
    rf_hold();
}

/* Balance_AIgen.c: the controller has stopped (fell, B1, "bal off").
   A move that was under way is dropped; one still queued stays queued. */
void Robot_Foward_BalanceStopped(void)
{
    balancing   = 0U;
    start_after = RF_SAMPLES(RF_START_DELAY_S);   /* a queued move waits again */
    if (rf_moving() || (rf.state == RF_BRAKE)) {
        rf.state = RF_STOPPED;
        rf.note  = "the balance controller stopped";
    }
    rf_hold();
}

/* Balance_AIgen.c: every 2 ms while balancing, before the controller step.
   Leaves this sample's references in rf.phi_ref, rf.omega_ref, rf.alpha_ref. */
void Robot_Foward_Update(uint32_t run_samples)
{
    run_samples_now = run_samples;

    /* Start a queued move: at once if it was asked for while balancing,
       RF_START_DELAY_S after engaging if it was asked for before */
    if ((rf.state == RF_QUEUED) && (run_samples >= start_after)) {
        x_start           = x_hold;
        enc_at_start      = Encoder_Distance_m();
        move_samples      = 0U;
        still_samples     = 0U;
        rf.follow_err_max = 0.0f;
        rf.state          = RF_ACCEL;
        RF_PRINT("fwd: moving\n");
    }

    if (rf.state == RF_BRAKE) {
        rf_brake_step();
        return;
    }
    if (!rf_moving()) {
        rf_hold();
        return;
    }

    /* Where the move says the robot should be now */
    float D = fabsf(rf.distance_m);
    rf.t = (float)move_samples * RF_TS_S;
    move_samples++;
    rf_path(rf.t, &rf.dist_ref, &rf.vel_ref, &rf.acc_ref);

    /* How far the encoders say it has gone since the move started */
    float enc_now = Encoder_Distance_m();
    rf.dist_meas  = dir * (enc_now - enc_at_start);
    rf.follow_err = rf.dist_ref - rf.dist_meas;
    if (fabsf(rf.follow_err) > rf.follow_err_max) {
        rf.follow_err_max = fabsf(rf.follow_err);
    }
    if (fabsf(rf.follow_err) > RF_FOLLOW_ERR_M) {
        /* Start again from where the robot really is. Its speed along the
           move, but no more than the plan's (the encoder speed also carries
           the robot's balancing wobble). */
        float v = dir * Encoder_Speed_mps();
        if (v < 0.0f) {
            v = 0.0f;
        } else if (v > rf.vel_ref) {
            v = rf.vel_ref;
        }
        rf_brake(enc_now - enc_at_engage, dir * v, 0.0f,
                 "too far from the planned position (blocked, or the move is too "
                 "fast for the robot)");
        rf_brake_step();
        return;
    }

    /* For the controller: wheel angle and its rates, signed */
    rf.phi_ref   = (x_start + dir * rf.dist_ref) / ENC_WHEEL_RADIUS_M;
    rf.omega_ref = dir * rf.vel_ref / ENC_WHEEL_RADIUS_M;
    rf.alpha_ref = dir * rf.acc_ref / ENC_WHEEL_RADIUS_M;

    if (rf.t < rf.t_accel) {
        rf.state = RF_ACCEL;
    } else if (rf.t < rf.t_accel + rf.t_cruise) {
        rf.state = RF_CRUISE;
    } else if (rf.t < rf.t_total) {
        rf.state = RF_DECEL;
    } else {
        /* The plan has finished and phi_ref sits on the target. Wait for the
           encoders to agree and the wheels to be nearly still. */
        if (rf.state != RF_SETTLE) {
            rf.state       = RF_SETTLE;
            rf.dist_at_end = rf.dist_meas;          /* how well it kept to the time */
        }
        uint8_t timed_out = (rf.t > rf.t_total + RF_SETTLE_TIMEOUT_S) ? 1U : 0U;
        if ((fabsf(D - rf.dist_meas) < RF_DONE_TOL_M) &&
            (fabsf(Encoder_Speed_mps()) < RF_DONE_SPEED_MPS)) {
            still_samples++;
        } else {
            still_samples = 0U;
        }
        if ((still_samples >= RF_SAMPLES(RF_DONE_HOLD_S)) || timed_out) {
            x_hold        = x_start + dir * D;      /* keep holding the target */
            rf.state      = RF_DONE;
            rf.note       = timed_out ? "holding the target, but not settled within "
                                        "RF_DONE_TOL_M of it" : "arrived";
            rf.time_taken = rf.t;
            rf.dist_final = rf.dist_meas;
            RF_PRINT("fwd: done. At the planned %ld ms the encoders read %ld mm of %ld mm. "
                     "%s %ld ms later at %ld mm. Furthest from plan %ld mm.\n",
                     RF_MM(rf.t_total), RF_MM(rf.dist_at_end), RF_MM(D),
                     timed_out ? "Still not settled" : "Settled",
                     RF_MM(rf.time_taken - rf.t_total), RF_MM(rf.dist_final),
                     RF_MM(rf.follow_err_max));
        }
    }
}

/* ========================================================================= */
/* Public                                                                    */
/* ========================================================================= */
/*
 * Robot_Foward( distance, average velocity, peak acceleration, acceleration gradient )
 *
 *   distance               metres. + = forwards, - = backwards.
 *   average velocity       m/s = distance / time for the whole move, speeding
 *                          up and slowing down included. The curve is fitted
 *                          to it (see "Fitting the curve" at the top).
 *                          Put 'N' here to ignore it and use the curve alone.
 *   peak acceleration      m/s^2. The top of the parabola (its y offset).
 *   acceleration gradient  m/s^3. Slope of the acceleration curve where it
 *                          leaves zero: bigger = sharper, shorter pulse.
 *                          Pulse length = 4 x peak / gradient.
 *
 *   Examples
 *     Robot_Foward(0.5f, 0.10f, 0.20f, 0.40f);   0.5 m in 5 s (curve lowered
 *                                                to peak 0.125, gradient 0.25)
 *     Robot_Foward(0.5f, 0.10f, 0.15f, 0.60f);   0.5 m in 5 s (peak kept,
 *                                                gradient lowered to 0.43)
 *     Robot_Foward(0.5f, 'N',   0.10f, 0.20f);   0.5 m, curve as given (5.75 s)
 *     Robot_Foward(-0.3f, 0.05f, 0.20f, 0.40f);  0.3 m backwards in 6 s
 *
 *   Call it after Balance_Start(seconds) in main.c: the robot is then
 *   balancing and the move starts at once. A Balance_Start(seconds) after
 *   it waits for the move to finish, then balances in place that long:
 *       Balance_Start(3.0f);  Robot_Foward(0.5f, 0.10f, 0.20f, 0.40f);
 *       Balance_Start(2.0f);  Robot_Foward(-0.5f, 0.10f, 0.20f, 0.40f);
 *       Balance_Start('N');
 *   Called before the robot is balancing, the move waits until it has been
 *   balancing for RF_START_DELAY_S. One move at a time: it returns 0 while
 *   one is queued or running.
 *   Returns 1 = accepted, 0 = refused (the serial port and rf.note say why).
 */
uint8_t Robot_Foward(float distance_m, float avg_velocity_mps,
                     float peak_accel_mps2, float accel_gradient_mps3)
{
    if ((rf.state == RF_QUEUED) || rf_moving() || (rf.state == RF_BRAKE)) {
        Lab5_printf("fwd: refused, a move is already queued or running (fwd stop cancels it)\n");
        return 0U;
    }

    uint8_t use_velocity = ((avg_velocity_mps == RF_NO_VELOCITY) ||
                            (avg_velocity_mps == (float)'n')) ? 0U : 1U;

    if (!rf_plan(fabsf(distance_m), use_velocity, avg_velocity_mps,
                 peak_accel_mps2, accel_gradient_mps3)) {
        rf.state = RF_IDLE;
        Lab5_printf("fwd: refused: %s\n", rf.note);
        return 0U;
    }

    rf.distance_m = distance_m;
    dir           = (distance_m < 0.0f) ? -1.0f : 1.0f;
    rf.note       = "";
    rf.state      = RF_QUEUED;       /* Robot_Foward_Update() starts it */
    if (balancing) {
        start_after = 0U;            /* already balancing: next sample */
    } else {
        start_after = RF_SAMPLES(RF_START_DELAY_S);
        Lab5_printf("fwd: queued. Starts %ld ms after the robot begins balancing.\n",
                    RF_MM(RF_START_DELAY_S));
    }
    return 1U;
}

void Robot_Foward_Stop(void)
{
    if (rf.state == RF_QUEUED) {
        rf.state = RF_IDLE;
        rf.note  = "queued move cancelled";
        Lab5_printf("fwd: queued move cancelled\n");
    } else if ((rf.state == RF_DECEL) || (rf.state == RF_SETTLE)) {
        Lab5_printf("fwd: already slowing down; the move ends at the target\n");
    } else if (rf_moving()) {
        /* Carry on from where the plan is now, and bring it to a halt */
        rf_brake(x_start + dir * rf.dist_ref, dir * rf.vel_ref, dir * rf.acc_ref,
                 "stopped (fwd stop)");
    } else {
        Lab5_printf("fwd: no move to stop\n");
    }
}

uint8_t Robot_Foward_Busy(void)
{
    return ((rf.state == RF_QUEUED) || rf_moving() || (rf.state == RF_BRAKE)) ? 1U : 0U;
}

/* Serial command (Lab5.c's command table):
     fwd                                     status
     fwd <distance_m> <velocity_mps|N> <peak_accel> <gradient>
     fwd stop                                stop the move, hold position */
void Robot_Foward_Cmd(int argc, char *argv[])
{
    static const char *const names[] = {
        "IDLE", "QUEUED", "ACCEL", "CRUISE", "DECEL", "SETTLE", "BRAKE", "DONE", "STOPPED"
    };

    if (argc == 5) {
        float v = ((argv[2][0] == 'N') || (argv[2][0] == 'n')) ? RF_NO_VELOCITY
                                                               : (float)atof(argv[2]);
        Robot_Foward((float)atof(argv[1]), v, (float)atof(argv[3]), (float)atof(argv[4]));
    } else if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
        Robot_Foward_Stop();
    } else if (argc == 1) {
        /* Whole numbers (mm, ms): this can run while the robot balances */
        Lab5_printf("fwd: %s%s%s\n", names[rf.state], (rf.note[0] != '\0') ? ": " : "", rf.note);
        Lab5_printf("  plan: %ld mm, average %ld mm/s, peak %ld mm/s2, gradient %ld mm/s3, "
                    "cruise %ld mm/s\n",
                    RF_MM(rf.distance_m), RF_MM(rf.avg_velocity), RF_MM(rf.peak_accel),
                    RF_MM(rf.gradient), RF_MM(rf.cruise_speed));
        Lab5_printf("        speed up %ld ms, cruise %ld ms, total %ld ms\n",
                    RF_MM(rf.t_accel), RF_MM(rf.t_cruise), RF_MM(rf.t_total));
        Lab5_printf("  now:  t %ld ms, planned %ld mm, encoders %ld mm, difference %ld mm "
                    "(largest %ld)\n",
                    RF_MM(rf.t), RF_MM(rf.dist_ref), RF_MM(rf.dist_meas),
                    RF_MM(rf.follow_err), RF_MM(rf.follow_err_max));
        Lab5_printf("  last finished move: %ld mm at its planned end, settled at %ld mm "
                    "after %ld ms\n",
                    RF_MM(rf.dist_at_end), RF_MM(rf.dist_final), RF_MM(rf.time_taken));
    } else {
        Lab5_printf("usage: fwd <distance_m> <velocity_mps|N> <peak_accel_mps2> "
                    "<gradient_mps3>   |   fwd stop   |   fwd\n");
    }
}

/*
 * ---------------------------------------------------------------------------
 * The request this file was written from (Gene Connell, 18 Nov 2026), word
 * for word:
 *
 * I want you to set up a function that can be called before the balancing
 * function that will add the ramp to phi ref and the controller. The function
 * will take the speed ref and how far I want it to go in meters as an
 * argument. The function will use the encoder values to estimate the
 * distance. Put a distance estimate function into the encoder glue code if
 * there isn't already one for this purpose. once the robot approaches the
 * required distance use an algorithm to ramp down the speed to zero and aim
 * to have the robot in the normal balancing state at the end. I want the
 * acceleration to follow a smooth parabolic curve with t intercepts at 0.
 * Parameterize the y offset and the gradient of the curve so I can adjust it.
 * have the curve parameters as arguments of the function.   The function
 * will look like this: Robot_Foward(distance(m), adv velocity(m/s) (velocity
 * is taken as distance needed/time taken not as the gradient of the
 * acceleration curve), peak acceleration, acceleration gradient). if have an
 * algorithm adjust the acceleration curve after I set the values to fit the
 * required velocity (priorities the velocity value I set over the parameters
 * for the acceleration curve). If I put the character 'N' for the velocity,
 * ignore the velocity parameter and just use the acceleration curve. Put all
 * of this in a separate file called Robot_Foward_AIgen.c place this promt
 * and the bottom of the file in a comment and comment the function template
 * above the function so I remember what goes where.
 * ---------------------------------------------------------------------------
 */
