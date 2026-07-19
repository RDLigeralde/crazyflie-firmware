/**
 * mixer.h — action -> per-motor RPM conversion, ported from
 * mjc_dronetests/dmcdrones' multi_drone_mujoco/utils/mixer.py (the exact
 * mapping every sim-trained race checkpoint was trained against — see that
 * file's module docstring on why this math must live in exactly one place).
 *
 * Both mixers here are pure, stateless, open-loop formulas — no feedback
 * from measured angular velocity anywhere (mixAttitudeRpm's roll/pitch/
 * yaw_rate terms are fixed-scale RPM differentials, not a real rate
 * controller). This is deliberate: the real Crazyflie's stock closed-loop
 * attitude-rate PID (controller_pid.c) was never part of any checkpoint's
 * training dynamics, so it must be bypassed at deployment (see
 * controlModeForce/normalizedForces in power_distribution_quadrotor.c) —
 * these two functions are what actually replaces it, for whichever
 * action_type the loaded checkpoint used.
 */
#ifndef MIXER_H
#define MIXER_H

/**
 * Direct per-motor normalized RPM command — mix_rpm_action's port.
 *
 * action<=0 -> linear from 0 RPM (at -1) to hoverRpm (at 0).
 * action>0  -> linear from hoverRpm (at 0) to maxRpm (at +1).
 *
 * Two segments (not one line through [-1,1]) because hoverRpm isn't the
 * midpoint of [0, maxRpm] — this reaches the true floor/ceiling on both
 * sides while keeping action=0 an exact hover. Caller must clip `action` to
 * [-1, 1] first (matching env.step()'s own clip in training) — this
 * function does not clip its input.
 */
float mixRpmAction(float action, float hoverRpm, float maxRpm);

/**
 * CTBR-ish attitude mixer — mix_attitude_rpm's port. action =
 * [thrustNorm, roll, pitch, yawRate], each expected in [-1, 1] (not
 * clipped here). Writes per-motor RPM into rpmOut[4], each clipped to
 * [0, maxRpm].
 *
 * Collective reuses mixRpmAction's two-segment mapping; roll/pitch/yawRate
 * are fixed-scale per-motor RPM differentials (see mixer.h's own top
 * docstring on why — no gyro feedback anywhere in this formula).
 *
 * Motor order matches mix_attitude_rpm.py's array literal exactly:
 *   rpmOut[0] = collective + roll*scale - pitch*scale - yawRate*scale
 *   rpmOut[1] = collective - roll*scale - pitch*scale + yawRate*scale
 *   rpmOut[2] = collective - roll*scale + pitch*scale - yawRate*scale
 *   rpmOut[3] = collective + roll*scale + pitch*scale + yawRate*scale
 * — do not reorder without re-checking against the sim's own motor/rotor
 * layout (mjc_dronetests' _generate_xml / dmcdrones' quadrotor X-frame).
 */
void mixAttitudeRpm(const float action[4], float hoverRpm, float maxRpm,
                     float differentialFrac, float rpmOut[4]);

#endif // MIXER_H
