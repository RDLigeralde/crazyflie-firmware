/**
 * race_controller.c — controllerOutOfTree() hook that bypasses
 * controller_pid.c's closed-loop attitude-rate PID entirely, going straight
 * to per-motor RPM via mixer.c and controlModeForce/normalizedForces (see
 * power_distribution_quadrotor.c's powerDistributionForce() — direct
 * per-motor thrust fraction, no mixing). This is required, not optional:
 * every sim-trained checkpoint (attitude or rpm action_type) was trained
 * against an OPEN LOOP action->RPM mixer with no gyro feedback anywhere
 * (see mixer.h's own docstring) — routing through the real closed-loop PID
 * instead would substitute dynamics the policy never saw in training.
 *
 * Attitude (mixAttitudeRpm) is the SOLE control path here — the rpm
 * action_type / mixRpmAction path was removed after attitude-trained
 * checkpoints clearly outperformed rpm ones in training (this session's
 * own runs: attitude baseline ~97 mean_return vs. rpm's ~85-95 even after
 * entropy tuning — see race/ sweep history). policy.c/h (export_policy_c.py)
 * is exported from an attitude-action_type checkpoint; policy_export_rpm/
 * keeps the rpm checkpoint's export as a reference, not built. mixRpmAction
 * itself stays in mixer.c/h — mixAttitudeRpm calls it internally for the
 * collective-thrust term, it's not rpm-mode-specific.
 *
 * `action[0..3]` is fed either by the ground-streamed policy (obs_channel.c
 * + policy.c, when obsChannelEnable != 0 — see the ctrlRace param group)
 * or, when obsChannelEnable == 0 (the default), directly by the ctrlRace
 * param group itself — a bench-testing stand-in that lets the actuation
 * path (mixer -> kf -> normalizedForces -> real motors) be verified in
 * isolation, without a ground station or trained checkpoint in the loop
 * at all. Both paths flow through the exact same mixer/thrust code below;
 * only how action0..3 get set differs.
 */
#include "controller.h"
#include "power_distribution.h"
#include "mixer.h"
#include "obs_channel.h"
#include "policy.h"
#include "policy_mem.h"

#include "param.h"
#include "log.h"

// 0 (default): action0..3 come only from the ctrlRace param group below
// (bench-testing — see file docstring). 1: obsChannelPoll()/policyForward()
// overwrite action0..3 every time a full observation frame arrives from
// the ground; between frames the last policy output is held (zero-order
// hold), same as this firmware already does for setpoints between updates.
// policy.h's POLICY_OBS_DIM/POLICY_ACTION_DIM are baked in by
// export_policy_c.py from whichever checkpoint was exported — must be an
// attitude action_type checkpoint (see file docstring); an rpm-type
// checkpoint's raw output has different semantics and would be
// misinterpreted by mixAttitudeRpm below.
//
// obsChannelEnable is only the OPERATOR's intent to arm — the policy loop
// below only actually runs when that intent is ALSO backed by
// policyWeightsValid() (see policy.h/policy_mem.c): a ground-station weight
// upload (policyWeightsWrite(), MEM_TYPE_APP) that's in progress or that
// failed its CRC check can never get armed no matter what this param is
// set to. armedLog/weightsValidLog mirror the live (gated) state, not just
// the raw request, so a rejected arm attempt is visible over the ctrlRace
// LOG_GROUP below rather than silently doing nothing.
static uint8_t obsChannelEnable = 0;
static uint32_t obsFramesReceived = 0;
static uint8_t armedLog = 0;
static uint8_t weightsValidLog = 0;

// Per-vehicle dynamics constants — defaults are dmcdrones' MJXVectorAviary
// nominals (kf=3.16e-10, hover derived from mass=0.027kg at g=9.81,
// max_rpm=21714, differential_frac=0.02). Tunable via param so a real
// vehicle's calibrated values (from system ID, not simulation) can override
// these without a recompile/reflash — same mechanism ltfly's rlt.* params
// and this project's own mode-switching plan already rely on.
static float hoverRpm = 14475.81f;
static float maxRpm = 21714.0f;
static float kf = 3.16e-10f;
static float differentialFrac = 0.02f;

// Bench-testing action stand-in (see file docstring) — 0 everywhere is a
// safe default (thrustNorm=0 => hoverRpm on all motors, no differential —
// see mixAttitudeRpm). Never left at a non-hover value across a reset by
// controllerOutOfTreeInit() below, so a stale bench-test value from a
// previous flight can't silently carry into the next one.
static float action0 = 0.0f;
static float action1 = 0.0f;
static float action2 = 0.0f;
static float action3 = 0.0f;

// Exposed for ground-side bench-test logging/validation (see
// test_mixer_host.c for the offline numerical check this mirrors on-target).
static float rpmOut[4];
static float normalizedForcesOut[4];

void controllerOutOfTreeInit(void)
{
  action0 = 0.0f;
  action1 = 0.0f;
  action2 = 0.0f;
  action3 = 0.0f;
  policyMemInit();
}

bool controllerOutOfTreeTest(void)
{
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint,
                          const sensorData_t *sensors, const state_t *state,
                          const stabilizerStep_t stabilizerStep)
{
  // Not yet used: this controller doesn't feed back off measured
  // attitude/rate (see mixer.h's docstring — neither mixer does, by
  // design) or setpoint. Silencing unused-parameter warnings rather than
  // removing the parameters, since the real interface (controller.h)
  // requires this exact signature.
  (void)setpoint;
  (void)sensors;
  (void)state;
  (void)stabilizerStep;

  // Mirror the operator's arm request into policy.c, gated on the currently
  // loaded weights actually validating — see the field docstring above and
  // policy.h's policySetArmed(). This must run every tick, before the
  // obsChannelEnable check below: it's also what tells policyWeightsWrite()
  // whether it's safe to accept an in-progress ground-station upload right
  // now (never, while armed).
  bool armed = (obsChannelEnable != 0) && policyWeightsValid();
  policySetArmed(armed);
  armedLog = armed ? 1 : 0;
  weightsValidLog = policyWeightsValid() ? 1 : 0;

  if (armed) {
    float obs[POLICY_OBS_DIM];
    if (obsChannelPoll(obs)) {
      float policyAction[POLICY_ACTION_DIM];
      policyForward(obs, policyAction);
      // Clip to [-1, 1] — matches training-time env.step()'s own
      // unconditional clip (see mixer.h's docstring: both mixers assume a
      // pre-clipped input, they don't clip it themselves).
      float a0 = policyAction[0], a1 = policyAction[1];
      float a2 = policyAction[2], a3 = policyAction[3];
      if (a0 < -1.0f) { a0 = -1.0f; } else if (a0 > 1.0f) { a0 = 1.0f; }
      if (a1 < -1.0f) { a1 = -1.0f; } else if (a1 > 1.0f) { a1 = 1.0f; }
      if (a2 < -1.0f) { a2 = -1.0f; } else if (a2 > 1.0f) { a2 = 1.0f; }
      if (a3 < -1.0f) { a3 = -1.0f; } else if (a3 > 1.0f) { a3 = 1.0f; }
      action0 = a0;
      action1 = a1;
      action2 = a2;
      action3 = a3;
      obsFramesReceived++;
    }
    // No new frame this tick: fall through and keep using the last
    // action0..3 (zero-order hold) — same as the bench-test path below.
  }

  float action[4] = {action0, action1, action2, action3};
  mixAttitudeRpm(action, hoverRpm, maxRpm, differentialFrac, rpmOut);

  // force_i = kf * rpm_i^2 (N) -> fraction of one motor's calibrated max
  // thrust. powerDistributionGetMaxThrust() is the TOTAL across all
  // motors (see its own docstring in power_distribution.h) — divide by
  // STABILIZER_NR_OF_MOTORS for the per-motor figure powerDistributionForce()
  // expects normalizedForces to be relative to. Read every call (not
  // cached) since it can change if idle-thrust/battery-compensation config
  // changes at runtime; the cost is negligible next to the physics step.
  float perMotorMaxThrust = powerDistributionGetMaxThrust() / STABILIZER_NR_OF_MOTORS;
  for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++) {
    float force = kf * rpmOut[i] * rpmOut[i];
    // Not clamped here: powerDistributionForce() (power_distribution_quadrotor.c)
    // already clips normalizedForces to [0, 1] itself before scaling to
    // UINT16_MAX — an out-of-range value here is handled downstream, not a
    // bug in this function specifically.
    normalizedForcesOut[i] = force / perMotorMaxThrust;
    control->normalizedForces[i] = normalizedForcesOut[i];
  }
  control->controlMode = controlModeForce;
}

PARAM_GROUP_START(ctrlRace)
PARAM_ADD(PARAM_UINT8, obsChanEnable, &obsChannelEnable)
PARAM_ADD(PARAM_FLOAT, hoverRpm, &hoverRpm)
PARAM_ADD(PARAM_FLOAT, maxRpm, &maxRpm)
PARAM_ADD(PARAM_FLOAT, kf, &kf)
PARAM_ADD(PARAM_FLOAT, differentialFrac, &differentialFrac)
PARAM_ADD(PARAM_FLOAT, action0, &action0)
PARAM_ADD(PARAM_FLOAT, action1, &action1)
PARAM_ADD(PARAM_FLOAT, action2, &action2)
PARAM_ADD(PARAM_FLOAT, action3, &action3)
PARAM_GROUP_STOP(ctrlRace)

LOG_GROUP_START(ctrlRace)
LOG_ADD(LOG_FLOAT, rpm0, &rpmOut[0])
LOG_ADD(LOG_FLOAT, rpm1, &rpmOut[1])
LOG_ADD(LOG_FLOAT, rpm2, &rpmOut[2])
LOG_ADD(LOG_FLOAT, rpm3, &rpmOut[3])
LOG_ADD(LOG_FLOAT, nf0, &normalizedForcesOut[0])
LOG_ADD(LOG_FLOAT, nf1, &normalizedForcesOut[1])
LOG_ADD(LOG_FLOAT, nf2, &normalizedForcesOut[2])
LOG_ADD(LOG_FLOAT, nf3, &normalizedForcesOut[3])
LOG_ADD(LOG_UINT32, obsFrames, &obsFramesReceived)
LOG_ADD(LOG_UINT8, armed, &armedLog)
LOG_ADD(LOG_UINT8, weightsValid, &weightsValidLog)
LOG_GROUP_STOP(ctrlRace)
