/**
 * race_controller.c — controllerOutOfTree() hook. Arbitrates between the
 * stock PID controller and the ground-streamed policy, per flight phase.
 *
 * CONFIG_CONTROLLER_OOT=y makes this function the SOLE controller for the
 * entire flight (controller.c's autoselect chain picks ControllerTypeOot,
 * and controller() dispatches every tick through this one function
 * pointer) — PID/Lee/Mellinger are never reached on their own. So anything
 * this function does not handle itself, nothing else will: takeoff, hover
 * and land included. It therefore delegates to controllerPid() for every
 * phase except an actively-streaming race (see selectMode() below).
 *
 * Getting this wrong is not subtle. Before this arbitration existed, this
 * function discarded `setpoint` outright and ran mixAttitudeRpm() on all
 * four phases: the workstation's CTBR takeoff setpoints were thrown away
 * on arrival and the vehicle instead flew open-loop constant hoverRpm with
 * no attitude feedback of any kind, which diverges immediately. That is a
 * tip-over on takeoff, every time.
 *
 * During a race the policy path DOES bypass PID, going straight to
 * per-motor RPM via mixer.c and controlModeForce/normalizedForces (see
 * power_distribution_quadrotor.c's powerDistributionForce() — direct
 * per-motor thrust fraction, no mixing). That part is required, not
 * optional: every sim-trained checkpoint (attitude or rpm action_type) was
 * trained against an OPEN LOOP action->RPM mixer with no gyro feedback
 * anywhere (see mixer.h's own docstring) — routing a policy action through
 * the real closed-loop PID would substitute dynamics the policy never saw
 * in training. The bypass is correct; applying it to takeoff was the bug.
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
 * + policy.c, in MODE_POLICY) or, in MODE_BENCH, directly by the ctrlRace
 * param group itself — a bench-testing stand-in that lets the actuation
 * path (mixer -> kf -> normalizedForces -> real motors) be verified in
 * isolation, without a ground station or trained checkpoint in the loop
 * at all. Both paths flow through the exact same mixer/thrust code below;
 * only how action0..3 get set differs.
 */
#include "controller.h"
#include "controller_pid.h"
#include "power_distribution.h"
#include "mixer.h"
#include "obs_channel.h"
#include "policy.h"
#include "policy_mem.h"

#include "param.h"
#include "log.h"

// obsChannelEnable is only the OPERATOR's intent to arm the policy — the
// policy path only actually runs when that intent is ALSO backed by
// policyWeightsValid() (see policy.h/policy_mem.c): a ground-station weight
// upload (policyWeightsWrite(), MEM_TYPE_APP) that's in progress or that
// failed its CRC check can never get armed no matter what this param is
// set to.
//
// It is deliberately NOT sufficient on its own to hand the vehicle to the
// policy. The documented bringup sequence sets obsChanEnable=1 while
// DISARMED, i.e. before takeoff — so this param cannot distinguish "about
// to race" from "about to take off", and gating on it alone would put the
// policy in control of the takeoff. Observation freshness is what actually
// separates the phases; see obsStaleTicks.
static uint8_t obsChannelEnable = 0;
static uint32_t obsFramesReceived = 0;
static uint8_t weightsValidLog = 0;

// THE phase gate, not merely a safety net. crazyflie_ros' controller_utils.c
// publishes /race_obs only while its FSM is in the 'racing' state — during
// takeoff/hover/land it sends ordinary CTBR setpoints instead and streams
// no observations at all. So "observations are arriving" is exactly
// equivalent to "the ground station believes we are racing", and it is the
// only signal available onboard that tracks the flight phase.
//
// This doubles as the link-loss failsafe: if the radio drops mid-lap the
// stream stops, this goes stale within obsStaleTicks, and control reverts
// to PID rather than holding the last policy action forever (which would
// be a flyaway at whatever thrust the policy last commanded).
//
// stabilizerStep_t is a uint32_t tick count at 1000 Hz (stabilizer_types.h),
// so obsStaleTicks is milliseconds. Exposed as a param because reflashing
// this vehicle is expensive — this is the number most likely to need
// tuning against real mocap/radio jitter. Too tight and the mode flaps
// mid-lap; too loose and a dropped link coasts further before recovering.
static uint16_t obsStaleTicks = 50;
static stabilizerStep_t lastObsTick = 0;
static bool obsEverReceived = false;

// Bench-test escape hatch. Before this file arbitrated modes at all, an
// obsChanEnable=0 vehicle ran the mixer off the action0..3 params, which is
// how the actuation path was verified on the bench. That is now the PID
// path, so preserving bench testing needs its own explicit opt-in. Forces
// MODE_BENCH regardless of everything else — never set this on a vehicle
// that is expected to fly under PID.
static uint8_t benchEnable = 0;

// Live mode, mirrored for ground-side logging: 0=PID, 1=policy, 2=bench.
// Log this alongside ctrlRace.obsFrames when diagnosing a flight — it is
// the ground truth for which controller actually produced the motor
// commands on any given tick.
typedef enum {
  MODE_PID    = 0,
  MODE_POLICY = 1,
  MODE_BENCH  = 2,
} RaceMode;
static uint8_t modeLog = MODE_PID;

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
  lastObsTick = 0;
  obsEverReceived = false;
  modeLog = MODE_PID;
  // Required: nothing else initializes PID when CONFIG_CONTROLLER_OOT=y,
  // because controller.c only ever init()s the one selected controller —
  // which is this one. Safe to call here and safe to call repeatedly in
  // general; stabilizer.c does exactly that on every runtime controller
  // switch.
  controllerPidInit();
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
  // Every parameter is now genuinely used — setpoint/sensors/state all flow
  // through to controllerPid() below. The unused-parameter casts that used
  // to sit here were the visible symptom of the takeoff bug described in
  // this file's docstring; they are gone on purpose.

  // Mirror the operator's arm request into policy.c, gated on the currently
  // loaded weights actually validating — see policy.h's policySetArmed().
  // This must run every tick, before any mode branch below: it's also what
  // tells policyWeightsWrite() whether it's safe to accept an in-progress
  // ground-station upload right now (never, while armed).
  bool armed = (obsChannelEnable != 0) && policyWeightsValid();
  policySetArmed(armed);
  weightsValidLog = policyWeightsValid() ? 1 : 0;

  // Drain the app-channel every tick regardless of mode. Doing this outside
  // the mode branch matters: it's what lets a fresh frame arriving during
  // PID flight promote us into MODE_POLICY on this very tick, and it keeps
  // the queue from backing up with stale frames while we're not racing.
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
      lastObsTick = stabilizerStep;
      obsEverReceived = true;
    }
    // No new frame this tick: the last policy output is held (zero-order
    // hold) for up to obsStaleTicks, then we fall back to PID below.
  }

  // Unsigned subtraction on a monotonic uint32_t tick counter, so this is
  // wrap-safe. obsEverReceived guards the boot case: without it, a
  // never-written lastObsTick of 0 would read as perfectly fresh on tick 0
  // and hand a just-powered-on vehicle straight to the policy.
  bool obsFresh = obsEverReceived &&
                  ((stabilizerStep - lastObsTick) <= (stabilizerStep_t)obsStaleTicks);

  RaceMode mode = MODE_PID;
  if (benchEnable != 0) {
    mode = MODE_BENCH;
  } else if (armed && obsFresh) {
    mode = MODE_POLICY;
  }
  modeLog = (uint8_t)mode;

  if (mode == MODE_PID) {
    // Takeoff, hover, land, pre-race idle, and any loss of the observation
    // stream all land here. controllerPid() writes control->controlMode
    // (controlModeLegacy) itself, so the mode alternation across a race
    // boundary is handled downstream by powerDistribution()'s own switch.
    //
    // Deliberately NOT re-initializing PID on the policy->PID edge: its
    // integrators sit frozen while the policy flies, so they resume at
    // their pre-race hover values. Zeroing them instead would drop the
    // z-integral that was compensating hover thrust bias, and the vehicle
    // would sag on handback before rebuilding it.
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    return;
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
PARAM_ADD(PARAM_UINT16, obsStaleTicks, &obsStaleTicks)
PARAM_ADD(PARAM_UINT8, benchEnable, &benchEnable)
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
// 0=PID, 1=policy, 2=bench. Replaces the old `armed` log, which reported
// the arm REQUEST and so read 1 all through a PID-flown takeoff; this
// reports which controller actually drove the motors.
LOG_ADD(LOG_UINT8, mode, &modeLog)
LOG_ADD(LOG_UINT8, weightsValid, &weightsValidLog)
LOG_GROUP_STOP(ctrlRace)
