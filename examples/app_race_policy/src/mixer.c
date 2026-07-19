#include "mixer.h"

float mixRpmAction(float action, float hoverRpm, float maxRpm)
{
  if (action <= 0.0f) {
    return (action + 1.0f) * hoverRpm;
  }
  return hoverRpm + (maxRpm - hoverRpm) * action;
}

static float clampf(float x, float lo, float hi)
{
  if (x < lo) { return lo; }
  if (x > hi) { return hi; }
  return x;
}

void mixAttitudeRpm(const float action[4], float hoverRpm, float maxRpm,
                     float differentialFrac, float rpmOut[4])
{
  float thrustNorm = action[0];
  float roll       = action[1];
  float pitch      = action[2];
  float yawRate    = action[3];

  float collective = mixRpmAction(thrustNorm, hoverRpm, maxRpm);
  float scale = differentialFrac * hoverRpm;

  rpmOut[0] = collective + roll * scale - pitch * scale - yawRate * scale;
  rpmOut[1] = collective - roll * scale - pitch * scale + yawRate * scale;
  rpmOut[2] = collective - roll * scale + pitch * scale - yawRate * scale;
  rpmOut[3] = collective + roll * scale + pitch * scale + yawRate * scale;

  for (int i = 0; i < 4; i++) {
    rpmOut[i] = clampf(rpmOut[i], 0.0f, maxRpm);
  }
}
