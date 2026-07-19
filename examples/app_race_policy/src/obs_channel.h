/**
 * obs_channel.h — reassembles POLICY_OBS_DIM-float observations streamed up
 * from the ground over the app-channel (see crazyflie_cpp's
 * Crazyflie::sendRaceObservation, which this is the receiving half of —
 * the two must agree on the chunking scheme, documented in both places).
 */
#ifndef OBS_CHANNEL_H
#define OBS_CHANNEL_H

#include <stdbool.h>

/**
 * Non-blocking: drains whatever app-channel packets are currently queued
 * (see appchannelReceiveDataPacket, APPCHANNEL_WAIT_FOREVER not used here —
 * this must never block the control loop that calls it) and reassembles
 * them into obsOut whenever a full POLICY_OBS_DIM-float frame completes.
 *
 * Call once per control tick. Returns true exactly on the tick a full,
 * freshly-completed frame is available in obsOut (obsOut is otherwise left
 * unmodified — the caller should hold onto its own last-known-good copy /
 * the policy's last output between completions, same zero-order-hold
 * pattern this firmware already uses for setpoints between updates).
 */
bool obsChannelPoll(float obsOut[/* POLICY_OBS_DIM */]);

#endif // OBS_CHANNEL_H
