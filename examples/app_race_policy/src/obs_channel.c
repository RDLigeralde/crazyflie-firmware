#include <string.h>
#include <stdint.h>

#include "obs_channel.h"
#include "policy.h"
#include "app_channel.h"

// Must match crazyflie_cpp's Crazyflie::sendRaceObservation exactly (see
// its doc comment in Crazyflie.h) — 1 seq byte + 7 floats (28 bytes) = 29
// bytes, matching APPCHANNEL_MTU (30) with the CRTP header byte accounted
// for on the wire (not present in what appchannelReceiveDataPacket returns
// here — that's payload only).
#define FLOATS_PER_CHUNK 7
#define EXPECTED_CHUNKS ((POLICY_OBS_DIM + FLOATS_PER_CHUNK - 1) / FLOATS_PER_CHUNK)

typedef struct {
  uint8_t seq;
  float values[FLOATS_PER_CHUNK];
} __attribute__((packed)) ObsChunkPacket;

static float assembling[POLICY_OBS_DIM];
static bool chunkReceived[EXPECTED_CHUNKS];

bool obsChannelPoll(float obsOut[/* POLICY_OBS_DIM */])
{
  bool completedThisPoll = false;
  ObsChunkPacket pkt;

  // Drain everything currently queued, not just one packet per tick — the
  // ground sends a whole frame's worth of chunks back-to-back every
  // control step, most likely faster than this gets polled.
  while (appchannelReceiveDataPacket(&pkt, sizeof(pkt), 0) == sizeof(pkt)) {
    if (pkt.seq >= EXPECTED_CHUNKS) {
      continue; // stale/corrupt seq — ignore rather than write out of bounds
    }
    int offset = pkt.seq * FLOATS_PER_CHUNK;
    int count = POLICY_OBS_DIM - offset;
    if (count > FLOATS_PER_CHUNK) {
      count = FLOATS_PER_CHUNK;
    }
    memcpy(&assembling[offset], pkt.values, count * sizeof(float));
    chunkReceived[pkt.seq] = true;

    bool allReceived = true;
    for (int i = 0; i < EXPECTED_CHUNKS; i++) {
      if (!chunkReceived[i]) {
        allReceived = false;
        break;
      }
    }
    if (allReceived) {
      // A later frame completing later in this same drain loop overwrites
      // obsOut again below — "most recent completed frame wins" for a
      // control loop polling slower than the obs stream arrives.
      memcpy(obsOut, assembling, sizeof(assembling));
      for (int i = 0; i < EXPECTED_CHUNKS; i++) {
        chunkReceived[i] = false;
      }
      completedThisPoll = true;
    }
  }
  return completedThisPoll;
}
