/**
 * policy_mem.c — registers a MEM_TYPE_APP handler (see mem.h) that routes
 * CRTP memory writes straight into policy.c's policyWeightsWrite(), which
 * owns the actual upload protocol (header reassembly/validation, CRC check,
 * the armed/disarmed safety gate). This file is deliberately thin: no
 * upload logic lives here, so policy.c's own logic stays host-testable
 * (test_policy_host.c) without pulling in mem.h/FreeRTOS at all.
 *
 * MEM_TYPE_APP (0x18) is the firmware's sanctioned extension point for
 * out-of-tree apps to expose custom read/write memory over CRTP (same
 * mechanism trajectory upload etc. use for their own MEM_TYPE_* — see
 * app_api/src/app_main.c's own MEM_TYPE_APP registration call, which only
 * demonstrates that the call compiles, not a full handler). Read-back isn't
 * implemented (.read left NULL, which memRead() already handles safely —
 * see mem.c) since there's nothing a caller needs to read back: verify an
 * upload landed via ctrlRace's weightsValid log variable (race_controller.c)
 * instead of reading raw bytes back — policyWeightsValid()/policyIsArmed()
 * are getters, not addressable statics, so they can't be LOG_ADD'd directly
 * from here; race_controller.c already runs every control tick and mirrors
 * them into a real log variable there.
 */
#include "policy.h"
#include "policy_mem.h"

#include "mem.h"

static uint32_t policyMemGetSize(const uint8_t internal_id)
{
  (void)internal_id;
  return policyMemSize();
}

static bool policyMemWrite(const uint8_t internal_id, const uint32_t memAddr,
                            const uint8_t writeLen, const uint8_t* buffer)
{
  (void)internal_id;
  return policyWeightsWrite(memAddr, writeLen, buffer);
}

static const MemoryHandlerDef_t policyMemDef = {
  .type = MEM_TYPE_APP,
  .getSize = policyMemGetSize,
  .write = policyMemWrite,
};

void policyMemInit(void)
{
  memoryRegisterHandler(&policyMemDef);
}
