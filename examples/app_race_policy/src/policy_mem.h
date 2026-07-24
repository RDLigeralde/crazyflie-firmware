/**
 * policy_mem.h — registers policy.c's weight buffer as a CRTP MEM_TYPE_APP
 * memory (see mem.h), so a ground-station tool
 * (upload_policy_weights.py) can overwrite the active policy at runtime —
 * see policy.h's policyWeightsWrite() for the actual upload protocol; this
 * file is only the thin glue between that and the firmware's generic
 * memory subsystem.
 */
#ifndef POLICY_MEM_H
#define POLICY_MEM_H

void policyMemInit(void);

#endif // POLICY_MEM_H
