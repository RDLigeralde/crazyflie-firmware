/**
 * app_race_policy.c — app entry point.
 *
 * controllerOutOfTree() (race_controller.c) already bypasses
 * controller_pid.c via mixer.c + controlModeForce/normalizedForces — see
 * that file's docstring. Its action input is still a bench-testing param
 * stand-in (ctrlRace group), not yet driven by a real observation stream:
 * the app-channel receive and the exported policy's forward pass are not
 * wired in yet. appMain() itself has nothing to do in the meantime — the
 * FreeRTOS stabilizer task calls controllerOutOfTree() on its own schedule,
 * independent of this app task — so it just idles.
 */
#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "RACEPOLICY"
#include "debug.h"

void appMain()
{
  DEBUG_PRINT("app_race_policy: mixer + controlModeForce wired, app-channel/policy not yet\n");
  while (1) {
    vTaskDelay(M2T(2000));
  }
}
