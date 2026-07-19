/**
 * test_mixer_host.c — standalone host-side numerical check of mixer.c
 * against reference values computed directly from mjc_dronetests/dmcdrones'
 * mixer.py (the Python original every sim checkpoint's action space was
 * trained against). Not part of the firmware's own Kbuild/Ceedling test
 * harness (no ruby/ceedling in the environment this was ported in) — this
 * is pure C, no CF/FreeRTOS dependencies, and builds with a plain host
 * compiler:
 *
 *   gcc -std=c99 -Wall -Wextra -I. test_mixer_host.c mixer.c -o test_mixer_host -lm
 *   ./test_mixer_host
 *
 * Reference values regenerated with:
 *   hover_rpm = sqrt(mass*g/(4*kf)), mass=0.027, g=9.81, kf=3.16e-10
 *   max_rpm = 21714.0, differential_frac = 0.02
 *   (dmcdrones' MJXVectorAviary's own nominal constants)
 */
#include <stdio.h>
#include <math.h>
#include "mixer.h"

static int failures = 0;

static void check(const char *name, float got, float want) {
    float diff = fabsf(got - want);
    // float32 vs a float64-computed reference: relative tolerance, since
    // absolute error scales with magnitude (~1e4 RPM here) at float32
    // precision (~7 significant digits) — not a fixed absolute tolerance.
    float tol = fmaxf(1e-6f * fabsf(want), 1e-4f);
    if (diff > tol) {
        printf("FAIL %-40s got=%.10f want=%.10f diff=%.10f\n", name, got, want, diff);
        failures++;
    } else {
        printf("ok   %-40s got=%.10f want=%.10f\n", name, got, want);
    }
}

int main(void) {
    const float hoverRpm = 14475.809152959684f;
    const float maxRpm = 21714.0f;
    const float diffFrac = 0.02f;

    printf("--- mixRpmAction ---\n");
    check("action=-1.00", mixRpmAction(-1.00f, hoverRpm, maxRpm), 0.0000000000f);
    check("action=-0.50", mixRpmAction(-0.50f, hoverRpm, maxRpm), 7237.9045764798f);
    check("action=+0.00", mixRpmAction(0.00f, hoverRpm, maxRpm), 14475.8091529597f);
    check("action=+0.50", mixRpmAction(0.50f, hoverRpm, maxRpm), 18094.9045764798f);
    check("action=+1.00", mixRpmAction(1.00f, hoverRpm, maxRpm), 21714.0000000000f);

    printf("--- mixAttitudeRpm ---\n");
    {
        float rpm[4];
        float a[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[0,0,0,0][0]", rpm[0], 14475.8091529597f);
        check("[0,0,0,0][1]", rpm[1], 14475.8091529597f);
        check("[0,0,0,0][2]", rpm[2], 14475.8091529597f);
        check("[0,0,0,0][3]", rpm[3], 14475.8091529597f);
    }
    {
        float rpm[4];
        float a[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[1,0,0,0][0]", rpm[0], 21714.0000000000f);
        check("[1,0,0,0][3]", rpm[3], 21714.0000000000f);
    }
    {
        float rpm[4];
        float a[4] = {-1.0f, 0.0f, 0.0f, 0.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[-1,0,0,0][0]", rpm[0], 0.0000000000f);
    }
    {
        float rpm[4];
        float a[4] = {0.0f, 1.0f, 0.0f, 0.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[0,1,0,0][0]", rpm[0], 14765.3253360189f);
        check("[0,1,0,0][1]", rpm[1], 14186.2929699005f);
        check("[0,1,0,0][2]", rpm[2], 14186.2929699005f);
        check("[0,1,0,0][3]", rpm[3], 14765.3253360189f);
    }
    {
        float rpm[4];
        float a[4] = {0.0f, 0.0f, 1.0f, 0.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[0,0,1,0][0]", rpm[0], 14186.2929699005f);
        check("[0,0,1,0][1]", rpm[1], 14186.2929699005f);
        check("[0,0,1,0][2]", rpm[2], 14765.3253360189f);
        check("[0,0,1,0][3]", rpm[3], 14765.3253360189f);
    }
    {
        float rpm[4];
        float a[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[0,0,0,1][0]", rpm[0], 14186.2929699005f);
        check("[0,0,0,1][1]", rpm[1], 14765.3253360189f);
        check("[0,0,0,1][2]", rpm[2], 14186.2929699005f);
        check("[0,0,0,1][3]", rpm[3], 14765.3253360189f);
    }
    {
        float rpm[4];
        float a[4] = {0.5f, 0.3f, -0.2f, 0.1f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[.5,.3,-.2,.1][0]", rpm[0], 18210.7110497035f);
        check("[.5,.3,-.2,.1][1]", rpm[1], 18094.9045764798f);
        check("[.5,.3,-.2,.1][2]", rpm[2], 17921.1948666443f);
        check("[.5,.3,-.2,.1][3]", rpm[3], 18152.8078130917f);
    }
    {
        // Saturating case: verifies the clamp matches np.clip's behavior
        float rpm[4];
        float a[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[1,1,1,1][0] (clamp hi)", rpm[0], 21424.4838169408f);
        check("[1,1,1,1][3] (clamp hi)", rpm[3], 21714.0000000000f);
    }
    {
        float rpm[4];
        float a[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
        mixAttitudeRpm(a, hoverRpm, maxRpm, diffFrac, rpm);
        check("[-1,-1,-1,-1][0] (clamp lo)", rpm[0], 289.5161830592f);
        check("[-1,-1,-1,-1][3] (clamp lo)", rpm[3], 0.0000000000f);
    }

    if (failures == 0) {
        printf("ALL_MIXER_CHECKS_PASSED\n");
        return 0;
    } else {
        printf("%d CHECK(S) FAILED\n", failures);
        return 1;
    }
}
