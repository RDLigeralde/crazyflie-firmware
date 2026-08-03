# app_race_policy

Onboard neural-network control for racing. The workstation streams observations over the app-channel while the Crazyflie runs the policy forward pass itself and drives per-motor RPM directly, bypassing the stock attitude-rate PID.

- **Policy:** fp16 MLP exported from an [`mjx-drone-trainer`](https://github.com/Jirl-upenn/mjx-drone-trainer) checkpoint by `export_policy_c.py` into `src/policy.c`/`policy.h`. Firmware comes with pre-trained weights, but they can be swapped without re-flashing (see `src/policy_mem.c`).
- **Observations:** — `crazyflie_ros`' controller node computes observations from external mocap and streams them via `Crazyflie::sendRaceObservation` (3 chunks/frame, 7 floats each).
- **Actuation** — `src/mixer.c` emulates the policy output → per-motor RPMs from  [`mjx-drone-trainer`](https://github.com/Jirl-upenn/mjx-drone-trainer).

## Expected Layout

Three sibling repos under one workspace directory. **Every path in this README is relative to this layout** — clone them side by side and the commands below work verbatim.

```
<workspace>/
├── crazyflie-firmware/                ← this repo
│   └── examples/app_race_policy/      ← you are here
├── crazyflie_ros/                     ← ROS 2 stack: mocap, radio driver, obs streaming
│   ├── bin/                           ← upload_policy_weights.py, set_ctrl_race_params.py
│   └── tools/crazyflie_cpp/           ← Crazyflie::sendRaceObservation
└── mjx-drone-trainer/                 ← training; also export_policy_c.py
    └── runs/<task>/<run-name>/        ← checkpoints (params.pkl + config.json)
```

- Relative hops worth memorizing: from this directory, the workspace root is `../../..`, so `crazyflie_ros` is `../../../crazyflie_ros` and the trainer is `../../../mjx-drone-trainer`.
- `export_policy_c.py` imports only `numpy` and `ml_dtypes` — no jax, no MuJoCo, no `dmcdrones`. You can export a checkpoint on the flight laptop without the training environment installed.

## Control Modes

Since take-off, hover, and landing have a large sim-to-real gap, we maintain the original PID controller for these actions and only switch to NN command after stable hover is achieved. We also use the PID to prevent crashes due to stale / lost observation packets.
| `mode` | Name | When | Drives motors via |
|---|---|---|---|
| 0 | PID | takeoff, hover, land, pre-race idle, stale/lost observations | `controllerPid()` |
| 1 | policy | observations arriving *and* `obsChanEnable=1` *and* weights valid | `policyForward()` → mixer |
| 2 | bench | `benchEnable=1` (overrides everything) | `ctrlRace.action0..3` → mixer |

## Build & Flash

Follow the [build docs](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) out of this directory rather than the project root:

```bash
export URI=radio://0/80/2M/E7E7E701B1     # your vehicle

cd <workspace>/crazyflie-firmware/examples/app_race_policy
make -j$(nproc)
make cload CLOAD_CMDS="-w $URI"
```
Notes:
- First build generates `build/.config` from `alldefconfig` merged with `app-config`; subsequent builds reuse it. `make clean` to reset.
- `make cload` warm-reboots the vehicle into its bootloader, flashes, and reboots. No physical button press needed.
- Config changes (`app-config`, Kconfig) need a `make clean` to take effect reliably.
- The compiled-in checkpoint is active immediately after flashing — a fresh flash always has a working policy.

## Flying It

Flashing is where this repo's involvement ends. Everything after it — weight upload, arming onboard mode, ROS bringup, and the takeoff/race/land sequence — belongs to the ground station:

**→ [`crazyflie_ros/README.md`](../../../crazyflie_ros/README.md) § Onboard Policy Flight Sequence**

Come back here for the firmware-side reference below: what the `ctrlRace` params and logs mean, and what to check when a flight goes wrong.

## Params (`ctrlRace`)

Set with `set_ctrl_race_params.py --set NAME=VALUE`. All are runtime-writable — no reflash needed.

- **`obsChanEnable`** (u8, default 0) — operator intent to arm the policy. Necessary, not sufficient.
- **`obsStaleTicks`** (u16, default 50) — ms without a complete observation frame before falling back to PID. Stabilizer ticks are 1 kHz, so this is milliseconds directly. Too tight → mode flaps mid-lap; too loose → a dropped link coasts further before recovering.
- **`benchEnable`** (u8, default 0) — force bench mode: mixer driven by `action0..3` params, PID never runs. **Never set on a vehicle expected to fly under PID.**
- **`hoverRpm` / `maxRpm` / `kf` / `differentialFrac`** — per-vehicle dynamics. Defaults are `mjx-drone-trainer` sim nominals (mass 0.027 kg, `max_rpm` 21714, `kf` 3.16e-10, `differential_frac` 0.02), *not* calibrated values.
- **`action0..3`** — bench-test action stand-in, only read in `benchEnable=1`. Zeroed on init.

## Logs (`ctrlRace`)

- **`mode`** (u8) — 0=PID, 1=policy, 2=bench. **The ground truth for which controller produced the motor commands.** Pull this first when diagnosing a flight.
- **`weightsValid`** (u8) — currently-loaded weights passed CRC + architecture validation.
- **`obsFrames`** (u32) — cumulative complete observation frames reassembled. Flat during a race means the uplink, not the policy, is the problem.
- **`rpm0..3`**, **`nf0..3`** — mixer output and the normalized per-motor forces handed to `powerDistributionForce()`.

## Troubleshooting

- **Tips over on takeoff** — check `ctrlRace.mode` during takeoff. It must read 0. If it reads 1, observations are arriving outside the racing state; if 2, `benchEnable` is set.
- **Policy never engages on `race`** — `obsFrames` flat → uplink problem (radio, `crazyradio_driver`, namespace). `obsFrames` climbing but `mode` still 0 → `obsChanEnable=0` or `weightsValid=0`.
- **`mode` oscillates 0↔1 mid-lap** — mocap/radio jitter exceeds `obsStaleTicks`. Raise it; no reflash needed.
- **`weightsValid=0` after an upload** — CRC or architecture mismatch. The previous weights are untouched; re-export against the firmware you actually flashed.
- **Sluggish or jittery only while racing** — the fp16 forward pass is software-decoded per multiply-accumulate and can approach the 1 ms stabilizer tick. Racing-phase-only symptoms point here, not at the mixer.

## Layout

- **`src/race_controller.c`** — `controllerOutOfTree()`, mode arbitration, `ctrlRace` param/log groups.
- **`src/policy.c`/`.h`** — **generated**, do not hand-edit. Re-run `export_policy_c.py`.
- **`src/mixer.c`/`.h`** — action → per-motor RPM. Must stay a byte-exact port of the sim's mixer.
- **`src/obs_channel.c`** — app-channel chunk reassembly. Must agree with `crazyflie_cpp`'s `sendRaceObservation`.
- **`src/policy_mem.c`** — `MEM_TYPE_APP` glue for runtime weight upload.
- **`src/test_mixer_host.c`, `src/test_policy_host.c`** — host-side numerical checks, built with plain gcc.
- **`policy_export_rpm/`** — an rpm-`action_type` checkpoint's export, kept for reference. Not built.
