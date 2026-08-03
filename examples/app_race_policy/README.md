# app_race_policy

Onboard neural-network control for racing. The workstation streams observations over the app-channel; the Crazyflie runs the policy forward pass itself and drives per-motor RPM directly, bypassing the stock attitude-rate PID.

- **Policy** — actor-only MLP, fp16 weights, exported from an `mjc_dronetests` checkpoint by `export_policy_c.py` into `src/policy.c`/`policy.h`.
- **Observations** — `crazyflie_ros`' controller node computes v3 observations from mocap and streams them via `Crazyflie::sendRaceObservation` (3 chunks/frame, 7 floats each).
- **Weights** — compiled into flash, and swappable at runtime over CRTP `MEM_TYPE_APP` without reflashing (see `src/policy_mem.c`).
- **Actuation** — `src/mixer.c` ports the sim's `mix_attitude_rpm` exactly, then `kf·rpm²` → `controlModeForce`. Open loop, no gyro feedback anywhere — deliberate, since that is what every checkpoint was trained against.

## Control Modes

`CONFIG_CONTROLLER_OOT=y` makes `controllerOutOfTree()` the **sole** controller for the entire flight — PID is never reached on its own. It therefore arbitrates per phase, reported live as `ctrlRace.mode`:

| `mode` | Name | When | Drives motors via |
|---|---|---|---|
| 0 | PID | takeoff, hover, land, pre-race idle, stale/lost observations | `controllerPid()` |
| 1 | policy | observations arriving *and* `obsChanEnable=1` *and* weights valid | `policyForward()` → mixer |
| 2 | bench | `benchEnable=1` (overrides everything) | `ctrlRace.action0..3` → mixer |

- **Observation freshness is the phase gate**, not `obsChanEnable`. The ROS controller publishes `/race_obs` only in its `racing` FSM state; every other phase sends ordinary CTBR setpoints and no observations. `obsChanEnable` is set before takeoff and cannot distinguish phases on its own.
- Same mechanism is the **link-loss failsafe** — the stream stops, `obsStaleTicks` elapses, control reverts to PID rather than holding the last policy action.
- Mode is evaluated every tick; entry and exit are both automatic.

## Build & Flash

Requires `arm-none-eabi-gcc` (or the Bitcraze toolbelt / docker image). Built out of this directory, not the repo root — see the [build docs](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/).

```bash
export URI=radio://0/80/2M/E7E7E701B1     # your vehicle

cd ~/visual_prelims/crazyflie-firmware/examples/app_race_policy
make -j$(nproc)
make cload CLOAD_CMDS="-w $URI"
```

- First build generates `build/.config` from `alldefconfig` merged with `app-config`; subsequent builds reuse it. `make clean` to reset.
- `make cload` warm-reboots the vehicle into its bootloader, flashes, and reboots. No physical button press needed.
- Config changes (`app-config`, Kconfig) need a `make clean` to take effect reliably.
- The compiled-in checkpoint is active immediately after flashing — a fresh flash always has a working policy.

## Flight Sequence

### 1. Build & flash

Per above. Everything below assumes `$URI` is exported.

### 2. Weight upload — *skip unless swapping checkpoints*

Only needed to run a **different** checkpoint than the one compiled into the flash you just did. A fresh flash's compiled-in checkpoint is already active. If skipping, go straight to step 3.

```bash
cd ~/visual_prelims/crazyflie_ros
python3 ../mjc_dronetests/export_policy_c.py \
  --run-dir /path/to/runs/race/sparse_attitude_seed0 \
  --out-dir /tmp/policy_export --weights-out /tmp/policy_export/policy_weights.bin
python3 bin/upload_policy_weights.py --uri $URI /tmp/policy_export/policy_weights.bin
```

- Vehicle must be **disarmed** — do this before step 3, not after.
- Uploads are refused outright while armed; a bad architecture or failed CRC is rejected whole, never partially applied.
- Weights live in RAM: **a power cycle reverts to the compiled-in checkpoint.** Re-upload after every reboot.

### 3. Enable onboard mode (while disarmed)

```bash
python3 bin/set_ctrl_race_params.py --uri $URI --enable-onboard --read
```

- Sets `ctrlRace.obsChanEnable=1`. This is the operator's *intent* to arm the policy — it does not by itself hand over control (see **Control Modes**).
- `--read` lists the **param** group only. `weightsValid` and `mode` are **log** variables and will not appear there — inspect them in cfclient's log tab, or with a cflib log block.
- If `weightsValid=0` on a fresh flash, the flashed binary itself is suspect (not a step-2 issue, if you skipped it) — recheck the build before flying.

### 4. Bringup (3 terminals)

```bash
ros2 launch jirl_bringup vicon.launch.py
ros2 launch jirl_bringup crazyradio_driver.launch.py
ros2 launch jirl_bringup controller.launch.py \
  namespace:=crazy_jirl_b5 \
  onboard_policy_enable:=true \
  onboard_policy_checkpoint_path:=../sparse_attitude_seed0
```

- `onboard_policy_checkpoint_path` is **ROS-side bookkeeping/logging only** — this launch does not push it to the vehicle.
- Make sure it matches what is actually running onboard (the flashed checkpoint, or step 2's upload). Nothing cross-checks this automatically.

### 5. Fly

```bash
ros2 service call /arm jirl_interfaces/srv/Arm "{crazyflie_name: 'crazy_jirl_b5', command: 0}"
ros2 service call /crazy_jirl_b5/takeoff std_srvs/srv/Trigger
ros2 service call /crazy_jirl_b5/race    std_srvs/srv/Trigger
ros2 service call /crazy_jirl_b5/land    std_srvs/srv/Trigger
```

- `takeoff`/`land` fly under **PID** (`mode=0`). Only `race` starts the observation stream that promotes the vehicle to `mode=1`.
- Expect `ctrlRace.mode` to read 0 → 1 on `race`, and 1 → 0 on `land`.

## Params (`ctrlRace`)

Set with `set_ctrl_race_params.py --set NAME=VALUE`. All are runtime-writable — no reflash needed.

- **`obsChanEnable`** (u8, default 0) — operator intent to arm the policy. Necessary, not sufficient.
- **`obsStaleTicks`** (u16, default 50) — ms without a complete observation frame before falling back to PID. Stabilizer ticks are 1 kHz, so this is milliseconds directly. Too tight → mode flaps mid-lap; too loose → a dropped link coasts further before recovering.
- **`benchEnable`** (u8, default 0) — force bench mode: mixer driven by `action0..3` params, PID never runs. **Never set on a vehicle expected to fly under PID.**
- **`hoverRpm` / `maxRpm` / `kf` / `differentialFrac`** — per-vehicle dynamics. Defaults are `mjc_dronetests` sim nominals (mass 0.027 kg, `max_rpm` 21714, `kf` 3.16e-10, `differential_frac` 0.02), *not* calibrated values.
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
