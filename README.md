# Go2W Low-Level Gestures

Containerized, fail-closed low-level gesture controller for a Unitree Go2W.
The controller talks directly to Unitree SDK2Py over CycloneDDS. It does not
use `rclpy`, Unitree ROS 2 messages, ROS 2 Foxy, or ROS 2 Humble at runtime.

The repository provides two selectable gestures and two explicit hardware
timing scripts. Both scripts import the same fail-closed controller logic; only
the repeated gesture transition/hold timing differs.

| Gesture | Low-level sequence |
| --- | --- |
| `height` | Standard, three low/high cycles, standard |
| `roll` | Standard, three right/left cycles, standard |

| Script | Timing profile | Gesture transition | Gesture hold |
| --- | --- | --- | --- |
| `go2w_gesture_real.py` | `slow` | 2.0 s | 2.0 s |
| `go2w_gesture_real_fast.py` | `fast` | 1.0 s | 0.5 s |

Every live gesture shares the same control-ownership checks, watchdogs,
captured-prone shutdown, explicit confirmation boundary, and conditional Sport
Mode restoration.

## Clone, build, and inspect

```bash
git clone https://github.com/koki67/go2w-lowlevel-gestures.git
cd go2w-lowlevel-gestures
make build
make test
make describe
```

`make build`, `make test`, and `make describe` do not connect to the robot.
`make describe` prints both timing profiles.

## Deployment assumptions

- Host: the Go2W onboard Jetson.
- Jetson robot-facing NIC: `eth0`.
- Required IPv4 address on `eth0`: `192.168.123.18`.
- Docker Engine with the Compose plugin.
- The container uses host networking so CycloneDDS sees the Jetson's `eth0`.
- Robot starts belly-down and motionless on a flat floor.
- Wheels are physically blocked; a support/spotter and hardware E-stop are
  immediately available.
- No other user low-level controller is running.
- The repository-local `runs/` directory is writable. Compose mounts it at
  `/app/runs` even though the rest of the container filesystem is read-only.

The image is based on Ubuntu 22.04. ROS 2 is deliberately omitted because this
controller uses SDK2Py directly. It can run alongside other Humble containers,
but sourcing a ROS environment is not required.

## Gesture definitions

The profile timing applies to the repeated low/high or right/left motions. Both
scripts retain the safer common startup and shutdown timing: captured prone to
standard is 2.0 s with a 2.0 s hold, standard recovery is 2.0 s with a 2.0 s
hold, and return to captured prone is 3.0 s with a 2.0 s hold.

### Height

- Captured prone to standard: 2.0 s; hold standard: 2.0 s.
- Three low/high cycles using the selected profile timing: slow `2.0/2.0 s` or
  fast `1.0/0.5 s` transition/hold.
- High to standard: 2.0 s; hold standard: 2.0 s.
- Standard to captured prone: 3.0 s; hold prone: 2.0 s.
- Zero-gain neutral command: 1.0 s, close LowCmd, then restore the Sport service
  captured at startup when one was active.

### Roll

The Go2W URDF hip-abduction range is `[-1.0472, 1.0472] rad`. Starting from
the standard hip targets, the largest symmetric common offset is `0.9472 rad`.
The roll gesture deliberately uses 70% of that value: `0.66304 rad` (about
38.0 degrees of joint offset).

- Captured prone to standard: 2.0 s; hold standard: 2.0 s.
- Three right/left cycles using the selected profile timing: slow `2.0/2.0 s`
  or fast `1.0/0.5 s` transition/hold.
- Left to standard: 2.0 s; hold standard: 2.0 s.
- Standard to captured prone: 3.0 s; hold prone: 2.0 s.
- Zero-gain neutral command: 1.0 s, close LowCmd, then restore the Sport service
  captured at startup when one was active.

The MuJoCo run reached approximately `+27.7/-27.6 degrees` of measured body
roll without falling and returned close to level. This is simulation evidence,
not Go2W hardware qualification.

## Read-only robot preflight

First verify the host NIC:

```bash
ip -4 addr show dev eth0
```

Then select the timing profile and gesture to inspect. Slow profile:

```bash
make preflight-slow-height
make preflight-slow-roll
```

Fast profile:

```bash
make preflight-fast-height
make preflight-fast-roll
```

The existing `make preflight-height` and `make preflight-roll` names remain
aliases for the slow profile.

Preflight initializes DDS, reads `rt/lowstate`, and calls read-only
`CheckMode()`. It verifies the expected NIC/IP, stable prone pose, joint and
wheel velocities, IMU tilt, current Sport-service state, and selected gesture
targets. An empty service name is reported as an already-released state rather
than rejected.
It does **not** call `StopMove()`, `ReleaseMode()`, `SelectMode()`, or publish
`LowCmd`.

## Live hardware execution

Do not continue unless the selected preflight succeeds and the physical safety
setup is ready.

Slow height and roll gestures:

```bash
make live-slow-height
make live-slow-roll
```

Fast height and roll gestures:

```bash
make live-fast-height
make live-fast-roll
```

The required typed confirmation depends on the gesture, not the timing
profile. Height:

```text
RUN GO2W LOW LEVEL
```

Roll:

```text
RUN GO2W ROLL LOW LEVEL
```

The live ownership sequence is common to both gestures:

1. Confirm a stable, belly-down measured pose.
2. Require the gesture-specific interactive phrase.
3. If `CheckMode()` reported an active Sport service, save its exact name, send
   Sport `StopMove()`, and repeatedly call `ReleaseMode()` plus `CheckMode()`
   until it is inactive. Any nonzero release result aborts before LowCmd starts.
4. If Sport was already released, skip `StopMove()`/`ReleaseMode()` and retain
   the released state.
5. Re-measure the prone shutdown target.
6. Require `rt/lowcmd` to become quiet before creating the sole user publisher.
7. Run the selected gesture at a nominal 500 Hz while buffering measured and
   target joint tracking telemetry in memory.
8. Return to the hardware-measured initial prone pose and hold it.
9. Publish a zero-gain neutral command briefly, stop publishing, and explicitly
   close the LowCmd DDS writer.
10. After another quiet-topic interval, call `SelectMode()` with the saved
    startup service name and require `CheckMode()` to confirm it.

There is intentionally no ambiguous `make live` target. The gesture name must
be part of the command and is shown again before confirmation.

## Sport Mode restoration boundary

Automatic restoration is attempted only after the controller confirms that it
has returned to the captured prone pose. It closes its LowCmd DDS writer first,
waits for `rt/lowcmd` to stay quiet, calls `SelectMode()` with the exact service
name captured at startup (normally `ai-w` on Go2W), and polls `CheckMode()` until
that same name is active. Failure to close the writer, a nonzero `SelectMode()`
result, an unexpected active service, or a confirmation timeout makes the
script exit nonzero; it does not claim that Sport stabilization is active.

If the script starts with no active service, it may proceed only after the
stable-prone and quiet-`rt/lowcmd` checks. Because there is no startup service
name to restore, successful completion leaves Sport released. This permits a
new invocation to recover after an earlier low-level run without guessing a
firmware-specific mode name.

A watchdog, hard stop, failed controlled return, process kill, host failure, or
network loss does not trigger automatic Sport restoration. In those cases the
posture is not confirmed safe for the ownership handoff. Keep the robot
supported and do not assume either software fallback succeeded. The restoration
sequence is unit-tested but remains unqualified on physical Go2W hardware.

## Runtime watchdogs

The live controller fails closed on:

- stale or non-finite LowState;
- body roll/pitch above `0.55 rad` (about 31.5 degrees);
- a warning when any of the 12 position-controlled leg-joint tracking errors
  exceeds `0.45 rad`, followed by an immediate stop above `0.55 rad`;
- DDS write failure;
- failure to release Sport Mode;
- failure to close LowCmd or restore and confirm the captured Sport service;
- LowCmd traffic that does not become quiet after release;
- NIC or IP mismatch;
- unstable or implausible initial prone pose;
- an unknown or omitted gesture.

Both joint-tracking thresholds are provisional heuristics. The original
`0.45 rad` stop threshold was not derived from measured Go2W tracking-error
distributions, actuator/torque limits, or a physically qualified safety test.
It is now retained as a throttled terminal warning and telemetry marker. The
stop threshold is relaxed only to `0.55 rad`, giving a narrow diagnostic range
without removing the independent application-level stop. Do not treat either
number as a certified limit. Raising or removing the stop allows larger
commanded-versus-measured errors, larger potential PD effort and contact loads,
and a longer loss-of-tracking interval before LowCmd is neutralized.

The pinned Unitree SDK2Py API accepts and publishes the requested `q`, `dq`,
`kp`, `kd`, and `tau` fields. Unitree's public SDK examples do not document an
SDK-side joint-error rejector or a guaranteed embedded fallback that makes an
application watchdog unnecessary. Motor firmware may apply undisclosed current
or torque saturation, but this controller does not rely on that as proof that a
large or sustained position error is safe.

The first Ctrl+C requests a controlled return to the captured prone pose. A
second Ctrl+C abandons that return and sends a short neutral command. A process
kill, host failure, container failure, or network loss can still prevent any
software fallback. Physical support and a hardware E-stop remain mandatory.

On a tracking-watchdog stop, the terminal reports every leg joint above the
limit with its motor/q index, name, measured angle, commanded angle, signed
`commanded-measured` error, and absolute error. Wheel motors 12--15 are not part
of this position-tracking check because they receive velocity commands rather
than position targets.

## Live joint-tracking logs

Every confirmed `--live` invocation prepares a log destination before it sends
`StopMove()` or releases Sport Mode. During LowCmd motion, the 500 Hz loop only
appends compact numeric samples to memory; it does not synchronously write to
disk. On normal completion, a controlled interrupt, a hard stop, or an
application exception such as the tracking watchdog, `main()` writes two files
under the host repository's `runs/` directory:

- `*_tracking.csv`: one row per runtime check, including the row that crossed
  the stop threshold;
- `*_tracking.summary.json`: outcome/error text, thresholds, sample counts,
  samples per phase, the global peak, and each joint's peak error.

Each CSV row contains the run and phase time, phase name, LowState sample age,
IMU roll/pitch/yaw, and, for every joint from `FR_hip` through `RL_calf`, the
measured angle, the last active target angle, and `target - measured` error. It
also identifies the maximum-error joint for that row. LowState age helps
separate true tracking lag from delayed state delivery, while differences in
`run_elapsed_s` expose loop-timing gaps. Logs are not created by `--describe`
or the read-only preflight.

List the newest files on the Jetson after a run:

```bash
ls -lt runs | head
```

For a direct non-Compose invocation, the default is `./runs`; override it with
`--tracking-log-dir PATH` or `GO2W_TRACKING_LOG_DIR=PATH`. A process kill, power
loss, or host failure can prevent the in-memory buffer from being flushed, just
as it can prevent neutralization and Sport restoration.

## Pinned dependencies

The Docker build downloads and verifies exact upstream commits:

| Component | Pin |
| --- | --- |
| Ubuntu | `22.04` |
| Eclipse CycloneDDS C library | `1be07de395e4ddf969db2b90328cdf4fb73e9a64` (`0.10.4`) |
| CycloneDDS Python binding | `0.10.2` |
| Unitree SDK2Py | `a035adeaa6f8ea171bef9a43e8477abb87a0b35e` |
| NumPy | `1.26.4` |
| OpenCV Python | `4.10.0.84` |

The Unitree checkout contains CRC libraries for both `x86_64` and `aarch64`.
Build the image natively on the Jetson for the first hardware trial.

The image imports SDK2Py directly from that verified checkout. This avoids an
upstream packaging issue in which `setup.py` omits the namespace-style `b2`
hierarchy; the checkout itself is not patched.

## Qualification status

- Non-hardware unit and command-generation tests: included in every image build.
- Height MuJoCo motion: validated separately.
- Roll MuJoCo motion: validated at 70% URDF-derived hip offset and 0.75 s
  transitions.
- Jetson `aarch64` image build and live startup: exercised; actual 500 Hz loop
  rate and jitter remain unmeasured.
- Go2W height hardware motion: both fast `1.0/0.5 s` and slow `2.0/2.0 s`
  live attempts stopped on the former provisional `0.45 rad` stop threshold.
  The new `0.45 rad` warning / `0.55 rad` stop policy and live telemetry remain
  unqualified; the full sequence remains unqualified.
- Go2W roll hardware motion: not yet performed.
- Automatic Sport Mode restoration after a confirmed prone return: implemented
  and unit-tested, but not yet physically qualified on Go2W hardware.

Do not interpret a successful build, dry-run, or simulation as physical
qualification.
