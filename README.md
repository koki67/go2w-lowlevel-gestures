# Go2W Low-Level Gestures

Containerized, fail-closed low-level gesture controller for a Unitree Go2W.
The controller talks directly to Unitree SDK2Py over CycloneDDS. It does not
use `rclpy`, Unitree ROS 2 messages, ROS 2 Foxy, or ROS 2 Humble at runtime.

The repository owns both sides of the gesture definition: the four original
hardware profiles, two closed-loop controllers, and MuJoCo controllers for the
same height and roll targets. The MuJoCo engine and Go2W model remain an
external dependency; they are not vendored into this repository.

| Gesture | Low-level sequence |
| --- | --- |
| `height` | Standard, three low/high cycles, standard |
| `roll` | Standard, three right/left cycles, standard |

| Script | Timing profile | Gesture transition | Gesture hold | Joint tracking-error stop |
| --- | --- | --- | --- | --- |
| `go2w_gesture_real.py` | `slow` | 2.0 s | 2.0 s | Above 0.55 rad |
| `go2w_gesture_real_fast.py` | `fast` | 1.0 s | 0.5 s | Above 0.55 rad |
| `go2w_gesture_real_no_tracking_stop.py` | `slow` | 2.0 s | 2.0 s | Disabled |
| `go2w_gesture_real_fast_no_tracking_stop.py` | `fast` | 1.0 s | 0.5 s | Disabled |
| `go2w_gesture_real_adaptive.py` | adaptive joint-space | 1.0 s nominal | 0.5 s minimum | Adaptive envelopes plus 0.55 rad stop |
| `go2w_gesture_real_wbc.py` | quasi-static kinematic WBC | 1.0 s nominal | 0.5 s minimum | Adaptive envelopes plus 0.55 rad stop |

Every live gesture shares the same control-ownership checks, captured-prone
shutdown, and conditional Sport Mode restoration. The fast no-tracking-stop
profile starts automatically after those live prechecks; the other profiles
retain the explicit confirmation boundary. The diagnostic variant disables
only the joint tracking-error stop; LowState freshness, finite-state,
body-tilt, DDS-write, and ownership watchdogs remain active.

## Clone, build, and inspect

```bash
git clone https://github.com/koki67/go2w-lowlevel-gestures.git
cd go2w-lowlevel-gestures
make build
make test
make describe
```

`make build`, `make test`, and `make describe` do not connect to the robot.
`make describe` preserves the original timing/watchdog descriptions. Use the
explicit `describe-adaptive-*` and `describe-wbc-*` targets for the new
closed-loop controllers.

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

The profile timing applies to the repeated low/high or right/left motions. All
four profiles retain the safer common startup and shutdown timing: captured
prone to standard is 2.0 s with a 2.0 s hold, standard recovery is 2.0 s with a
2.0 s hold, and return to captured prone is 3.0 s with a 2.0 s hold.

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

## Closed-loop controllers

`go2w_gesture_real_adaptive.py` keeps the existing joint targets and 500 Hz
position-PD command path, but replaces time-only phase progression with a
reference governor. Command-versus-measurement envelopes are `0.18 rad` for
hip, `0.14 rad` for thigh, and `0.25 rad` for calf joints. Progress runs at the
scheduled speed through 50% of an envelope, slows linearly from 50% to 90%,
and stops at 90%. A 100% breach sustained for 0.10 s requests a controlled
captured-prone return. A phase completes only after all joints are within the
50% convergence gate and maximum joint speed is at most `0.20 rad/s` for
0.30 s. Startup, height, STANDARD, and prone-return phases keep that gate.

The right/left adaptive roll endpoints use an additional load-aware static-PD
equilibrium gate because a body roll needs nonzero `q_ref - q` to generate its
holding torque. It permits at most 70% of each tracking envelope, requires
`tau_est` to agree with `Kp*(q_ref-q) - Kd*dq` within 10% of the corresponding
joint envelope, requires `max |dq| <= 0.02 rad/s` and torque below the 60%
warning threshold, and checks that IMU roll moved at least half of
`0.395469 rad` in the direction produced by the existing adaptive joint
target. Every condition must remain true for 0.30 s after trajectory progress
reaches 100%. The 90% progress stop, full tracking envelope, `0.55 rad` tilt
watchdog, torque return/error thresholds, and controlled-return behavior are
unchanged. These roll-gate thresholds are provisional MuJoCo-backed
application values, not physical qualification. Wall timeouts remain 8 s for
a repeated transition, 12 s for startup, and 15 s for the prone return.

`go2w_gesture_real_wbc.py` uses the same adaptive startup and return. During
the gesture it solves a 100 Hz constrained kinematic QP for
`[base twist 6, leg dq 12]`, integrates a bounded position reference, and
resends that reference through the existing 500 Hz position PD. It keeps every
LowCmd `tau` field at zero. This is a quasi-static kinematic WBC, not a
direct-torque dynamic WBC, and it is not intended for jumps, fast contact
switches, or dynamic gaits.

The WBC target offsets relative to the captured STANDARD state are:

| Gesture endpoint | Task-space target |
| --- | --- |
| Height low | `-0.093178 m` |
| Height high | `+0.076281 m` |
| Roll right | `-0.35 rad` (about `-20.054 degrees`) |
| Roll left | `+0.35 rad` (about `+20.054 degrees`) |

The earlier WBC roll target of `+/-0.395469 rad` transiently lifted an inside
wheel in the flat-floor MuJoCo qualification. The WBC-only target is therefore
limited to `+/-0.35 rad`; the existing scripted joint-space roll poses are not
changed. Roll WBC uses STANDARD as its secondary posture objective so the task
QP, rather than the scripted roll pose, determines the load-bearing joint
configuration.

Roll and pitch come from the IMU. Relative height comes from leg kinematics
and estimated wheel loads; it is not claimed as absolute world height. Contact
loads are estimated from the audited model, actuator-sign `tau_est`, external
gravity, and `J(q)^T f`. The forces are accepted only when the QP status,
Jacobian conditioning, torque fit, force/moment balance, total vertical load,
and every positive wheel load pass continuously for 0.5 s. Go2W `foot_force`
is not used as a valid sensor input. On hardware, the WBC also holds STANDARD
at 500 Hz while a separate input thread requires:

```text
FOUR WHEELS LOADED AND BELLY CLEAR
```

Both closed-loop controllers apply provisional application protections at 60%
(warning), 75% (stop progress), 85% for 0.10 s (controlled return), and 100%
(immediate error) of the audited model torque ranges. These thresholds are not
actuator certification values. They also stop on stale/non-finite state, mode
changes, increases in `lost`, tilt, DDS failures, solver failure, and timing
failure. Temperature and power are recorded but have no invented absolute stop
threshold. Neither controller retries automatically or falls back to a
no-tracking-stop script.

The kinematic WBC keeps its 100 Hz QP period and the 10 ms fail-closed runtime
limit. Its fixed-size OSQP workspace and initial factorization are created
during preflight, then updated and warm-started instead of being rebuilt every
cycle.
Estimated minimum wheel load additionally slows progress below 10% of body
weight, backs away from the endpoint at or below 6%, requests a controlled
return below 4% for 0.10 s, and must be at least 8% for roll completion. These
are model-derived operational margins, not measured wheel loads or physical
qualification. After the gesture, WBC explicitly returns to and settles the
STANDARD task before the existing adaptive prone return.

The first physical trial is intentionally configured for the full task target
and all three cycles, following the selected test protocol. That is riskier
than an amplitude ramp and does not weaken any watchdog or confirmation gate.

## MuJoCo simulation

The simulation controller code and flat-scene definition live in this
repository under `simulation/`. A built `unitree_mujoco` checkout supplies the
simulator executable, Go2W MJCF/assets, and a Python environment containing
`unitree_sdk2py`. This keeps the gesture implementation beside the hardware
controller without copying or forking the upstream simulator.

By default, Make expects the repositories to be siblings:

```text
~/ws/go2w-lowlevel-gestures
~/ws/unitree_mujoco
```

The external checkout must provide:

- `simulate/build/unitree_mujoco`;
- `unitree_robots/go2w/go2w.xml` and its `assets/` directory; and
- `simulate_python/.venv/bin/python` with `unitree_sdk2py` importable.

Check those requirements without opening MuJoCo:

```bash
make sim-doctor
make sim-describe
```

For a checkout elsewhere, override the root. The Python executable may also be
overridden independently:

```bash
make sim-doctor UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco
make sim-doctor \
  UNITREE_MUJOCO_ROOT=/path/to/unitree_mujoco \
  UNITREE_MUJOCO_PYTHON=/path/to/python
```

The closed-loop qualification harness uses pinned NumPy/SciPy/OSQP in an
isolated `/tmp` target so it does not modify the external simulator checkout or
its virtual environment. Prepare it once, inspect the contract, and run the
four controller/gesture combinations:

```bash
make sim-closed-loop-deps
make sim-closed-loop-doctor
make sim-closed-loop-describe
make sim-adaptive-height
make sim-adaptive-roll
make sim-wbc-height
make sim-wbc-roll
```

Each target runs three cycles from normal, asymmetric-prone, and
belly-loaded-prone preparation states. The controller receives only joint
position/velocity, actuator torque, and IMU-equivalent inputs. Simulator base
pose, contacts, and actuator force are evaluation-only. JSON summaries are
written under `runs/mujoco/closed-loop/`; a nonzero result is retained as
qualification evidence rather than bypassed. These headless results are
simulation qualification only.

Closed-loop runs do not buffer or write plot data by default. Adaptive runs can
save two human-readable SVGs by adding the opt-in `save-plot` goal:

```bash
make sim-adaptive-height save-plot
make sim-adaptive-roll save-plot
```

The Make goal forwards the simulator's `--save-plot` CLI option. Equivalently,
use `make sim-adaptive-height SIM_ARGS=--save-plot` when composing additional
simulator arguments.

For each initial condition, one SVG contains the 12 adaptive joint references
and measured joint angles. The second contains phase-local nominal versus
adaptive trajectory progress, governor speed scale, normalized tracking error,
and normalized `tau_est`, including their slowdown, stop, return, and error
thresholds. Phase bands use the same time axis in both files. A failed or timed
out case still writes the samples captured before and during its adaptive
return. The summary JSON records both artifact paths and whether plot
generation succeeded.

WBC runs use the same opt-in goal and save five controller-specific SVGs:

```bash
make sim-wbc-height save-plot
make sim-wbc-roll save-plot
```

The files show:

1. all 12 WBC position-PD references against measured joint angles;
2. relative height, roll, pitch, and yaw targets against controller estimates,
   including the height-gesture `+/-0.015 m`, roll-gesture `+/-0.020 m`, and
   roll/pitch `+/-2 deg` endpoint bands;
3. estimated normal load at each wheel, total and left/right support load,
   lateral center of pressure, and the 10%/6%/4% controller support margins;
4. contact-force QP validity, torque and force/moment residuals, Jacobian
   condition number, solve time, and iterations; and
5. WBC QP solve time/residuals, wheel-velocity residual, velocity/acceleration
   bound utilization, joint tracking, `tau_est`, and body-tilt safety margins.

The support plots are based on `tau_est` and the modeled Jacobians; they are not
direct foot-force measurements. Height is relative to the validated STANDARD
pose, not an absolute world height. Failed contact validation, QP rejection, or
a later controlled return remains visible in the saved samples. SVG files are
written only after the controller case has stopped. Non-event WBC plot samples
are decimated to the 100 Hz QP rate so GUI rendering plus `save-plot` does not
buffer redundant 500 Hz points; live 500 Hz deadline-miss
evidence remains in the hardware CSV/summary rather than being synthesized by
MuJoCo. These plots are simulation diagnostics, not physical qualification.

To inspect the exact same adaptive/WBC controller path in MuJoCo's GUI, run one
initial condition at a time:

```bash
make sim-view-adaptive-height
make sim-view-adaptive-roll
make sim-view-wbc-height
make sim-view-wbc-roll
```

The corresponding adaptive or WBC plots can be saved after closing the GUI
viewer:

```bash
make sim-view-adaptive-height save-plot
make sim-view-adaptive-roll save-plot SIM_INITIAL=normal
make sim-view-wbc-height save-plot
make sim-view-wbc-roll save-plot SIM_INITIAL=normal
```

The default is `normal` at real-time speed. Select either prepared failure case
or slow the display without changing simulated dynamics:

```bash
make sim-view-wbc-roll SIM_INITIAL=asymmetric-prone
make sim-view-wbc-roll SIM_INITIAL=belly-loaded-prone VIEWER_SPEED=0.5
```

The terminal prints each controller phase while the passive viewer displays the
same `MjModel`, `MjData`, 2 ms stepping, sensor inputs, and controller kernel as
the headless harness. At completion or a watchdog failure, the final state stays
visible until the MuJoCo window is closed; the summary is then written with
`execution_mode: viewer-inspection`. Closing the window during motion aborts
that case. Viewer runs are visual diagnostics; use the headless targets above as
the timing and qualification authority.

Launch the flat-scene GUI and the selected low-level controller from this
repository:

```bash
make sim-height
make sim-roll
make sim-quick-stand
make sim-shake-off
```

`sim-quick-stand` follows the standard and low setup poses, then interpolates
from low to high in `0.1 s`. It is a simulation-only sequence.

`sim-shake-off` reuses the same 70%-of-limit roll targets as `sim-roll`, but
runs 8 right/left cycles with a `0.10 s` transition and `0.03 s` hold at each
side. It is an intentionally aggressive, simulation-only starting point for a
wet-dog-style shake and is not qualified for hardware.

These default commands do not record or save a joint-tracking graph. To record
the target and actual joint-angle history and save it as an SVG, add the common
`save-plot` goal to any of these scripted simulation runs:

```bash
make sim-height save-plot
make sim-roll save-plot
make sim-quick-stand save-plot
make sim-shake-off save-plot
```

Each command fixes DDS to domain `0` on loopback (`lo`), refuses to start if a
simulator or LowCmd publisher is already active there, starts and owns the
MuJoCo child process, and stops it on `Ctrl+C`. The repository-owned flat scene
is assembled in a temporary directory with links to the external Go2W model;
the external checkout is not modified at runtime. With `--save-plot`, the SVG
is written under `runs/mujoco/` after 3 seconds of the final hold. Closed-loop
adaptive SVGs are written under `runs/mujoco/closed-loop/` after the case ends,
including failure cases. These SVGs are ignored generated output. Without the
flag, tracking samples are not kept in memory and no graph is written.

The external checkout's `simulate/config.yaml` and
`simulate_python/config.py` do not need to be changed: robot, scene, DDS domain,
and interface are supplied explicitly by the launcher.

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

Slow diagnostic profile without a joint tracking-error stop:

```bash
make preflight-no-tracking-stop-height
make preflight-no-tracking-stop-roll
```

Fast diagnostic profile without a joint tracking-error stop:

```bash
make preflight-fast-no-tracking-stop-height
make preflight-fast-no-tracking-stop-roll
```

Closed-loop read-only preflights:

```bash
make preflight-adaptive-height
make preflight-adaptive-roll
make preflight-wbc-height
make preflight-wbc-roll
```

The existing `make preflight-height` and `make preflight-roll` names remain
aliases for the slow profile.

Preflight initializes DDS, reads `rt/lowstate`, and calls read-only
`CheckMode()`. It verifies the expected NIC/IP, stable prone pose, joint and
wheel velocities, IMU tilt, current Sport-service state, and selected gesture
targets. An empty service name is reported as an already-released state rather
than rejected.
It does **not** call `StopMove()`, `ReleaseMode()`, `SelectMode()`, or publish
`LowCmd`. In particular, a WBC preflight does not run its contact QP or request
the four-wheel confirmation because those occur only after controlled STANDARD
has been reached during a confirmed live run.

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

Slow height and roll gestures without a joint tracking-error stop:

```bash
make live-no-tracking-stop-height
make live-no-tracking-stop-roll
```

Fast height and roll gestures without a joint tracking-error stop:

```bash
make live-fast-no-tracking-stop-height
make live-fast-no-tracking-stop-roll
```

These two fast no-tracking-stop targets do not prompt for `RUN GO2W ...`.
After the NIC/IP, DDS, active-service, stable-prone, and runtime-state prechecks
pass, they proceed directly to the Sport/LowCmd ownership handoff and motion.
Keep the wheels blocked, support/spotter in place, and hardware E-stop ready
before running either `make` command.

Closed-loop live targets exist explicitly, but the first physical evaluation
must use the qualification runner in the next section:

```bash
make live-adaptive-height
make live-adaptive-roll
make live-wbc-height
make live-wbc-roll
```

All other live targets require a typed confirmation. The phrase depends on the
gesture. Height:

```text
RUN GO2W LOW LEVEL
```

Roll:

```text
RUN GO2W ROLL LOW LEVEL
```

The live ownership sequence is common to both gestures:

1. Confirm a stable, belly-down measured pose.
2. Require the gesture-specific interactive phrase, except for the two
   `live-fast-no-tracking-stop-*` targets, which start automatically.
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
be part of the command and is shown in the live precheck output.

## Jetson qualification runner

The four `qualify-live-*` targets fail closed on a dirty worktree, wrong branch
or SHA, non-`aarch64` host, NIC/IP mismatch, build failure, test failure,
`pip check` failure, describe failure, or read-only preflight failure. They do
not stash, reset, delete, retry, select another gesture, or invoke a
no-tracking-stop fallback.

After the desktop-qualified feature commit has been pushed, run these on the
Jetson in this exact order, replacing `<desktop-qualified-sha>` with the full
40-character SHA:

```bash
cd /home/unitree/go2w-lowlevel-gestures
make qualify-live-adaptive-height QUALIFIED_SHA=<desktop-qualified-sha>
make qualify-live-adaptive-roll QUALIFIED_SHA=<desktop-qualified-sha>
make qualify-live-wbc-height QUALIFIED_SHA=<desktop-qualified-sha>
make qualify-live-wbc-roll QUALIFIED_SHA=<desktop-qualified-sha>
```

Before each live child starts, the runner prints a Japanese physical checklist
and requires a controller/gesture-specific phrase. The underlying controller
then retains its existing ownership confirmation; WBC adds its second
four-wheel/belly-clear confirmation while continuing the 500 Hz STANDARD hold.
Stop the remaining cases after any firmware error, E-stop, watchdog,
controlled-return failure, or unconfirmed Sport restoration.

Artifacts are isolated under `runs/qualification/<timestamp>/`: Git SHA,
commands, stage exit codes, terminal log, controller CSV/summary files, runner
summary, and SHA-256 manifest. The runner leaves `physical_pass` false for
manual review; a process exit code alone cannot establish safe physical
behavior. Invoking the runner directly without `--live` performs the software
pipeline and never calls a live Make target.

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

The standard slow and fast controllers fail closed on:

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

`go2w_gesture_real_no_tracking_stop.py` and
`go2w_gesture_real_fast_no_tracking_stop.py` are the separately named slow and
fast diagnostic variants for runs where tracking-error telemetry must not
terminate the gesture. They still print throttled warnings above `0.45 rad`
and record all 12 leg joints, but never stop solely because the
target-minus-measured joint error is large. The independent `0.55 rad` body
roll/pitch watchdog remains enabled. Their live precheck explicitly reports
`tracking-error stop disabled`. The fast variant does not wait for a typed
confirmation after that precheck; the slow variant retains the
gesture-specific confirmation.

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
  the stop threshold for the standard scripts;
- `*_tracking.summary.json`: outcome/error text, whether the tracking stop was
  enabled, thresholds, sample counts, samples per phase, the global peak, and
  each joint's peak error.

Each CSV row contains the run and phase time, phase name, LowState sample age,
IMU roll/pitch/yaw, and, for every joint from `FR_hip` through `RL_calf`, the
measured angle, the last active target angle, and `target - measured` error. It
also identifies the maximum-error joint for that row. LowState age helps
separate true tracking lag from delayed state delivery, while differences in
`run_elapsed_s` expose loop-timing gaps. Logs are not created by `--describe`
or the read-only preflight.

Adaptive and WBC live runs also write a controller-labelled closed-loop CSV and
summary JSON. They add `dq`, 16-motor `tau_est`/mode/lost/temperature, IMU gyro
and acceleration, power, phase progress, speed scale, and publication deadline
misses. Adaptive roll rows also record every static-PD equilibrium subcondition,
including dwell, raw tracking ratio, PD residual, signed IMU roll, velocity,
and the joint/torque/direction gates; the summary contains their ranges and
provisional thresholds. WBC rows additionally include task target/estimate,
per-wheel estimated force, balance and torque residuals, solver
status/iterations/residuals, solve time, and contact-velocity residual. The
500 Hz controller only appends rows in memory; files are finalized after
command publication stops.

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
| SciPy | `1.13.1` |
| OSQP | `1.1.3` |
| OpenCV Python | `4.10.0.84` |

The Unitree checkout contains CRC libraries for both `x86_64` and `aarch64`.
Build the image natively on the Jetson for the first hardware trial.

The image imports SDK2Py directly from that verified checkout. This avoids an
upstream packaging issue in which `setup.py` omits the namespace-style `b2`
hierarchy; the checkout itself is not patched.

## Qualification status

- Non-hardware unit and command-generation tests: included in every image build.
- Height MuJoCo motion: simulation code and launcher included; motion validated
  with the external `unitree_mujoco` runtime.
- Roll MuJoCo motion: simulation code and launcher included; validated at 70%
  URDF-derived hip offset and 0.75 s transitions.
- Jetson `aarch64` image build and live startup: exercised; actual 500 Hz loop
  rate and jitter remain unmeasured.
- Go2W height hardware motion: both fast `1.0/0.5 s` and slow `2.0/2.0 s`
  live attempts stopped on the former provisional `0.45 rad` stop threshold.
  The new `0.45 rad` warning / `0.55 rad` stop policy and live telemetry remain
  unqualified; the full sequence remains unqualified.
- Slow and fast no-tracking-stop variants: implemented and unit-tested, but not
  yet physically qualified through a full Go2W sequence.
- Adaptive joint-space and quasi-static WBC controllers: implemented with
  unit/contract coverage. The generated closed-loop MuJoCo summaries are the
  authority for each controller/gesture/initial-condition result; a failed
  case is not converted into a pass by documentation.
- Closed-loop Jetson software qualification and the four full-amplitude,
  three-cycle physical cases: not yet completed. Until all four are reviewed
  as passing, the feature branch must not be merged into `main`.
- Go2W roll hardware motion: not yet performed.
- Automatic Sport Mode restoration after a confirmed prone return: implemented
  and unit-tested, but not yet physically qualified on Go2W hardware.

Do not interpret a successful build, dry-run, or simulation as physical
qualification.
