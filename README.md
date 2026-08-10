# Go2W Low-Level Gestures

Containerized, fail-closed low-level gesture controller for a Unitree Go2W.
The controller talks directly to Unitree SDK2Py over CycloneDDS. It does not
use `rclpy`, Unitree ROS 2 messages, ROS 2 Foxy, or ROS 2 Humble at runtime.

The repository currently provides two selectable gestures:

| Gesture | Low-level sequence |
| --- | --- |
| `height` | Standard, three low/high cycles, standard |
| `roll` | Standard, three right/left cycles, standard |

Every live gesture shares the same control-ownership checks, watchdogs,
captured-prone shutdown, and explicit confirmation boundary.

## Clone, build, and inspect

```bash
git clone https://github.com/koki67/go2w-lowlevel-gestures.git
cd go2w-lowlevel-gestures
make build
make test
make describe
```

`make build`, `make test`, and `make describe` do not connect to the robot.

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

The image is based on Ubuntu 22.04. ROS 2 is deliberately omitted because this
controller uses SDK2Py directly. It can run alongside other Humble containers,
but sourcing a ROS environment is not required.

## Gesture definitions

### Height

- Captured prone to standard: 2.0 s; hold standard: 2.0 s.
- Three low/high cycles, each transition 1.0 s and hold 0.5 s.
- High to standard: 2.0 s; hold standard: 2.0 s.
- Standard to captured prone: 3.0 s; hold prone: 2.0 s.
- Zero-gain neutral command: 1.0 s, then stop LowCmd.

### Roll

The Go2W URDF hip-abduction range is `[-1.0472, 1.0472] rad`. Starting from
the standard hip targets, the largest symmetric common offset is `0.9472 rad`.
The roll gesture deliberately uses 70% of that value: `0.66304 rad` (about
38.0 degrees of joint offset).

- Captured prone to standard: 2.0 s; hold standard: 2.0 s.
- Three right/left cycles, each transition 0.75 s and hold 0.5 s.
- Left to standard: 2.0 s; hold standard: 2.0 s.
- Standard to captured prone: 3.0 s; hold prone: 2.0 s.
- Zero-gain neutral command: 1.0 s, then stop LowCmd.

The MuJoCo run reached approximately `+27.7/-27.6 degrees` of measured body
roll without falling and returned close to level. This is simulation evidence,
not Go2W hardware qualification.

## Read-only robot preflight

First verify the host NIC:

```bash
ip -4 addr show dev eth0
```

Then select the gesture to inspect:

```bash
make preflight-height
make preflight-roll
```

Preflight initializes DDS, reads `rt/lowstate`, and calls read-only
`CheckMode()`. It verifies the expected NIC/IP, stable prone pose, joint and
wheel velocities, IMU tilt, active Sport service, and selected gesture targets.
It does **not** call `StopMove()`, `ReleaseMode()`, `SelectMode()`, or publish
`LowCmd`.

## Live hardware execution

Do not continue unless the selected preflight succeeds and the physical safety
setup is ready.

Height gesture:

```bash
make live-height
```

Required typed confirmation:

```text
RUN GO2W LOW LEVEL
```

Roll gesture:

```bash
make live-roll
```

Required typed confirmation:

```text
RUN GO2W ROLL LOW LEVEL
```

The live ownership sequence is common to both gestures:

1. Confirm a stable, belly-down measured pose.
2. Require the gesture-specific interactive phrase.
3. Send Sport `StopMove()`.
4. Re-measure the prone shutdown target.
5. Repeatedly call `CheckMode()` and `ReleaseMode()` until no Sport service is
   active.
6. Require `rt/lowcmd` to become quiet before creating the sole user publisher.
7. Run the selected gesture at a nominal 500 Hz.
8. Return to the hardware-measured initial prone pose.
9. Hold prone, publish a zero-gain neutral command briefly, and stop LowCmd.

There is intentionally no ambiguous `make live` target. The gesture name must
be part of the command and is shown again before confirmation.

## Sport Mode restoration boundary

The SDK exposes `MotionSwitcherClient.SelectMode()`, but this repository does
not automatically reactivate Sport Mode. An overlap-free handoff from a live
user LowCmd publisher to the specific Go2W firmware's Sport controller has not
been qualified. Automatic restoration could create either a command gap or two
simultaneous owners.

Consequently, the successful final state is belly-down, LowCmd stopped, and
Sport Mode still released. Reactivate Sport Mode only with a separate,
qualified procedure while the robot is safely supported.

## Runtime watchdogs

The live controller fails closed on:

- stale or non-finite LowState;
- body roll/pitch above `0.55 rad` (about 31.5 degrees);
- leg-joint tracking error above `0.45 rad`;
- DDS write failure;
- failure to release Sport Mode;
- LowCmd traffic that does not become quiet after release;
- NIC or IP mismatch;
- unstable or implausible initial prone pose;
- an unknown or omitted gesture.

The first Ctrl+C requests a controlled return to the captured prone pose. A
second Ctrl+C abandons that return and sends a short neutral command. A process
kill, host failure, container failure, or network loss can still prevent any
software fallback. Physical support and a hardware E-stop remain mandatory.

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
- Jetson `aarch64` image build and 500 Hz timing: not yet measured.
- Go2W height hardware motion: not yet performed.
- Go2W roll hardware motion: not yet performed.
- Automatic Sport Mode restoration: intentionally not implemented.

Do not interpret a successful build, dry-run, or simulation as physical
qualification.
