# Go2W Height Sequence

Containerized, fail-closed low-level height-sequence controller for a Unitree
Go2W. The controller talks directly to Unitree SDK2Py over CycloneDDS. It does
not use `rclpy`, Unitree ROS 2 messages, ROS 2 Foxy, or ROS 2 Humble at runtime.

The repository is intended to provide one path from clone to execution:

```bash
git clone https://github.com/koki67/go2w-height-sequence.git
cd go2w-height-sequence
make build
make test
make describe
```

Only `make preflight` and `make live` connect to the robot-facing network.

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

## Commands

Build the image. The build itself runs the non-hardware unit tests and the
`--describe` check:

```bash
make build
```

Run the same non-hardware tests again from the finished image:

```bash
make test
make describe
```

### Read-only robot preflight

First verify the host NIC:

```bash
ip -4 addr show dev eth0
```

Then run the default preflight:

```bash
make preflight
```

The preflight initializes DDS, reads `rt/lowstate`, and calls read-only
`CheckMode()`. It verifies the expected NIC/IP, stable prone pose, joint and
wheel velocities, IMU tilt, and active Sport service. It does **not** call
`StopMove()`, `ReleaseMode()`, `SelectMode()`, or publish `LowCmd`.

### Live hardware sequence

Do not continue unless the preflight succeeds and the physical safety setup is
ready. Live motion requires both the explicit target and a typed confirmation:

```bash
make live
```

The program asks for this exact phrase before changing control ownership:

```text
RUN GO2W LOW LEVEL
```

The live ownership sequence is:

1. Reconfirm the measured stable prone pose.
2. Send Sport `StopMove()`.
3. Re-measure the prone pose.
4. Repeatedly call `CheckMode()` and `ReleaseMode()` until no Sport service is
   active.
5. Require `rt/lowcmd` to become quiet before creating the sole user publisher.
6. Run the low-level pose sequence at a nominal 500 Hz.
7. Return to the hardware-measured initial prone pose.
8. Hold prone, publish a zero-gain neutral command briefly, and stop `LowCmd`.

The pose sequence is:

- captured prone → standard: 2.0 s; hold standard: 2.0 s;
- three cycles of low/high, each transition 1.0 s and hold 0.5 s;
- high → standard: 2.0 s; hold standard: 2.0 s;
- standard → captured prone: 3.0 s; hold prone: 2.0 s;
- zero-gain neutral command: 1.0 s, then stop publication.

## Sport Mode restoration boundary

The SDK exposes `MotionSwitcherClient.SelectMode()`, but this repository does
not automatically reactivate Sport Mode. An overlap-free handoff from a live
user `LowCmd` publisher to the specific Go2W firmware's Sport controller has
not been qualified. Automatic restoration could create either a command gap or
two simultaneous owners.

Consequently, the successful final state is belly-down, LowCmd stopped, and
Sport Mode still released. Reactivate Sport Mode only with a separate,
qualified procedure while the robot is safely supported.

## Runtime watchdogs

The live controller fails closed on:

- stale or non-finite `LowState`;
- excessive roll/pitch;
- excessive leg-joint tracking error;
- DDS write failure;
- failure to release Sport Mode;
- LowCmd traffic that does not become quiet after release;
- NIC or IP mismatch;
- unstable or implausible initial prone pose.

The first Ctrl+C requests a controlled return to the captured prone pose. A
second Ctrl+C abandons that return and sends a short neutral command. A process
kill, host failure, container failure, or network loss can still prevent any
software fallback. Physical support and a hardware E-stop remain mandatory.

## Qualification status

- Container build: required before use.
- Unit and command-generation tests: run during every image build.
- MuJoCo sequence: validated separately, but not proof of hardware safety.
- Jetson 500 Hz timing: not yet measured.
- Go2W hardware motion: not yet performed.
- Automatic Sport Mode restoration: intentionally not implemented.

Do not interpret a successful build, dry-run, or simulation as physical
qualification.
