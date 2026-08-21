# Plan

- Created: 2026-08-21T17:20:18+09:00
- Snapshot: 2026-08-21T17:20:18+09:00
- Status: final
- Language: en
- Session: unavailable
- Branch: `main` (implementation branch `feat/adaptive-wbc-gestures` will be created)
- Workspace: `/home/user/ws/go2w-lowlevel-gestures`
- Translated from: `2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.md`
- Scope: Add a tracking-adaptive joint sequence and a posture-feedback plus quasi-static WBC sequence while preserving the existing no-tracking-stop variants. Implement height and roll for three cycles each using the fast timings, validate them in MuJoCo, evaluate them on the Jetson and physical robot, and merge them into main only after they pass.

## Context and current state

- The current controller interpolates from the measured initial joint angles to fixed joint poses according to elapsed time and sends those poses to joint PD through LowCmd. Measured error is logged and can stop the run, but it does not adjust trajectory progress or body posture.
- `live-fast-*` stops at 0.55 rad tracking error, whereas `live-fast-no-tracking-stop-*` continues sending the same commands. Hardware overload errors have also been observed with the latter.
- Do not delete or rename any of the four existing scripts, especially the no-tracking-stop variants, and do not change their commands or stop policies.
- The current `main` has uncommitted changes for quick-stand, shake-off, and optional plot saving. `git diff --check`, nine simulation contract tests, all 36 tests in Docker, `make sim-doctor`, and `make sim-describe` have already passed. Generated SVG files under `runs/` are ignored.
- The development machine is an x86_64 desktop. The physical Jetson is `unitree@192.168.111.110`, with the repository at `/home/unitree/go2w-lowlevel-gestures`. Never store the SSH password in automation; the user runs Jetson commands in a terminal on the Jetson.
- The Go2W model is `/home/user/ws/unitree_mujoco/unitree_robots/go2w/go2w.xml`, with SHA-256 `c8feaef4afdf360335727c80a826d1611950c562a3daaa5b5bfcf8b57f6859a6`. The model mass is 19.126408 kg, static weight is approximately 187.63 N, and the joint torque ranges are hip/thigh ±23.7 N·m and calf ±45.43 N·m.
- LowState provides `q`, `dq`, `tau_est`, temperature, mode, lost, IMU, `power_a`, and other fields, but Go2W `foot_force` must not be treated as a valid real sensor reading. [Unitree LowState](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/LowState_.hpp)

## Decisions and constraints

- Add the following two scripts; each script accepts `--gesture height|roll`.

  - `go2w_gesture_real_adaptive.py`: joint-space control with a tracking-coupled reference governor and convergence gate.
  - `go2w_gesture_real_wbc.py`: a quasi-static whole-body controller that adds closed-loop body height, roll, and pitch control after the same adaptive startup.

- For both controllers, the repeated portion uses the fast baseline: 1.0-second transitions, minimum 0.5-second holds, and three cycles. Tracking state may extend wall-clock duration. Preserve the 2.0-second startup-to-STANDARD baseline and 3.0-second return-to-prone baseline.
- The WBC variant does not switch to direct torque control. A constrained kinematic QP at 100 Hz generates joint velocity and position targets, which are sent through the existing 500 Hz LowCmd position PD. The commanded `tau` remains zero.
- Use OSQP for the QP and pin a version that provides Python 3.10 and aarch64 wheels. [OSQP Python API](https://osqp.org/docs/get_started/python.html), [OSQP 1.1.3](https://pypi.org/project/osqp/)
- Keep the WBC runtime model in the repository as explicit parameters: the minimum link lengths, hip locations, masses, centers of mass, joint axes, joint limits, and torque ranges extracted from the audited MJCF. The external `unitree_mujoco` checkout must not be a Jetson runtime dependency.
- Task-space targets must correspond to the existing gestures.

  - height: relative to STANDARD, LOW is `−0.093178 m` and HIGH is `+0.076281 m`.
  - roll: relative to STANDARD, the left and right targets are `±0.395469 rad`, approximately `±22.659°`.
  - Hold pitch at the STANDARD reference, and hold yaw and horizontal position at their startup references.

- In place of foot sensors, under a slow quasi-static assumption, estimate each wheel contact force using `τ_contact ≈ τ_est − τ_gravity` and `τ_contact ≈ J(q)ᵀ·f_contact`. Compute gravity torque from the model masses and centers of mass.
- Do not apply the contact estimate to control unconditionally. Check the Jacobian condition number, QP state, force and moment balance residuals, total vertical load, and positive normal load at every wheel. Enter the WBC gesture only after the estimate has remained valid continuously for 0.5 seconds. If it is invalid, return to prone under adaptive control and exit nonzero.
- Do not use WBC for belly-down-to-STANDARD. Use the same joint-space control as the adaptive variant. Establish a four-wheel load estimate while holding STANDARD; on hardware, require the user to visually confirm belly clearance and four-wheel support before switching to WBC.
- Per the user's selection, hardware motion begins at 100% amplitude and runs three cycles. Do not run an amplitude-ramp test. Do not automatically retry after an anomaly, fall back to no-tracking-stop, or continue to another gesture.
- Hardware operation is fail-closed. Do not execute `--live` until the user has powered the robot, restrained the wheels, installed support, prepared a spotter and E-stop, and supplied the required confirmation input.

## Public interfaces

- Add these Make targets.

```text
describe-adaptive-height
describe-adaptive-roll
preflight-adaptive-height
preflight-adaptive-roll
live-adaptive-height
live-adaptive-roll

describe-wbc-height
describe-wbc-roll
preflight-wbc-height
preflight-wbc-roll
live-wbc-height
live-wbc-roll

sim-adaptive-height
sim-adaptive-roll
sim-wbc-height
sim-wbc-roll

qualify-live-adaptive-height
qualify-live-adaptive-roll
qualify-live-wbc-height
qualify-live-wbc-roll
```

- `preflight-*` reads DDS and LowState but does not release Sport or send LowCmd.
- On the Jetson, `qualify-live-*` checks Git state, aarch64, NIC/IP, build, all tests, describe, and preflight in order. Only after those checks pass does it display a Japanese physical-readiness checklist and dedicated confirmation phrase, then launch the matching `live-*` target.
- While the WBC variant holds STANDARD and continues sending at 500 Hz, a separate input thread requires `FOUR WHEELS LOADED AND BELLY CLEAR`. Waiting for input must not stop the control cycle.
- Do not change the existing tracking log. New logs contain a controller-type field and are written as CSV plus summary JSON. Record q/dq/target/error, IMU, tau_est, mode/lost, temperature, power, phase, trajectory progress, speed multiplier, and deadline misses. For WBC, also record task targets and measurements, estimated contact forces, balance residuals, QP status, iterations, and solve time.
- The qualification runner saves the Git SHA, executed command, exit code, terminal log, summary, and hashes under `runs/qualification/<timestamp>/`. Do not perform file I/O in the active 500 Hz loop.

## Final plan

1. At implementation start, save this final plan globally and translate the same complete plan into English.

   - `~/.codex/memories/rollout_plans/2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.md`
   - `~/.codex/memories/rollout_plans/2026-08-21_172018_plan_go2w-adaptive-wbc-gestures.en.md`

   Save the same two files under `docs/plans/`. The Japanese version is authoritative; do not omit any structure, values, commands, or acceptance criteria from the English translation.

2. Re-audit the current uncommitted simulation work. Exclude unintended files and generated files under `runs/`, then rerun `git diff --check`, the nine simulation tests, the Docker build, all 36 tests, `sim-doctor`, and `sim-describe`.

3. Commit only the current changes to `main` as `feat: add quick-stand and shake-off simulations` and push them. Confirm that local HEAD, `origin/main`, and remote `main` have the same SHA. Do not include the plan files in this pre-existing work commit.

4. Create `feat/adaptive-wbc-gestures` from the updated `main`. Put all subsequent implementation and plan files only on this branch. Do not delete, rename, or change the thresholds of the existing no-tracking-stop scripts.

5. Make the ownership, stopping, and Sport-restoration behavior in `go2w_gesture_real.py` reusable.

   - Add a controller factory or subclass hook without changing the results produced by the existing wrappers.
   - For new controllers, read tau_est, mode, lost, and temperature for all 12 leg motors and four wheel motors, plus IMU gyro/acceleration and power_v/power_a from LowState.
   - Keep stale/nonfinite, DDS, body tilt, 0.55 rad tracking-stop, controlled return to prone, neutral, and Sport restoration in a shared safety layer.
   - Stop if `lost` increases from its startup value or mode deviates from its expected value. Because there are no official Go2W thresholds for temperature and power, do not invent absolute stop thresholds; record them and include them in summary evaluation.

6. Add the pure-computation module `go2w_closed_loop_control.py`.

   - Separate the smoothstep path, phase state machine, reference governor, convergence check, kinematics, Jacobians, gravity torque, contact-force estimation, task-space estimator, and WBC QP from DDS and file I/O.
   - Set command-versus-measurement envelope limits to hip `0.18 rad`, thigh `0.14 rad`, and calf `0.25 rad`, retaining margin for KD and gravity.
   - At normalized error no greater than 50% of the envelope, use scheduled speed. From 50% to 90%, reduce speed linearly. At 90% or above, stop progress. If 100% or above persists for 0.1 seconds, back off to the last converged pose.
   - Complete a transition only after every joint is within 50% of its envelope and maximum joint speed is `≤0.20 rad/s` continuously for 0.30 seconds.
   - Use wall-clock timeouts of 8 seconds for fast transitions, 12 seconds for startup, 15 seconds for the return to prone, and 5 seconds for hold convergence. Never advance to the next phase after a timeout.
   - Warn when tau_est reaches 60% of the model limit, stop progress at 75%, begin a controlled return if 85% persists for 0.1 seconds, and raise an immediate error at 100%. State clearly that these are provisional application protection thresholds, not physically qualified limits.

7. Add `go2w_gesture_real_adaptive.py`.

   - For height, run STANDARD→LOW→HIGH for three cycles. For roll, run STANDARD→RIGHT→LEFT for three cycles.
   - Preserve the existing target joint poses; change only trajectory progress and phase-completion conditions.
   - Log slowdowns, stops, and return reasons caused by tracking error or tau_est.
   - Do not implement a fallback to no-tracking-stop.

8. Implement the quasi-static WBC.

   - At 100 Hz, solve a QP whose variables are generalized velocity `[base twist 6, leg dq 12]`.
   - Use zero wheel-center velocity as the four contact constraints, body z/roll/pitch tracking as the primary task, and x/y/yaw holding plus the existing joint posture as secondary tasks.
   - Use joint position limits, `|dq|≤1.0 rad/s`, commanded acceleration `≤4.0 rad/s²`, and the reference-governor envelope as hard bounds.
   - Configure OSQP with warm starting, fixed iteration limits, and fixed primal/dual residual tolerances. An unsolved or infeasible problem or nonfinite result immediately stops task progress and starts an adaptive return.
   - Resend the latest safe q_ref through LowCmd at 500 Hz and update the QP every five ticks. Stop if a 100 Hz solve exceeds 10 ms or 500 Hz deadlines are missed consecutively.
   - Estimate body roll and pitch from the IMU. Estimate relative height from fixed wheel contacts and leg kinematics using a load-weighted average. Do not claim it is absolute world height.
   - `go2w_gesture_real_wbc.py` executes adaptive startup→STANDARD→contact estimate and visual confirmation→three task-space cycles→adaptive STANDARD/prone return.

9. Update Docker, Make, and documentation.

   - Preserve `numpy==1.26.4`, pin `scipy==1.13.1` and `osqp==1.1.3`, and verify x86_64 and aarch64 builds. [SciPy 1.13.1](https://pypi.org/project/scipy/1.13.1/)
   - Include the two new hardware wrappers, shared control module, simulation runner, and qualification runner in the image.
   - In README, distinguish “adaptive joint-space” from “quasi-static kinematic WBC”; explain that the latter is not direct-torque dynamic WBC and that 100% hardware testing was the user's choice.
   - Preserve compatibility with existing Make targets and the existing explanation of no-tracking-stop.

10. Add `simulation/go2w_closed_loop_sequence_sim.py` for MuJoCo.

    - Use the same pure control kernel as hardware, replacing only the DDS ownership portion with a simulation harness.
    - Run adaptive/WBC × height/roll for three cycles each.
    - In addition to the normal spawn, prepare an asymmetric prone state and a heavily belly-loaded prone state, then start control from those measured states.
    - Do not modify the external MJCF or source checkout; use only a temporary scene.
    - Use simulator ground-truth base pose, contacts, and actuator forces only in evaluation logs; do not mix them into hardware-equivalent control inputs.

11. Add tests.

    - Reference-governor normal progress, slowdown, stop, backoff, timeout, and convergence gate.
    - q_ref envelope, joint velocity, acceleration, and position limits.
    - Agreement between analytical and finite-difference Jacobians.
    - Recovery of tau_est generated from known contact forces, gravity compensation, and rejection of singular postures and inconsistent loading.
    - Task-space height and roll direction and target values, and four-wheel contact-velocity residual.
    - Fail-closed behavior for OSQP infeasible, timeout, and nonfinite output.
    - No file I/O in the 500 Hz loop.
    - Three height/roll cycles, return to prone, neutral, and Sport restoration for adaptive and WBC.
    - No change to the existing 36 tests or the outputs and stop policies of the four existing wrappers.
    - Without `--live`, the Jetson qualifier never sends LowCmd; after any failed stage, it does not execute subsequent stages.

12. Run desktop qualification.

    - `git diff --check`
    - Docker build, all unit and contract tests, and `pip check`
    - All describe and preflight dry runs
    - MuJoCo adaptive/WBC × height/roll × three initial conditions
    - Summarize task tracking, contact-estimate residual, tau_est ratio, tracking error, QP timing, and return posture.
    - Clearly separate simulation PASS from hardware qualification.

13. Split the implementation into three logical commits.

    - `feat: add adaptive closed-loop gestures`
    - `feat: add quasi-static whole-body gestures`
    - `test: add closed-loop gesture qualification`

    Rerun Docker, tests, and describe in a clean clone, then push `feat/adaptive-wbc-gestures` to origin and record the remote SHA.

14. Begin Jetson hardware evaluation when the user has powered on the robot.

    - The user changes to `/home/unitree/go2w-lowlevel-gestures` in a Jetson terminal.
    - Stop if the worktree is dirty; never stash, reset, or delete automatically.
    - Fetch and switch to the feature branch, matching the SHA evaluated on the desktop.
    - Run in this order: adaptive-height, adaptive-roll, WBC-height, WBC-roll.
    - Before every run, the user checks robot power, stationary belly-down posture, restrained wheels, support fixture, spotter, E-stop, and exclusive LowCmd ownership.
    - The user launches the corresponding `make qualify-live-*`; the script automatically runs build, tests, describe, and preflight.
    - After preflight passes, require the dedicated confirmation phrase. For WBC, after reaching STANDARD, display the four-wheel load estimate and require belly-clear confirmation again.
    - Every gesture starts at 100% amplitude and performs three cycles.
    - If any anomaly, firmware error, E-stop, controlled-return failure, or unconfirmed Sport restoration occurs, stop all remaining live tests. Do not retry with no-tracking-stop.

15. Only if all four cases pass, merge the feature branch into `main` on the desktop using `--no-ff`, rerun all tests, and push. Compare local HEAD, `origin/main`, and the GitHub remote SHA. Do not delete the feature branch.

## Validation

### Structure and software

- The existing uncommitted work lands on main as an independent commit and is not mixed with this feature's commits.
- All four existing scripts and all existing Make targets remain present.
- Docker builds, `pip check`, and all tests pass on x86_64 and Jetson aarch64.
- Desktop QP solve p99 is below 5 ms, Jetson p99 is below 8 ms, and the 500 Hz publication deadline is met continuously.

### MuJoCo

- Adaptive/WBC height/roll each complete three cycles and return to prone from all three initial conditions.
- The adaptive controller does not exceed the q_ref envelope or reach the 0.55 rad watchdog.
- At the end of each hold, WBC height error is `≤0.015 m` and roll/pitch error is `≤2°`.
- Four-wheel contact-velocity residual is `≤0.01 m/s`, and contact-force-estimate balance residual is `≤15%` of body weight.
- No model joint/torque constraint, tilt, solver, or state-freshness violation occurs.
- These results qualify simulation only and must not be described as physical PASS.

### Hardware

- Every run saves the Git SHA, preflight evidence, user confirmations, live log, summary, and exit code.
- All four cases complete three cycles without firmware errors, E-stop, or tracking/tau/tilt/mode/lost/watchdog violations.
- For WBC, the four-wheel support estimate is valid, and the user visually confirms belly clearance, intended vertical movement or left/right roll, and no wheel slip, wheel lift, or floor strike.
- Every run confirms return to the captured prone pose, closure of the LowCmd writer, and restoration of the startup Sport service.
- Do not merge to main before hardware qualification passes.

## Acceptance criteria

- `go2w_gesture_real_adaptive.py` and `go2w_gesture_real_wbc.py` both require an explicit height or roll selection.
- The adaptive variant slows, stops, or returns based on measured tracking and never advances a phase based on time alone.
- The WBC variant closes body-pose feedback using IMU and kinematics, and validates four-wheel load estimation from tau_est plus Jacobians before task-space motion.
- Existing no-tracking-stop variants remain available but are never selected automatically by the new controllers.
- Simulation PASS, Jetson software PASS, physical hardware PASS, and Git publication state are recorded separately.
- Merge to main only after all four 100%-amplitude, three-cycle live runs pass.

## Risks / cautions

- `tau_est` is not a direct foot-force sensor and is affected by friction, gravity-model error, unit variation, temperature, and Jacobian conditioning. Do not run WBC when the estimates are inconsistent.
- There is no sensor that directly detects belly contact. Combine adaptive startup, total-load consistency, kinematic residuals, and user observation, but do not claim recovery from arbitrary initial placements.
- This WBC is quasi-static and kinematic, not a dynamic WBC that directly optimizes torque. It is not suitable for high speed, jumping, or contact switching.
- Beginning at 100% amplitude for three cycles is riskier than staged amplitude testing. Keep the normal watchdogs; stop immediately after an anomaly and do not retry.
- Simulator tau_est comes from actuator force, while real-hardware tau_est is estimated; agreement is not guaranteed. MuJoCo PASS alone does not establish physical safety.
- Remeasure WBC solver and SciPy/OSQP aarch64 timing on the Jetson; do not proceed to live motion if the timing constraints are not met.
- Do not overwrite, stash, reset, or delete the current uncommitted work, generated SVGs, or dirty work on the Jetson.
