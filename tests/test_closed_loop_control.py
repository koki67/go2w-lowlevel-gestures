import math
import unittest
from unittest import mock

import numpy as np

import go2w_closed_loop_control as control
import go2w_gesture_real as hardware


class ReferenceGovernorTests(unittest.TestCase):
    def make_governor(self, duration=1.0, timeout=8.0):
        return control.ReferenceGovernor(
            np.zeros(12), np.full(12, 0.1), duration, timeout
        )

    def test_normal_progress_and_convergence_gate(self):
        governor = self.make_governor()
        measured = np.zeros(12)
        decision = None
        for index in range(300):
            decision = governor.step(measured, np.zeros(12), np.zeros(12), 0.01, index * 0.01)
            measured = decision.q_ref.copy()
            if decision.completed:
                break
        self.assertIsNotNone(decision)
        self.assertTrue(decision.completed)
        self.assertAlmostEqual(decision.progress, 1.0)
        np.testing.assert_allclose(decision.q_ref, 0.1, atol=1.0e-12)

    def test_tracking_ratio_selects_scheduled_slow_and_stopped_speeds(self):
        governor = self.make_governor()
        measured = governor.current_ref - 0.70 * governor.envelopes
        slowed = governor.step(measured, np.zeros(12), np.zeros(12), 0.01, 0.01)
        self.assertAlmostEqual(slowed.tracking_ratio, 0.70)
        self.assertAlmostEqual(slowed.speed_scale, 0.50)

        governor = self.make_governor()
        measured = governor.current_ref - 0.95 * governor.envelopes
        stopped = governor.step(measured, np.zeros(12), np.zeros(12), 0.01, 0.01)
        self.assertEqual(stopped.speed_scale, 0.0)
        self.assertEqual(stopped.progress, 0.0)
        self.assertFalse(stopped.request_return)

    def test_persistent_envelope_breach_backs_off_to_last_converged_reference(self):
        governor = self.make_governor()
        governor.progress = 0.5
        governor.current_ref = np.full(12, 0.05)
        governor.last_converged_ref = np.full(12, 0.025)
        measured = governor.current_ref - 1.01 * governor.envelopes
        decision = None
        for index in range(11):
            decision = governor.step(
                measured, np.zeros(12), np.zeros(12), 0.01, index * 0.01
            )
        self.assertTrue(decision.request_return)
        self.assertIn("tracking envelope", decision.reason)
        np.testing.assert_allclose(decision.q_ref, governor.last_converged_ref)

    def test_timeout_never_advances_to_the_next_phase(self):
        governor = self.make_governor()
        decision = governor.step(np.zeros(12), np.zeros(12), np.zeros(12), 0.01, 8.001)
        self.assertTrue(decision.timed_out)
        self.assertTrue(decision.request_return)
        self.assertFalse(decision.completed)

    def test_torque_policy_warns_stops_returns_and_errors(self):
        limits = control.TORQUE_LIMIT_NM
        governor = self.make_governor()
        warned = governor.step(np.zeros(12), np.zeros(12), 0.61 * limits, 0.01, 0.01)
        self.assertIsNotNone(warned.warning)

        governor = self.make_governor()
        stopped = governor.step(np.zeros(12), np.zeros(12), 0.76 * limits, 0.01, 0.01)
        self.assertEqual(stopped.speed_scale, 0.0)

        governor = self.make_governor()
        returned = None
        for index in range(11):
            returned = governor.step(
                np.zeros(12), np.zeros(12), 0.86 * limits, 0.01, index * 0.01
            )
        self.assertTrue(returned.request_return)
        self.assertIn("85%", returned.reason)

        governor = self.make_governor()
        emergency = governor.step(np.zeros(12), np.zeros(12), limits, 0.01, 0.01)
        self.assertTrue(emergency.emergency)


class LoadedRollEquilibriumGateTests(unittest.TestCase):
    def make_loaded_state(self):
        target = np.zeros(12)
        error = 0.63 * control.TRACKING_ENVELOPE_RAD
        measured = target - error
        velocity = np.zeros(12)
        kp = np.asarray(hardware.KP, dtype=float)
        kd = np.asarray(hardware.KD, dtype=float)
        torque = kp * error - kd * velocity
        return target, measured, velocity, torque

    def make_gate(self, sign=1.0):
        return control.LoadedRollEquilibriumGate(
            0.0,
            sign,
            hardware.KP,
            hardware.KD,
        )

    def test_static_pd_holding_error_completes_loaded_roll_but_not_default_gate(self):
        target, measured, velocity, torque = self.make_loaded_state()
        default_gate = control.ConvergenceGate()
        loaded_gate = self.make_gate()
        status = None
        for _index in range(31):
            self.assertFalse(
                default_gate.update(target, measured, velocity, 0.01)
            )
            status = loaded_gate.update(
                target,
                measured,
                velocity,
                torque,
                0.48,
                0.01,
                endpoint_reached=True,
            )
        self.assertIsNotNone(status)
        self.assertTrue(status.completed)
        self.assertTrue(status.pd_balance_met)
        self.assertAlmostEqual(status.raw_tracking_ratio, 0.63)
        self.assertLess(status.pd_residual_ratio, 1.0e-12)

    def test_loaded_roll_gate_rejects_wrong_direction_motion_torque_and_error(self):
        target, measured, velocity, torque = self.make_loaded_state()
        cases = (
            (
                "wrong roll direction",
                measured,
                velocity,
                torque,
                -0.48,
                {"roll_direction_met": False},
            ),
            (
                "still moving",
                measured,
                np.full(12, control.LOADED_ROLL_MAX_DQ_RAD_S + 0.001),
                torque,
                0.48,
                {"low_velocity_met": False},
            ),
            (
                "unexplained torque",
                measured,
                velocity,
                np.zeros(12),
                0.48,
                {"pd_balance_met": False},
            ),
            (
                "torque margin",
                measured,
                velocity,
                control.TORQUE_WARN_RATIO * control.TORQUE_LIMIT_NM,
                0.48,
                {"torque_margin_met": False},
            ),
            (
                "raw joint bound",
                target - 0.71 * control.TRACKING_ENVELOPE_RAD,
                velocity,
                np.asarray(hardware.KP)
                * (0.71 * control.TRACKING_ENVELOPE_RAD),
                0.48,
                {"joint_bound_met": False},
            ),
        )
        for name, case_q, case_dq, case_tau, roll, expected in cases:
            with self.subTest(name=name):
                status = self.make_gate().update(
                    target,
                    case_q,
                    case_dq,
                    case_tau,
                    roll,
                    0.31,
                    endpoint_reached=True,
                )
                self.assertFalse(status.completed)
                for field, value in expected.items():
                    self.assertEqual(getattr(status, field), value)

    def test_loaded_roll_gate_never_completes_before_path_endpoint(self):
        target, measured, velocity, torque = self.make_loaded_state()
        status = self.make_gate().update(
            target,
            measured,
            velocity,
            torque,
            0.48,
            1.0,
            endpoint_reached=False,
        )
        self.assertFalse(status.completed)
        self.assertEqual(status.accumulated_s, 0.0)


class KinematicsAndContactTests(unittest.TestCase):
    def test_analytic_leg_jacobians_match_finite_difference(self):
        q = np.asarray(hardware.STANDARD, dtype=float)
        epsilon = 1.0e-7
        for leg in range(4):
            q_leg = q[3 * leg : 3 * leg + 3]
            analytic = control.leg_kinematics(q_leg, leg).jacobian
            numeric = np.zeros((3, 3))
            for joint in range(3):
                plus = q_leg.copy()
                minus = q_leg.copy()
                plus[joint] += epsilon
                minus[joint] -= epsilon
                numeric[:, joint] = (
                    control.leg_kinematics(plus, leg).wheel_position
                    - control.leg_kinematics(minus, leg).wheel_position
                ) / (2.0 * epsilon)
            np.testing.assert_allclose(analytic, numeric, atol=1.0e-8, rtol=1.0e-7)

    def synthetic_equilibrium(self):
        q = np.asarray(hardware.STANDARD, dtype=float)
        rpy = np.asarray([0.10, -0.05, 0.02])
        rotation = control.rpy_rotation(rpy)
        positions = (rotation @ control.wheel_positions(q).T).T
        torque_map = np.zeros((12, 12))
        equilibrium = np.zeros((6, 12))
        for leg, jacobian in enumerate(control.leg_jacobians(q)):
            block = slice(3 * leg, 3 * leg + 3)
            torque_map[block, block] = jacobian.T @ rotation.T
            equilibrium[:3, block] = np.eye(3)
            equilibrium[3:, block] = control.skew(positions[leg])
        upward = np.asarray([0.0, 0.0, control.BODY_WEIGHT_N])
        com_world = rotation @ control.whole_body_com(q)
        desired_wrench = np.concatenate([upward, np.cross(com_world, upward)])
        forces = equilibrium.T @ np.linalg.solve(
            equilibrium @ equilibrium.T, desired_wrench
        )
        gravity = control.gravity_torques(q, rpy)
        tau_est = -gravity - torque_map @ forces
        return q, rpy, forces.reshape(4, 3), tau_est

    def test_contact_force_qp_recovers_known_static_load(self):
        q, rpy, expected_forces, tau_est = self.synthetic_equilibrium()
        estimate = control.estimate_contact_forces(q, tau_est, rpy)
        self.assertTrue(estimate.valid, estimate.reason)
        np.testing.assert_allclose(estimate.forces, expected_forces, atol=0.002)
        self.assertLess(estimate.torque_residual_ratio, 1.0e-3)
        self.assertLess(estimate.balance_residual_ratio, 1.0e-3)
        self.assertAlmostEqual(estimate.total_vertical_load_n, control.BODY_WEIGHT_N, places=2)

    def test_contact_estimator_rejects_singular_and_inconsistent_states(self):
        singular = control.estimate_contact_forces([0.0] * 12, [0.0] * 12)
        self.assertFalse(singular.valid)
        self.assertIn("Jacobian condition", singular.reason)

        inconsistent = control.estimate_contact_forces(
            hardware.STANDARD, [0.0] * 12
        )
        self.assertFalse(inconsistent.valid)
        self.assertIsNotNone(inconsistent.reason)

    def test_contact_qp_setup_exception_is_an_invalid_estimate(self):
        class FakeSolver:
            def setup(self, **_kwargs):
                raise RuntimeError("setup failed")

        fake_osqp = mock.Mock(OSQP=FakeSolver)
        from scipy import sparse

        with mock.patch.object(
            control, "_load_qp_dependencies", return_value=(fake_osqp, sparse)
        ):
            estimate = control.estimate_contact_forces(
                hardware.STANDARD, [0.0] * 12
            )
        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.status, "setup-exception")
        self.assertIn("setup failed", estimate.reason)

    def test_height_roll_targets_and_task_estimator_have_expected_direction(self):
        baseline = [0.03, -0.02, 0.04]
        low = control.task_target_for_gesture("height", "low", baseline)
        high = control.task_target_for_gesture("height", "high", baseline)
        right = control.task_target_for_gesture("roll", "right", baseline)
        left = control.task_target_for_gesture("roll", "left", baseline)
        self.assertAlmostEqual(low.relative_height_m, -0.093178)
        self.assertAlmostEqual(high.relative_height_m, 0.076281)
        self.assertAlmostEqual(right.roll_rad - baseline[0], -0.395469)
        self.assertAlmostEqual(left.roll_rad - baseline[0], 0.395469)
        self.assertEqual(right.pitch_rad, baseline[1])
        self.assertEqual(right.yaw_rad, baseline[2])

        forces = np.tile([0.0, 0.0, control.BODY_WEIGHT_N / 4.0], (4, 1))
        standard = control.estimate_task_space(
            hardware.STANDARD, baseline, forces, baseline_height_m=0.0
        )
        lowered = control.estimate_task_space(
            hardware.LOW, baseline, forces, baseline_height_m=standard.raw_height_m
        )
        self.assertLess(lowered.relative_height_m, 0.0)


class KinematicWBCTests(unittest.TestCase):
    def make_problem(self):
        q = np.asarray(hardware.STANDARD, dtype=float)
        forces = np.tile([0.0, 0.0, control.BODY_WEIGHT_N / 4.0], (4, 1))
        estimate = control.estimate_task_space(q, [0.0, 0.0, 0.0], forces, baseline_height_m=0.0)
        target = control.WBCTarget(
            estimate.relative_height_m - 0.01, 0.02, 0.0, 0.0
        )
        return q, estimate, target

    def test_qp_respects_contact_velocity_acceleration_and_position_bounds(self):
        q, estimate, target = self.make_problem()
        solver = control.KinematicWBC()
        result = solver.solve(q, q, q, estimate, target, [0.0, 0.0, 0.0])
        self.assertTrue(result.valid, result.reason)
        self.assertLessEqual(
            np.max(np.abs(result.generalized_velocity[6:])),
            control.WBC_MAX_DDQ_RAD_S2 * control.WBC_PERIOD_S + 1.0e-6,
        )
        self.assertLess(result.contact_velocity_residual_m_s, 0.01)
        self.assertTrue(np.all(result.q_ref >= control.JOINT_LOWER_RAD))
        self.assertTrue(np.all(result.q_ref <= control.JOINT_UPPER_RAD))
        self.assertTrue(np.all(np.abs(result.q_ref - q) <= control.TRACKING_ENVELOPE_RAD))

    def test_conflicting_bounds_fail_closed_before_osqp(self):
        q, estimate, target = self.make_problem()
        result = control.KinematicWBC().solve(
            q,
            control.JOINT_UPPER_RAD + 1.0,
            q,
            estimate,
            target,
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "infeasible-bounds")

    def test_nonfinite_solver_output_fails_closed(self):
        q, estimate, target = self.make_problem()

        class FakeInfo:
            status = "solved"
            iter = 1
            solve_time = 1.0e-5
            prim_res = 0.0
            dual_res = 0.0

        class FakeSolver:
            def setup(self, **_kwargs):
                pass

            def warm_start(self, **_kwargs):
                pass

            def solve(self):
                return mock.Mock(info=FakeInfo(), x=np.full(18, np.nan))

        fake_osqp = mock.Mock(OSQP=FakeSolver)
        from scipy import sparse

        with mock.patch.object(
            control, "_load_qp_dependencies", return_value=(fake_osqp, sparse)
        ):
            result = control.KinematicWBC().solve(q, q, q, estimate, target)
        self.assertFalse(result.valid)
        self.assertIn("non-finite", result.reason)

    def test_infeasible_solver_status_fails_closed(self):
        q, estimate, target = self.make_problem()

        class FakeInfo:
            status = "primal infeasible"
            iter = 20
            solve_time = 1.0e-5
            prim_res = 1.0
            dual_res = 1.0

        class FakeSolver:
            def setup(self, **_kwargs):
                pass

            def warm_start(self, **_kwargs):
                pass

            def solve(self):
                return mock.Mock(info=FakeInfo(), x=None)

        fake_osqp = mock.Mock(OSQP=FakeSolver)
        from scipy import sparse

        with mock.patch.object(
            control, "_load_qp_dependencies", return_value=(fake_osqp, sparse)
        ):
            result = control.KinematicWBC().solve(q, q, q, estimate, target)
        self.assertFalse(result.valid)
        self.assertIn("not solved", result.reason)

    def test_total_qp_runtime_over_ten_milliseconds_fails_closed(self):
        q, estimate, target = self.make_problem()
        with mock.patch.object(
            control.time, "perf_counter", side_effect=(100.0, 100.011)
        ):
            result = control.KinematicWBC().solve(q, q, q, estimate, target)
        self.assertFalse(result.valid)
        self.assertIn("exceeded 10 ms", result.reason)

    def test_solver_setup_exception_fails_closed(self):
        q, estimate, target = self.make_problem()

        class FakeSolver:
            def setup(self, **_kwargs):
                raise RuntimeError("setup failed")

        fake_osqp = mock.Mock(OSQP=FakeSolver)
        from scipy import sparse

        with mock.patch.object(
            control, "_load_qp_dependencies", return_value=(fake_osqp, sparse)
        ):
            result = control.KinematicWBC().solve(q, q, q, estimate, target)
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "setup-exception")
        self.assertIn("setup failed", result.reason)


if __name__ == "__main__":
    unittest.main()
