#!/usr/bin/env python3
"""Dependency-free SVG diagnostics for quasi-static Go2W WBC MuJoCo runs."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
import math
from pathlib import Path
from typing import Sequence

import go2w_adaptive_plot as adaptive_plot


NAN = float("nan")


def _finite(value, default=NAN) -> float:
    try:
        converted = float(value)
    except (TypeError, ValueError):
        return default
    return converted if math.isfinite(converted) else default


def _object_value(instance, name, default=NAN) -> float:
    return _finite(getattr(instance, name, default)) if instance is not None else default


@dataclass(frozen=True)
class WBCPlotSample:
    time_s: float
    phase: str
    event: str | None
    q_ref: tuple[float, ...]
    q_measured: tuple[float, ...]
    progress: float
    speed_scale: float
    tracking_ratio: float
    torque_ratio: float
    tilt_ratio: float
    max_measured_dq_rad_s: float
    task_target_height_m: float
    task_measured_height_m: float
    task_target_roll_rad: float
    task_measured_roll_rad: float
    task_target_pitch_rad: float
    task_measured_pitch_rad: float
    task_target_yaw_rad: float
    task_measured_yaw_rad: float
    contact_force_n: tuple[float, ...]
    contact_total_vertical_load_n: float
    contact_minimum_normal_load_n: float
    contact_left_vertical_load_n: float
    contact_right_vertical_load_n: float
    contact_cop_y_m: float
    support_min_y_m: float
    support_max_y_m: float
    contact_valid: float
    contact_torque_residual_ratio: float
    contact_balance_residual_ratio: float
    contact_jacobian_condition: float
    contact_qp_solve_time_ms: float
    contact_qp_iterations: float
    wbc_qp_valid: float
    wbc_qp_solve_time_ms: float
    wbc_qp_iterations: float
    wbc_qp_primal_residual: float
    wbc_qp_dual_residual: float
    contact_velocity_residual_m_s: float
    commanded_dq_utilization: float
    commanded_ddq_utilization: float


class WBCPlotRecorder(adaptive_plot.AdaptivePlotRecorder):
    """Buffer controller-equivalent WBC evidence and render it after the run."""

    def __init__(
        self,
        enabled: bool,
        joint_names: Sequence[str],
        tracking_envelopes: Sequence[float],
        torque_limits: Sequence[float],
        *,
        body_weight_n: float,
        tilt_limit_rad: float,
        max_commanded_dq_rad_s: float,
        max_commanded_ddq_rad_s2: float,
        wbc_period_s: float,
        height_tolerance_m: float = 0.015,
        support_slow_ratio: float = 0.10,
        support_backoff_ratio: float = 0.06,
        support_return_ratio: float = 0.04,
    ) -> None:
        super().__init__(
            enabled,
            joint_names,
            tracking_envelopes,
            controller_label="WBC",
            companion_plot_label="the WBC diagnostic plots",
        )
        limits = tuple(float(value) for value in torque_limits)
        if len(limits) != 12 or any(value <= 0.0 for value in limits):
            raise ValueError("WBC plots require 12 positive torque limits")
        positive_values = (
            body_weight_n,
            tilt_limit_rad,
            max_commanded_dq_rad_s,
            max_commanded_ddq_rad_s2,
            wbc_period_s,
            height_tolerance_m,
            support_slow_ratio,
            support_backoff_ratio,
            support_return_ratio,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise ValueError("WBC plot limits must be positive and finite")
        self.torque_limits = limits
        self.body_weight_n = float(body_weight_n)
        self.tilt_limit_rad = float(tilt_limit_rad)
        self.max_commanded_dq_rad_s = float(max_commanded_dq_rad_s)
        self.max_commanded_ddq_rad_s2 = float(max_commanded_ddq_rad_s2)
        self.wbc_period_s = float(wbc_period_s)
        self.height_tolerance_m = float(height_tolerance_m)
        self.support_slow_ratio = float(support_slow_ratio)
        self.support_backoff_ratio = float(support_backoff_ratio)
        self.support_return_ratio = float(support_return_ratio)
        if not (
            self.support_return_ratio
            < self.support_backoff_ratio
            < self.support_slow_ratio
        ):
            raise ValueError("WBC support ratios must satisfy return < backoff < slow")
        self.samples: list[WBCPlotSample] = []
        self._previous_commanded_dq: tuple[float, ...] | None = None

    def record(
        self,
        *,
        time_s: float,
        phase: str,
        q_ref: Sequence[float],
        q_measured: Sequence[float],
        measured_dq: Sequence[float],
        tau_est: Sequence[float],
        body_rpy: Sequence[float],
        progress: float = NAN,
        speed_scale: float = NAN,
        task_target=None,
        task_estimate=None,
        contact=None,
        qp_result=None,
        support_y_m: Sequence[float] | None = None,
        event: str | None = None,
    ) -> None:
        if not self.enabled:
            return
        reference = tuple(float(value) for value in q_ref)
        measured = tuple(float(value) for value in q_measured)
        velocity = tuple(float(value) for value in measured_dq)
        torque = tuple(float(value) for value in tau_est)
        rpy = tuple(float(value) for value in body_rpy)
        if any(len(values) != 12 for values in (reference, measured, velocity, torque)):
            raise ValueError("WBC plot joint samples require 12 values")
        if len(rpy) != 3:
            raise ValueError("WBC plot body_rpy requires three values")
        tracking_ratio = max(
            abs(reference[index] - measured[index]) / self.tracking_envelopes[index]
            for index in range(12)
        )
        torque_ratio = max(
            abs(torque[index]) / self.torque_limits[index]
            for index in range(12)
        )
        tilt_ratio = max(abs(rpy[0]), abs(rpy[1])) / self.tilt_limit_rad

        forces = [NAN] * 12
        if contact is not None:
            try:
                forces = [
                    _finite(contact.forces[leg][axis])
                    for leg in range(4)
                    for axis in range(3)
                ]
            except (IndexError, TypeError):
                forces = [NAN] * 12
        vertical = [forces[3 * leg + 2] for leg in range(4)]
        left_load = (
            vertical[1] + vertical[3]
            if all(math.isfinite(vertical[index]) for index in (1, 3))
            else NAN
        )
        right_load = (
            vertical[0] + vertical[2]
            if all(math.isfinite(vertical[index]) for index in (0, 2))
            else NAN
        )
        support_source = () if support_y_m is None else support_y_m
        support = tuple(_finite(value) for value in support_source)
        total_vertical = _object_value(contact, "total_vertical_load_n")
        cop_y = NAN
        if (
            len(support) == 4
            and math.isfinite(total_vertical)
            and abs(total_vertical) > 1.0e-9
            and all(math.isfinite(value) for value in vertical + list(support))
        ):
            cop_y = sum(support[index] * vertical[index] for index in range(4)) / total_vertical

        commanded_dq = None
        dq_utilization = NAN
        ddq_utilization = NAN
        if qp_result is not None:
            try:
                generalized = tuple(float(value) for value in qp_result.generalized_velocity)
                if len(generalized) == 18 and all(math.isfinite(value) for value in generalized):
                    commanded_dq = generalized[6:]
                    dq_utilization = max(abs(value) for value in commanded_dq) / self.max_commanded_dq_rad_s
                    if self._previous_commanded_dq is not None:
                        ddq_utilization = max(
                            abs(commanded_dq[index] - self._previous_commanded_dq[index])
                            for index in range(12)
                        ) / (self.wbc_period_s * self.max_commanded_ddq_rad_s2)
                    self._previous_commanded_dq = commanded_dq
            except (TypeError, ValueError):
                commanded_dq = None

        contact_valid = (
            float(bool(getattr(contact, "valid", False))) if contact is not None else NAN
        )
        qp_valid = (
            float(bool(getattr(qp_result, "valid", False)))
            if qp_result is not None
            else NAN
        )
        if event is None and contact is not None and not bool(getattr(contact, "valid", False)):
            event = str(getattr(contact, "reason", "contact estimate invalid") or "contact estimate invalid")
        if event is None and qp_result is not None and not bool(getattr(qp_result, "valid", False)):
            event = str(getattr(qp_result, "reason", "WBC QP invalid") or "WBC QP invalid")

        self.samples.append(
            WBCPlotSample(
                time_s=float(time_s),
                phase=str(phase),
                event=None if event is None else str(event),
                q_ref=reference,
                q_measured=measured,
                progress=_finite(progress),
                speed_scale=_finite(speed_scale),
                tracking_ratio=tracking_ratio,
                torque_ratio=torque_ratio,
                tilt_ratio=tilt_ratio,
                max_measured_dq_rad_s=max(abs(value) for value in velocity),
                task_target_height_m=_object_value(task_target, "relative_height_m"),
                task_measured_height_m=_object_value(task_estimate, "relative_height_m"),
                task_target_roll_rad=_object_value(task_target, "roll_rad"),
                task_measured_roll_rad=_object_value(task_estimate, "roll_rad"),
                task_target_pitch_rad=_object_value(task_target, "pitch_rad"),
                task_measured_pitch_rad=_object_value(task_estimate, "pitch_rad"),
                task_target_yaw_rad=_object_value(task_target, "yaw_rad"),
                task_measured_yaw_rad=_object_value(task_estimate, "yaw_rad"),
                contact_force_n=tuple(forces),
                contact_total_vertical_load_n=total_vertical,
                contact_minimum_normal_load_n=_object_value(contact, "minimum_normal_load_n"),
                contact_left_vertical_load_n=left_load,
                contact_right_vertical_load_n=right_load,
                contact_cop_y_m=cop_y,
                support_min_y_m=min(support) if support else NAN,
                support_max_y_m=max(support) if support else NAN,
                contact_valid=contact_valid,
                contact_torque_residual_ratio=_object_value(contact, "torque_residual_ratio"),
                contact_balance_residual_ratio=_object_value(contact, "balance_residual_ratio"),
                contact_jacobian_condition=_object_value(contact, "max_jacobian_condition"),
                contact_qp_solve_time_ms=1000.0 * _object_value(contact, "solve_time_s"),
                contact_qp_iterations=_object_value(contact, "iterations"),
                wbc_qp_valid=qp_valid,
                wbc_qp_solve_time_ms=1000.0 * _object_value(qp_result, "solve_time_s"),
                wbc_qp_iterations=_object_value(qp_result, "iterations"),
                wbc_qp_primal_residual=_object_value(qp_result, "primal_residual"),
                wbc_qp_dual_residual=_object_value(qp_result, "dual_residual"),
                contact_velocity_residual_m_s=_object_value(
                    qp_result, "contact_velocity_residual_m_s"
                ),
                commanded_dq_utilization=dq_utilization,
                commanded_ddq_utilization=ddq_utilization,
            )
        )

    def write(self, output_dir: Path, stem: str) -> dict[str, object]:
        if not self.enabled:
            raise RuntimeError("WBC plot recording is disabled")
        if not self.samples:
            raise RuntimeError("no WBC plot samples were recorded")
        destination = Path(output_dir)
        destination.mkdir(parents=True, exist_ok=True)
        paths = {
            "joint_tracking_svg": destination / "{}_wbc_joint_tracking.svg".format(stem),
            "wbc_task_tracking_svg": destination / "{}_wbc_task_tracking.svg".format(stem),
            "wbc_contact_support_svg": destination / "{}_wbc_contact_support.svg".format(stem),
            "wbc_contact_qp_health_svg": destination / "{}_wbc_contact_qp_health.svg".format(stem),
            "wbc_solver_safety_svg": destination / "{}_wbc_solver_safety.svg".format(stem),
        }
        renderers = {
            "joint_tracking_svg": self._joint_tracking_svg,
            "wbc_task_tracking_svg": self._task_tracking_svg,
            "wbc_contact_support_svg": self._contact_support_svg,
            "wbc_contact_qp_health_svg": self._contact_health_svg,
            "wbc_solver_safety_svg": self._solver_safety_svg,
        }
        for key, path in paths.items():
            path.write_text(renderers[key](stem), encoding="utf-8")
        result = {key: str(path.resolve()) for key, path in paths.items()}
        result["sample_count"] = len(self.samples)
        return result

    @staticmethod
    def _values(samples, getter) -> list[float]:
        values = []
        for sample in samples:
            value = _finite(getter(sample))
            if math.isfinite(value):
                values.append(value)
        return values

    def _axis_range(
        self,
        samples,
        getters,
        *,
        default=(-1.0, 1.0),
        include=(),
        zero_floor=False,
        padding_fraction=0.08,
    ):
        values = [value for getter in getters for value in self._values(samples, getter)]
        values.extend(float(value) for value in include if math.isfinite(float(value)))
        if not values:
            return default
        lower, upper = min(values), max(values)
        if zero_floor:
            lower = min(0.0, lower)
        span = max(upper - lower, max(abs(lower), abs(upper), 1.0) * 0.05)
        padding = padding_fraction * span
        return lower - padding, upper + padding

    def _multi_panel_svg(
        self,
        stem: str,
        title: str,
        subtitle: str,
        panel_specs,
        footer: str,
    ) -> str:
        samples = self._selected_samples()
        x_max = max(self.samples[-1].time_s, 1.0e-6)
        width = 1600
        top, bottom, left, right = 145.0, 72.0, 120.0, 72.0
        row_gap, panel_height = 42.0, 150.0
        height = int(top + bottom + len(panel_specs) * panel_height + (len(panel_specs) - 1) * row_gap)
        svg = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" viewBox="0 0 {} {}">'.format(
                width, height, width, height
            ),
            '<rect width="100%" height="100%" fill="white" />',
            '<style>text { font-family: sans-serif; fill: #222; }</style>',
            '<text x="800" y="28" text-anchor="middle" font-size="20" font-weight="bold">{}</text>'.format(
                escape(stem + " " + title)
            ),
            '<text x="800" y="52" text-anchor="middle" font-size="13">{}</text>'.format(
                escape(subtitle)
            ),
        ]
        for row, spec in enumerate(panel_specs):
            panel_top = top + row * (panel_height + row_gap)
            panel_bottom = panel_top + panel_height
            y_min, y_max = spec["range"]
            if y_max <= y_min:
                y_max = y_min + 1.0

            def x_coordinate(value):
                return left + (width - right - left) * value / x_max

            def y_coordinate(value):
                bounded = min(y_max, max(y_min, value))
                return panel_bottom - panel_height * (bounded - y_min) / (y_max - y_min)

            svg.extend(
                self._phase_background(
                    x_coordinate,
                    panel_top,
                    panel_bottom,
                    labels=row == 0,
                )
            )
            svg.extend(
                self._axes(
                    left,
                    width - right,
                    panel_top,
                    panel_bottom,
                    x_max,
                    y_min,
                    y_max,
                    x_labels=row == len(panel_specs) - 1,
                )
            )
            label_x = 25.0
            label_y = panel_top + panel_height / 2.0
            svg.append(
                '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" font-size="12" font-weight="bold" transform="rotate(-90 {:.1f} {:.1f})">{}</text>'.format(
                    label_x,
                    label_y,
                    label_x,
                    label_y,
                    escape(spec["label"]),
                )
            )
            legend_x = left + 8.0
            for line_index, (_getter, color, dash, line_width, line_label) in enumerate(spec["lines"]):
                x = legend_x + line_index * 205.0
                svg.append(
                    '<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" stroke="{}" stroke-width="{:.1f}"{} />'.format(
                        x,
                        panel_top - 9.0,
                        x + 36.0,
                        panel_top - 9.0,
                        color,
                        line_width,
                        ' stroke-dasharray="{}"'.format(dash) if dash else "",
                    )
                )
                svg.append(
                    '<text x="{:.1f}" y="{:.1f}" font-size="10">{}</text>'.format(
                        x + 42.0,
                        panel_top - 5.0,
                        escape(line_label),
                    )
                )
            for threshold, color, threshold_label in spec.get("thresholds", ()):
                if y_min <= threshold <= y_max:
                    y = y_coordinate(threshold)
                    svg.append(
                        '<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" stroke="{}" stroke-width="1.1" stroke-dasharray="5 4" />'.format(
                            left, y, width - right, y, color
                        )
                    )
                    svg.append(
                        '<text x="{:.1f}" y="{:.1f}" text-anchor="end" font-size="10" fill="{}">{}</text>'.format(
                            width - right - 4.0,
                            y - 4.0,
                            color,
                            escape(threshold_label),
                        )
                    )
            for getter, color, dash, line_width, _line_label in spec["lines"]:
                svg.extend(
                    self._polyline(
                        samples,
                        getter,
                        x_coordinate,
                        y_coordinate,
                        color,
                        dash=dash,
                        width=line_width,
                    )
                )
            svg.extend(
                self._event_markers(
                    x_coordinate,
                    panel_top,
                    panel_bottom,
                    labels=row == 0,
                )
            )
        svg.extend(
            [
                '<text x="800" y="{}" text-anchor="middle" font-size="12">Time [s] from captured-prone control start</text>'.format(
                    height - 38
                ),
                '<text x="800" y="{}" text-anchor="middle" font-size="11" fill="#555">{}</text>'.format(
                    height - 17,
                    escape(footer),
                ),
                '</svg>',
            ]
        )
        return "\n".join(svg) + "\n"

    def _task_tracking_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        angular_tolerance = math.radians(2.0)
        specifications = (
            (
                "Relative height [m]",
                "task_target_height_m",
                "task_measured_height_m",
                self.height_tolerance_m,
                (-0.12, 0.10),
            ),
            (
                "Roll [rad]",
                "task_target_roll_rad",
                "task_measured_roll_rad",
                angular_tolerance,
                (-0.50, 0.50),
            ),
            (
                "Pitch [rad]",
                "task_target_pitch_rad",
                "task_measured_pitch_rad",
                angular_tolerance,
                (-0.10, 0.10),
            ),
            (
                "Yaw [rad]",
                "task_target_yaw_rad",
                "task_measured_yaw_rad",
                None,
                (-0.10, 0.10),
            ),
        )
        panels = []
        for label, target_name, measured_name, tolerance, default_range in specifications:
            target = lambda item, name=target_name: getattr(item, name)
            measured = lambda item, name=measured_name: getattr(item, name)
            getters = [target, measured]
            lines = [
                (target, "#d62728", None, 2.0, "target"),
                (measured, "#1f77b4", None, 2.1, "controller estimate"),
            ]
            if tolerance is not None:
                upper = lambda item, name=target_name, tol=tolerance: getattr(item, name) + tol
                lower = lambda item, name=target_name, tol=tolerance: getattr(item, name) - tol
                getters.extend((upper, lower))
                lines.extend(
                    (
                        (upper, "#777", "5 4", 1.0, "+ tolerance"),
                        (lower, "#777", "5 4", 1.0, "- tolerance"),
                    )
                )
            panels.append(
                {
                    "label": label,
                    "range": self._axis_range(samples, getters, default=default_range),
                    "lines": tuple(lines),
                }
            )
        return self._multi_panel_svg(
            stem,
            "WBC task-space tracking",
            "target vs controller estimate; height is relative, not absolute world height",
            panels,
            "Acceptance bands apply at hold endpoints: height +/-{:.3f} m, roll/pitch +/-2 deg.".format(
                self.height_tolerance_m
            ),
        )

    def _contact_support_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        fz_getters = tuple(
            lambda item, index=leg: item.contact_force_n[3 * index + 2]
            for leg in range(4)
        )
        fz_range = self._axis_range(
            samples,
            fz_getters,
            default=(0.0, self.body_weight_n),
            include=(0.0, self.body_weight_n),
            zero_floor=True,
        )
        total = lambda item: item.contact_total_vertical_load_n
        side_left = lambda item: item.contact_left_vertical_load_n
        side_right = lambda item: item.contact_right_vertical_load_n
        cop = lambda item: item.contact_cop_y_m
        support_min = lambda item: item.support_min_y_m
        support_max = lambda item: item.support_max_y_m
        minimum_normal = lambda item: item.contact_minimum_normal_load_n
        panels = (
            {
                "label": "Wheel normal load Fz [N]",
                "range": fz_range,
                "lines": tuple(
                    (getter, color, None, 1.9, label)
                    for getter, color, label in zip(
                        fz_getters,
                        ("#d62728", "#1f77b4", "#ff7f0e", "#2ca02c"),
                        ("FR", "FL", "RR", "RL"),
                    )
                ),
                "thresholds": (
                    (self.support_slow_ratio * self.body_weight_n, "#e6a700", "progress slow"),
                    (self.support_backoff_ratio * self.body_weight_n, "#d95f02", "path backoff"),
                    (self.support_return_ratio * self.body_weight_n, "#c62828", "controlled return"),
                ),
            },
            {
                "label": "Total vertical load [N]",
                "range": self._axis_range(
                    samples,
                    (total,),
                    default=(0.0, 1.6 * self.body_weight_n),
                    include=(0.0, 1.5 * self.body_weight_n),
                    zero_floor=True,
                ),
                "lines": ((total, "#9467bd", None, 2.2, "estimated total Fz"),),
                "thresholds": (
                    (0.50 * self.body_weight_n, "#d95f02", "50% weight"),
                    (self.body_weight_n, "#2ca02c", "model weight"),
                    (1.50 * self.body_weight_n, "#d95f02", "150% weight"),
                ),
            },
            {
                "label": "Left/right vertical load [N]",
                "range": self._axis_range(
                    samples,
                    (side_left, side_right),
                    default=(0.0, self.body_weight_n),
                    include=(0.0, self.body_weight_n),
                    zero_floor=True,
                ),
                "lines": (
                    (side_left, "#1f77b4", None, 2.1, "left FL+RL"),
                    (side_right, "#d62728", None, 2.1, "right FR+RR"),
                ),
            },
            {
                "label": "Estimated lateral CoP [m]",
                "range": self._axis_range(
                    samples,
                    (cop, support_min, support_max),
                    default=(-0.20, 0.20),
                    include=(-0.20, 0.20),
                ),
                "lines": (
                    (cop, "#9467bd", None, 2.2, "estimated CoP y"),
                    (support_min, "#777", "5 4", 1.2, "support min y"),
                    (support_max, "#777", "5 4", 1.2, "support max y"),
                ),
            },
            {
                "label": "Minimum wheel normal load [N]",
                "range": self._axis_range(
                    samples,
                    (minimum_normal,),
                    default=(0.0, 0.5 * self.body_weight_n),
                    include=(0.0, self.support_slow_ratio * self.body_weight_n),
                    zero_floor=True,
                ),
                "lines": ((minimum_normal, "#2ca02c", None, 2.1, "minimum Fz"),),
                "thresholds": (
                    (self.support_slow_ratio * self.body_weight_n, "#e6a700", "progress slow"),
                    (self.support_backoff_ratio * self.body_weight_n, "#d95f02", "path backoff"),
                    (self.support_return_ratio * self.body_weight_n, "#c62828", "controlled return"),
                ),
            },
        )
        return self._multi_panel_svg(
            stem,
            "WBC estimated contact support",
            "tau_est/Jacobian-derived loads; these are not direct foot-force sensor measurements",
            panels,
            "Body model weight {:.2f} N; support and CoP use controller kinematics only.".format(
                self.body_weight_n
            ),
        )

    def _contact_health_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        valid = lambda item: item.contact_valid
        torque_residual = lambda item: item.contact_torque_residual_ratio
        balance_residual = lambda item: item.contact_balance_residual_ratio
        jacobian = lambda item: item.contact_jacobian_condition
        solve_ms = lambda item: item.contact_qp_solve_time_ms
        iterations = lambda item: item.contact_qp_iterations
        panels = (
            {
                "label": "Contact estimate valid",
                "range": (-0.05, 1.05),
                "lines": ((valid, "#2ca02c", None, 2.1, "valid=1"),),
                "thresholds": ((1.0, "#2ca02c", "valid"),),
            },
            {
                "label": "Contact torque residual ratio",
                "range": self._axis_range(samples, (torque_residual,), default=(0.0, 0.30), include=(0.0, 0.25), zero_floor=True),
                "lines": ((torque_residual, "#1f77b4", None, 2.1, "torque residual"),),
                "thresholds": ((0.25, "#c62828", "25% limit"),),
            },
            {
                "label": "Force/moment balance ratio",
                "range": self._axis_range(samples, (balance_residual,), default=(0.0, 0.20), include=(0.0, 0.15), zero_floor=True),
                "lines": ((balance_residual, "#9467bd", None, 2.1, "balance residual"),),
                "thresholds": ((0.15, "#c62828", "15% limit"),),
            },
            {
                "label": "Max Jacobian condition number",
                "range": self._axis_range(samples, (jacobian,), default=(0.0, 220.0), include=(0.0, 200.0), zero_floor=True),
                "lines": ((jacobian, "#ff7f0e", None, 2.1, "max condition"),),
                "thresholds": ((200.0, "#c62828", "configured limit"),),
            },
            {
                "label": "Contact-force QP solve [ms]",
                "range": self._axis_range(samples, (solve_ms,), default=(0.0, 10.5), include=(0.0, 10.0), zero_floor=True),
                "lines": ((solve_ms, "#2ca02c", None, 2.1, "contact QP solve"),),
                "thresholds": ((10.0, "#c62828", "100 Hz period"),),
            },
            {
                "label": "Contact-force QP iterations",
                "range": self._axis_range(samples, (iterations,), default=(0.0, 1000.0), include=(0.0, 1000.0), zero_floor=True),
                "lines": ((iterations, "#8c564b", None, 2.1, "OSQP iterations"),),
                "thresholds": ((1000.0, "#c62828", "iteration ceiling"),),
            },
        )
        return self._multi_panel_svg(
            stem,
            "WBC contact-estimator health",
            "validity prerequisites for using tau_est/J(q)^T force inference",
            panels,
            "Invalid samples and reasons are marked in red; invalid contact must never be hidden by plotting.",
        )

    def _solver_safety_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        solve_ms = lambda item: item.wbc_qp_solve_time_ms
        iterations = lambda item: item.wbc_qp_iterations
        primal = lambda item: item.wbc_qp_primal_residual
        dual = lambda item: item.wbc_qp_dual_residual
        contact_velocity = lambda item: item.contact_velocity_residual_m_s
        dq_util = lambda item: item.commanded_dq_utilization
        ddq_util = lambda item: item.commanded_ddq_utilization
        tracking = lambda item: item.tracking_ratio
        torque = lambda item: item.torque_ratio
        tilt = lambda item: item.tilt_ratio
        panels = (
            {
                "label": "WBC QP solve [ms]",
                "range": self._axis_range(samples, (solve_ms,), default=(0.0, 10.5), include=(0.0, 10.0), zero_floor=True),
                "lines": ((solve_ms, "#2ca02c", None, 2.1, "total WBC solve"),),
                "thresholds": (
                    (5.0, "#e6a700", "desktop p99 target"),
                    (8.0, "#d95f02", "Jetson p99 target"),
                    (10.0, "#c62828", "runtime stop"),
                ),
            },
            {
                "label": "WBC QP iterations",
                "range": self._axis_range(samples, (iterations,), default=(0.0, 1000.0), include=(0.0, 1000.0), zero_floor=True),
                "lines": ((iterations, "#8c564b", None, 2.1, "OSQP iterations"),),
                "thresholds": ((1000.0, "#c62828", "iteration ceiling"),),
            },
            {
                "label": "WBC primal/dual residual",
                "range": self._axis_range(samples, (primal, dual), default=(0.0, 0.0006), include=(0.0, 0.0005), zero_floor=True),
                "lines": (
                    (primal, "#1f77b4", None, 2.0, "primal"),
                    (dual, "#9467bd", None, 2.0, "dual"),
                ),
                "thresholds": ((0.0005, "#c62828", "acceptance limit"),),
            },
            {
                "label": "Contact velocity residual [m/s]",
                "range": self._axis_range(samples, (contact_velocity,), default=(0.0, 0.012), include=(0.0, 0.01), zero_floor=True),
                "lines": ((contact_velocity, "#ff7f0e", None, 2.1, "max wheel residual"),),
                "thresholds": ((0.01, "#c62828", "qualification limit"),),
            },
            {
                "label": "Command constraint utilization",
                "range": (0.0, 1.08),
                "lines": (
                    (dq_util, "#1f77b4", None, 2.0, "|dq| / 1 rad/s"),
                    (ddq_util, "#d62728", None, 2.0, "|ddq| / 4 rad/s^2"),
                ),
                "thresholds": ((1.0, "#c62828", "hard bound"),),
            },
            {
                "label": "Joint tracking envelope ratio",
                "range": self._axis_range(samples, (tracking,), default=(0.0, 1.05), include=(0.0, 1.0), zero_floor=True),
                "lines": ((tracking, "#1f77b4", None, 2.1, "max tracking ratio"),),
                "thresholds": (
                    (0.50, "#e6a700", "slowdown"),
                    (0.90, "#d95f02", "progress stop"),
                    (1.00, "#c62828", "controlled return"),
                ),
            },
            {
                "label": "tau_est / model torque range",
                "range": self._axis_range(samples, (torque,), default=(0.0, 1.05), include=(0.0, 1.0), zero_floor=True),
                "lines": ((torque, "#9467bd", None, 2.1, "max tau_est ratio"),),
                "thresholds": (
                    (0.60, "#e6a700", "warning"),
                    (0.75, "#d95f02", "progress stop"),
                    (0.85, "#c62828", "controlled return"),
                    (1.00, "#7f0000", "immediate error"),
                ),
            },
            {
                "label": "Body tilt / 0.55 rad watchdog",
                "range": self._axis_range(samples, (tilt,), default=(0.0, 1.05), include=(0.0, 1.0), zero_floor=True),
                "lines": ((tilt, "#2ca02c", None, 2.1, "max |roll,pitch| ratio"),),
                "thresholds": ((1.0, "#c62828", "tilt watchdog"),),
            },
        )
        return self._multi_panel_svg(
            stem,
            "WBC solver timing and safety",
            "QP health, hard-bound utilization, tracking, effort, and tilt",
            panels,
            "Simulation does not synthesize hardware 500 Hz deadline misses; live CSV remains authoritative for those.",
        )
