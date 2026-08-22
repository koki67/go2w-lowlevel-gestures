#!/usr/bin/env python3
"""Dependency-free SVG plots for adaptive Go2W MuJoCo runs."""

from __future__ import annotations

from dataclasses import dataclass
from html import escape
import math
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class AdaptivePlotSample:
    time_s: float
    phase: str
    phase_elapsed_s: float
    nominal_progress: float
    progress: float
    speed_scale: float
    tracking_ratio: float
    torque_ratio: float
    limiting_joint: str
    event: str | None
    q_ref: tuple[float, ...]
    q_measured: tuple[float, ...]


class AdaptivePlotRecorder:
    """Buffer adaptive telemetry in memory and write plots after control stops."""

    def __init__(
        self,
        enabled: bool,
        joint_names: Sequence[str],
        tracking_envelopes: Sequence[float],
        *,
        controller_label: str = "adaptive",
        companion_plot_label: str = "the governor plot",
    ) -> None:
        self.enabled = bool(enabled)
        self.joint_names = tuple(str(name) for name in joint_names)
        self.tracking_envelopes = tuple(
            float(value) for value in tracking_envelopes
        )
        if len(self.joint_names) != 12 or len(self.tracking_envelopes) != 12:
            raise ValueError("adaptive plots require 12 joint names and envelopes")
        if any(value <= 0.0 for value in self.tracking_envelopes):
            raise ValueError("tracking envelopes must be positive")
        self.controller_label = str(controller_label)
        self.companion_plot_label = str(companion_plot_label)
        self.samples: list[AdaptivePlotSample] = []

    def record(
        self,
        *,
        time_s: float,
        phase: str,
        phase_elapsed_s: float,
        phase_duration_s: float,
        progress: float,
        speed_scale: float,
        tracking_ratio: float,
        torque_ratio: float,
        q_ref: Sequence[float],
        q_measured: Sequence[float],
        event: str | None = None,
    ) -> None:
        if not self.enabled:
            return
        reference = tuple(float(value) for value in q_ref)
        measured = tuple(float(value) for value in q_measured)
        if len(reference) != 12 or len(measured) != 12:
            raise ValueError("adaptive plot samples require 12 joint values")
        if phase_duration_s <= 0.0 or not math.isfinite(phase_duration_s):
            raise ValueError("phase_duration_s must be positive and finite")
        normalized_errors = [
            abs(reference[index] - measured[index])
            / self.tracking_envelopes[index]
            for index in range(12)
        ]
        limiting_index = max(range(12), key=normalized_errors.__getitem__)
        nominal_progress = min(
            1.0, max(0.0, float(phase_elapsed_s) / float(phase_duration_s))
        )
        self.samples.append(
            AdaptivePlotSample(
                time_s=float(time_s),
                phase=str(phase),
                phase_elapsed_s=float(phase_elapsed_s),
                nominal_progress=nominal_progress,
                progress=float(progress),
                speed_scale=float(speed_scale),
                tracking_ratio=float(tracking_ratio),
                torque_ratio=float(torque_ratio),
                limiting_joint=self.joint_names[limiting_index],
                event=None if event is None else str(event),
                q_ref=reference,
                q_measured=measured,
            )
        )

    def write(self, output_dir: Path, stem: str) -> dict[str, object]:
        if not self.enabled:
            raise RuntimeError("adaptive plot recording is disabled")
        if not self.samples:
            raise RuntimeError("no adaptive plot samples were recorded")
        destination = Path(output_dir)
        destination.mkdir(parents=True, exist_ok=True)
        joint_path = destination / "{}_adaptive_joint_tracking.svg".format(stem)
        governor_path = destination / "{}_adaptive_governor.svg".format(stem)
        joint_path.write_text(self._joint_tracking_svg(stem), encoding="utf-8")
        governor_path.write_text(self._governor_svg(stem), encoding="utf-8")
        return {
            "sample_count": len(self.samples),
            "joint_tracking_svg": str(joint_path.resolve()),
            "adaptive_governor_svg": str(governor_path.resolve()),
        }

    def _downsample_indices(self, maximum: int = 2400) -> list[int]:
        count = len(self.samples)
        if count <= maximum:
            return list(range(count))
        step = math.ceil((count - 1) / (maximum - 1))
        indices = set(range(0, count, step))
        indices.add(count - 1)
        for index in range(1, count):
            if self.samples[index].phase != self.samples[index - 1].phase:
                indices.add(index - 1)
                indices.add(index)
        return sorted(indices)

    def _selected_samples(self) -> list[AdaptivePlotSample]:
        return [self.samples[index] for index in self._downsample_indices()]

    def _phase_spans(self) -> list[tuple[str, float, float]]:
        spans: list[tuple[str, float, float]] = []
        phase = self.samples[0].phase
        start = self.samples[0].time_s
        for index in range(1, len(self.samples)):
            sample = self.samples[index]
            if sample.phase != phase:
                spans.append((phase, start, sample.time_s))
                phase = sample.phase
                start = sample.time_s
        end = max(self.samples[-1].time_s, start + 1.0e-6)
        spans.append((phase, start, end))
        return spans

    @staticmethod
    def _polyline(
        samples: Sequence[AdaptivePlotSample],
        value_getter,
        x_coordinate,
        y_coordinate,
        color: str,
        *,
        dash: str | None = None,
        width: float = 2.0,
    ) -> list[str]:
        lines: list[str] = []
        segment: list[AdaptivePlotSample] = []
        previous_phase = None
        for sample in samples:
            if previous_phase is not None and sample.phase != previous_phase:
                if segment:
                    lines.extend(
                        AdaptivePlotRecorder._polyline_segment(
                            segment,
                            value_getter,
                            x_coordinate,
                            y_coordinate,
                            color,
                            dash=dash,
                            width=width,
                        )
                    )
                segment = []
            segment.append(sample)
            previous_phase = sample.phase
        if segment:
            lines.extend(
                AdaptivePlotRecorder._polyline_segment(
                    segment,
                    value_getter,
                    x_coordinate,
                    y_coordinate,
                    color,
                    dash=dash,
                    width=width,
                )
            )
        return lines

    @staticmethod
    def _polyline_segment(
        samples,
        value_getter,
        x_coordinate,
        y_coordinate,
        color,
        *,
        dash,
        width,
    ) -> list[str]:
        points = []
        for sample in samples:
            value = float(value_getter(sample))
            if math.isfinite(sample.time_s) and math.isfinite(value):
                points.append(
                    "{:.1f},{:.1f}".format(
                        x_coordinate(sample.time_s), y_coordinate(value)
                    )
                )
        if not points:
            return []
        dash_attribute = (
            ' stroke-dasharray="{}"'.format(dash) if dash is not None else ""
        )
        return [
            '<polyline points="{}" fill="none" stroke="{}" '
            'stroke-width="{:.1f}" stroke-linejoin="round"{} />'.format(
                " ".join(points), color, width, dash_attribute
            )
        ]

    @staticmethod
    def _phase_label(phase: str) -> str:
        replacements = (
            ("failure-return-captured-prone", "failure return"),
            ("failure-hold-captured-prone", "failure hold"),
            ("return-captured-prone", "return prone"),
            ("hold-captured-prone", "hold prone"),
            ("hold-return-standard", "hold standard"),
            ("return-standard", "return standard"),
            ("startup-standard", "startup"),
            ("transition-", "move "),
            ("hold-", "hold "),
        )
        label = phase
        for source, target in replacements:
            label = label.replace(source, target)
        return label

    def _phase_background(
        self,
        x_coordinate,
        top: float,
        bottom: float,
        *,
        labels: bool = False,
    ) -> list[str]:
        result: list[str] = []
        spans = self._phase_spans()
        for index, (phase, start, end) in enumerate(spans):
            x_start = x_coordinate(start)
            x_end = x_coordinate(end)
            if index % 2:
                result.append(
                    '<rect x="{:.1f}" y="{:.1f}" width="{:.1f}" '
                    'height="{:.1f}" fill="#f4f7fb" />'.format(
                        x_start, top, max(0.0, x_end - x_start), bottom - top
                    )
                )
            result.append(
                '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
                'stroke="#c7c7c7" stroke-width="0.8" />'.format(
                    x_start, top, bottom
                )
            )
            if labels:
                midpoint = (x_start + x_end) / 2.0
                result.append(
                    '<text x="{:.1f}" y="{:.1f}" text-anchor="end" '
                    'font-size="9" transform="rotate(-35 {:.1f} {:.1f})">{}</text>'.format(
                        midpoint,
                        top - 6.0,
                        midpoint,
                        top - 6.0,
                        escape(self._phase_label(phase)),
                    )
                )
        return result

    def _event_markers(
        self,
        x_coordinate,
        top: float,
        bottom: float,
        *,
        labels: bool = False,
    ) -> list[str]:
        result: list[str] = []
        seen = set()
        for sample in self.samples:
            if sample.event is None:
                continue
            key = (round(sample.time_s, 9), sample.event)
            if key in seen:
                continue
            seen.add(key)
            x = x_coordinate(sample.time_s)
            result.append(
                '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
                'stroke="#b71c1c" stroke-width="2.0" />'.format(x, top, bottom)
            )
            if labels:
                result.append(
                    '<text x="{:.1f}" y="{:.1f}" text-anchor="end" '
                    'font-size="10" font-weight="bold" fill="#b71c1c" '
                    'transform="rotate(-90 {:.1f} {:.1f})">{}</text>'.format(
                        x - 4.0,
                        bottom - 5.0,
                        x - 4.0,
                        bottom - 5.0,
                        escape(sample.event),
                    )
                )
        return result

    @staticmethod
    def _axes(
        left: float,
        right: float,
        top: float,
        bottom: float,
        x_max: float,
        y_min: float,
        y_max: float,
        *,
        x_labels: bool,
    ) -> list[str]:
        result = [
            '<rect x="{:.1f}" y="{:.1f}" width="{:.1f}" height="{:.1f}" '
            'fill="none" stroke="#999" />'.format(
                left, top, right - left, bottom - top
            )
        ]
        for tick in range(6):
            fraction = tick / 5.0
            x = left + (right - left) * fraction
            timestamp = x_max * fraction
            result.append(
                '<line x1="{0:.1f}" y1="{1:.1f}" x2="{0:.1f}" y2="{2:.1f}" '
                'stroke="#e5e5e5" />'.format(x, top, bottom)
            )
            if x_labels:
                result.append(
                    '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                    'font-size="10">{:.1f}</text>'.format(x, bottom + 17.0, timestamp)
                )
        for tick in range(5):
            fraction = tick / 4.0
            y = bottom - (bottom - top) * fraction
            value = y_min + (y_max - y_min) * fraction
            result.append(
                '<line x1="{0:.1f}" y1="{2:.1f}" x2="{1:.1f}" y2="{2:.1f}" '
                'stroke="#e5e5e5" />'.format(left, right, y)
            )
            result.append(
                '<text x="{:.1f}" y="{:.1f}" text-anchor="end" '
                'font-size="10">{:.2f}</text>'.format(left - 6.0, y + 3.0, value)
            )
        return result

    def _joint_tracking_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        x_max = max(self.samples[-1].time_s, 1.0e-6)
        width, height = 1600, 1180
        left, right, top, bottom = 75.0, 30.0, 100.0, 55.0
        column_gap, row_gap = 24.0, 28.0
        panel_width = (width - left - right - 2.0 * column_gap) / 3.0
        panel_height = (height - top - bottom - 3.0 * row_gap) / 4.0
        svg = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
            'viewBox="0 0 {} {}">'.format(width, height, width, height),
            '<rect width="100%" height="100%" fill="white" />',
            '<style>text { font-family: sans-serif; fill: #222; }</style>',
            '<text x="800" y="28" text-anchor="middle" font-size="20" '
            'font-weight="bold">{}</text>'.format(
                escape(stem + " {} joint tracking".format(self.controller_label))
            ),
            '<text x="800" y="50" text-anchor="middle" font-size="13">'
            '{} q reference vs MuJoCo measured q; phase bands are shared '
            'with {}</text>'.format(
                escape(self.controller_label),
                escape(self.companion_plot_label),
            ),
            '<line x1="1180" y1="72" x2="1225" y2="72" stroke="#d62728" '
            'stroke-width="2" /><text x="1232" y="76" font-size="12">'
            '{} target</text>'.format(escape(self.controller_label)),
            '<line x1="1370" y1="72" x2="1415" y2="72" stroke="#1f77b4" '
            'stroke-width="2" /><text x="1422" y="76" font-size="12">measured</text>',
        ]
        for joint_index, joint_name in enumerate(self.joint_names):
            row, column = divmod(joint_index, 3)
            panel_x = left + column * (panel_width + column_gap)
            panel_y = top + row * (panel_height + row_gap)
            plot_left = panel_x + 54.0
            plot_right = panel_x + panel_width - 12.0
            plot_top = panel_y + 25.0
            plot_bottom = panel_y + panel_height - 34.0
            q_values = [
                value
                for sample in samples
                for value in (sample.q_ref[joint_index], sample.q_measured[joint_index])
                if math.isfinite(value)
            ]
            y_min = min(q_values) if q_values else -1.0
            y_max = max(q_values) if q_values else 1.0
            padding = max(0.05, (y_max - y_min) * 0.08)
            y_min -= padding
            y_max += padding

            def x_coordinate(value):
                return plot_left + (plot_right - plot_left) * value / x_max

            def y_coordinate(value):
                return plot_bottom - (plot_bottom - plot_top) * (
                    value - y_min
                ) / (y_max - y_min)

            svg.extend(self._phase_background(x_coordinate, plot_top, plot_bottom))
            svg.extend(
                self._axes(
                    plot_left,
                    plot_right,
                    plot_top,
                    plot_bottom,
                    x_max,
                    y_min,
                    y_max,
                    x_labels=row == 3,
                )
            )
            svg.append(
                '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                'font-size="13" font-weight="bold">{}</text>'.format(
                    panel_x + panel_width / 2.0,
                    panel_y + 17.0,
                    escape(joint_name),
                )
            )
            svg.extend(
                self._polyline(
                    samples,
                    lambda item, index=joint_index: item.q_ref[index],
                    x_coordinate,
                    y_coordinate,
                    "#d62728",
                    width=1.7,
                )
            )
            svg.extend(self._event_markers(x_coordinate, plot_top, plot_bottom))
            svg.extend(
                self._polyline(
                    samples,
                    lambda item, index=joint_index: item.q_measured[index],
                    x_coordinate,
                    y_coordinate,
                    "#1f77b4",
                    width=1.7,
                )
            )
            if row == 3:
                svg.append(
                    '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                    'font-size="11">Time [s]</text>'.format(
                        panel_x + panel_width / 2.0, panel_y + panel_height - 4.0
                    )
                )
            if column == 0:
                label_x = panel_x + 11.0
                label_y = panel_y + panel_height / 2.0
                svg.append(
                    '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                    'font-size="11" transform="rotate(-90 {:.1f} {:.1f})">'
                    'Angle [rad]</text>'.format(label_x, label_y, label_x, label_y)
                )
        svg.append('</svg>')
        return "\n".join(svg) + "\n"

    def _governor_svg(self, stem: str) -> str:
        samples = self._selected_samples()
        x_max = max(self.samples[-1].time_s, 1.0e-6)
        width, height = 1600, 1120
        left, right, top, bottom = 110.0, 65.0, 145.0, 70.0
        row_gap = 42.0
        panel_height = (height - top - bottom - 3.0 * row_gap) / 4.0
        slow_time = 0.0
        stopped_time = 0.0
        for current, following in zip(self.samples, self.samples[1:]):
            dt = max(0.0, following.time_s - current.time_s)
            if current.speed_scale < 1.0 - 1.0e-9:
                slow_time += dt
            if current.speed_scale <= 1.0e-9:
                stopped_time += dt
        peak = max(self.samples, key=lambda item: item.tracking_ratio)
        svg = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<svg xmlns="http://www.w3.org/2000/svg" width="{}" height="{}" '
            'viewBox="0 0 {} {}">'.format(width, height, width, height),
            '<rect width="100%" height="100%" fill="white" />',
            '<style>text { font-family: sans-serif; fill: #222; }</style>',
            '<text x="800" y="28" text-anchor="middle" font-size="20" '
            'font-weight="bold">{}</text>'.format(
                escape(stem + " adaptive reference governor")
            ),
            '<text x="800" y="52" text-anchor="middle" font-size="13">'
            'slowdown {:.3f} s; stopped {:.3f} s; peak tracking ratio {:.3f} ({})'
            '</text>'.format(
                slow_time,
                stopped_time,
                peak.tracking_ratio,
                escape(peak.limiting_joint),
            ),
            '<text x="800" y="73" text-anchor="middle" font-size="12">'
            'solid blue: adaptive; dashed gray: time-only nominal; phase progress '
            'resets at each phase</text>',
        ]
        panel_specs = (
            (
                "Phase progress",
                0.0,
                1.05,
                (
                    (lambda item: item.nominal_progress, "#777", "7 5", 1.7),
                    (lambda item: item.progress, "#1f77b4", None, 2.4),
                ),
                (),
            ),
            (
                "Governor speed scale",
                0.0,
                1.05,
                ((lambda item: item.speed_scale, "#2ca02c", None, 2.2),),
                ((1.0, "#888", "scheduled speed"),),
            ),
            (
                "Tracking envelope ratio",
                0.0,
                max(1.05, peak.tracking_ratio * 1.10),
                ((lambda item: item.tracking_ratio, "#1f77b4", None, 2.2),),
                (
                    (0.50, "#e6a700", "slowdown"),
                    (0.90, "#d95f02", "progress stop"),
                    (1.00, "#c62828", "controlled return"),
                ),
            ),
            (
                "tau_est / model torque range",
                0.0,
                max(1.05, max(item.torque_ratio for item in self.samples) * 1.10),
                ((lambda item: item.torque_ratio, "#9467bd", None, 2.2),),
                (
                    (0.60, "#e6a700", "warning"),
                    (0.75, "#d95f02", "progress stop"),
                    (0.85, "#c62828", "controlled return"),
                    (1.00, "#7f0000", "immediate error"),
                ),
            ),
        )
        for row, (label, y_min, y_max, lines, thresholds) in enumerate(panel_specs):
            panel_top = top + row * (panel_height + row_gap)
            panel_bottom = panel_top + panel_height

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
                    x_labels=row == 3,
                )
            )
            label_x = 24.0
            label_y = panel_top + panel_height / 2.0
            svg.append(
                '<text x="{:.1f}" y="{:.1f}" text-anchor="middle" '
                'font-size="12" font-weight="bold" '
                'transform="rotate(-90 {:.1f} {:.1f})">{}</text>'.format(
                    label_x, label_y, label_x, label_y, escape(label)
                )
            )
            for threshold, color, threshold_label in thresholds:
                if y_min <= threshold <= y_max:
                    y = y_coordinate(threshold)
                    svg.append(
                        '<line x1="{:.1f}" y1="{:.1f}" x2="{:.1f}" y2="{:.1f}" '
                        'stroke="{}" stroke-width="1.1" stroke-dasharray="5 4" />'.format(
                            left, y, width - right, y, color
                        )
                    )
                    svg.append(
                        '<text x="{:.1f}" y="{:.1f}" text-anchor="end" '
                        'font-size="10" fill="{}">{:.2f} {}</text>'.format(
                            width - right - 4.0,
                            y - 4.0,
                            color,
                            threshold,
                            escape(threshold_label),
                        )
                    )
            for getter, color, dash, line_width in lines:
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
                '<text x="800" y="1085" text-anchor="middle" font-size="12">'
                'Time [s] from captured-prone control start</text>',
                '<text x="800" y="1106" text-anchor="middle" font-size="11" '
                'fill="#555">Intervention evidence only; causal scripted/adaptive '
                'performance comparison requires paired runs.</text>',
                '</svg>',
            ]
        )
        return "\n".join(svg) + "\n"
