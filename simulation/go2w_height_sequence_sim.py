#!/usr/bin/env python3
"""Run a Go2W low-level height sequence, then hold standard in MuJoCo.

This script is simulation-only: DDS is fixed to domain 0 on the loopback
interface, and the script starts and owns its MuJoCo child process.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from html import escape
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_UNITREE_MUJOCO_ROOT = WORKSPACE_ROOT.parent / "unitree_mujoco"
UNITREE_MUJOCO_ROOT = Path(
    os.environ.get("UNITREE_MUJOCO_ROOT", DEFAULT_UNITREE_MUJOCO_ROOT)
).expanduser().resolve()
VENV_PYTHON = Path(
    os.environ.get(
        "UNITREE_MUJOCO_PYTHON",
        UNITREE_MUJOCO_ROOT / "simulate_python" / ".venv" / "bin" / "python",
    )
).expanduser().absolute()
VENV_ROOT = VENV_PYTHON.parent.parent

if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

import go2w_gesture_real as hardware_gesture  # noqa: E402


ChannelFactoryInitialize = None
ChannelPublisher = None
ChannelSubscriber = None
LowCmd_ = None
LowState_ = None
SportModeState_ = None
CRC = None
unitree_go_msg_dds__LowCmd_ = None


def reexec_with_simulator_python(script_path: str | Path | None = None) -> None:
    """Run the controller with the SDK environment supplied by unitree_mujoco."""
    if Path(sys.prefix).resolve() == VENV_ROOT.resolve():
        return
    if not VENV_PYTHON.is_file():
        raise RuntimeError(
            "simulation Python not found: {} (set UNITREE_MUJOCO_PYTHON)".format(
                VENV_PYTHON
            )
        )
    target = Path(script_path).resolve() if script_path else Path(__file__).resolve()
    os.execv(
        str(VENV_PYTHON),
        [str(VENV_PYTHON), str(target), *sys.argv[1:]],
    )


def load_unitree_sdk() -> None:
    global ChannelFactoryInitialize
    global ChannelPublisher
    global ChannelSubscriber
    global LowCmd_
    global LowState_
    global SportModeState_
    global CRC
    global unitree_go_msg_dds__LowCmd_

    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize as _ChannelFactoryInitialize,
            ChannelPublisher as _ChannelPublisher,
            ChannelSubscriber as _ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__LowCmd_ as _LowCmdDefault,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
            LowCmd_ as _LowCmd,
            LowState_ as _LowState,
            SportModeState_ as _SportModeState,
        )
        from unitree_sdk2py.utils.crc import CRC as _CRC
    except ImportError as error:
        raise RuntimeError(
            "unitree_sdk2py is unavailable in {}: {}".format(
                Path(sys.executable).resolve(), error
            )
        ) from error

    ChannelFactoryInitialize = _ChannelFactoryInitialize
    ChannelPublisher = _ChannelPublisher
    ChannelSubscriber = _ChannelSubscriber
    LowCmd_ = _LowCmd
    LowState_ = _LowState
    SportModeState_ = _SportModeState
    CRC = _CRC
    unitree_go_msg_dds__LowCmd_ = _LowCmdDefault


DDS_DOMAIN = 0
DDS_INTERFACE = "lo"
CONTROL_PERIOD_S = 0.002  # 500 Hz
STANDARD_TRANSITION_S = 2.0
HEIGHT_TRANSITION_S = 1.0
STANDARD_HOLD_S = 2.0
HEIGHT_HOLD_S = 0.5
HEIGHT_CYCLES = 3
STATE_TIMEOUT_S = 0.25
FINAL_HOLD_PLOT_S = 3.0

SIMULATOR = UNITREE_MUJOCO_ROOT / "simulate" / "build" / "unitree_mujoco"
MODEL_DIR = UNITREE_MUJOCO_ROOT / "unitree_robots" / "go2w"
MODEL_XML = MODEL_DIR / "go2w.xml"
MODEL_ASSETS = MODEL_DIR / "assets"
FLAT_SCENE = WORKSPACE_ROOT / "simulation" / "scenes" / "go2w_flat.xml"
PLOT_OUTPUT_DIR = WORKSPACE_ROOT / "runs" / "mujoco"

KP = [60.0, 80.0, 80.0] * 4
KD = [5.0, 4.0, 4.0] * 4
JOINT_NAMES = tuple(
    f"{leg} {joint}"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)


# The real-hardware controller is authoritative for the joint targets. Keeping
# the simulator imports here makes a target change visible to both paths.
STANDARD = list(hardware_gesture.STANDARD)
LOW = list(hardware_gesture.LOW)
HIGH = list(hardware_gesture.HIGH)


def runtime_requirements() -> tuple[tuple[str, Path, str], ...]:
    return (
        ("unitree_mujoco root", UNITREE_MUJOCO_ROOT, "directory"),
        ("simulator binary", SIMULATOR, "executable"),
        ("simulation Python", VENV_PYTHON, "executable"),
        ("Go2W model", MODEL_XML, "file"),
        ("Go2W assets", MODEL_ASSETS, "directory"),
        ("workspace flat scene", FLAT_SCENE, "file"),
    )


def requirement_is_ready(path: Path, kind: str) -> bool:
    if kind == "directory":
        return path.is_dir()
    if kind == "executable":
        return path.is_file() and os.access(path, os.X_OK)
    return path.is_file()


def doctor() -> bool:
    ready = True
    print(f"UNITREE_MUJOCO_ROOT={UNITREE_MUJOCO_ROOT}")
    print(f"UNITREE_MUJOCO_PYTHON={VENV_PYTHON}")
    for label, path, kind in runtime_requirements():
        status = "ok" if requirement_is_ready(path, kind) else "missing"
        print(f"[{status}] {label}: {path}")
        ready = ready and status == "ok"

    if requirement_is_ready(VENV_PYTHON, "executable"):
        sdk_check = subprocess.run(
            [
                str(VENV_PYTHON),
                "-c",
                "from unitree_sdk2py.core.channel import ChannelFactoryInitialize",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        sdk_ready = sdk_check.returncode == 0
        print(
            "[{}] unitree_sdk2py import via simulation Python".format(
                "ok" if sdk_ready else "missing"
            )
        )
        if not sdk_ready and sdk_check.stderr.strip():
            print(sdk_check.stderr.strip(), file=sys.stderr)
        ready = ready and sdk_ready
    return ready


class SequenceController:
    def __init__(
        self,
        plot_stem: str = "go2w_height_sequence_sim",
        *,
        save_plot: bool = False,
    ) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._low_state: LowState_ | None = None
        self._last_low_state_time: float | None = None
        self._base_height: float | None = None
        self._lowcmd_seen_before_start = False
        self._publishing = False
        self._simulator: subprocess.Popen[bytes] | None = None
        self._scene_workspace: tempfile.TemporaryDirectory[str] | None = None
        self._plot_stem = plot_stem
        self._save_plot = save_plot
        self._tracking_start_time: float | None = None
        self._tracking_times: list[float] = []
        self._tracking_targets: list[list[float]] = []
        self._tracking_actuals: list[list[float]] = []
        self._tracking_complete = False

        self._command = unitree_go_msg_dds__LowCmd_()
        self._crc = CRC()
        self._configure_command()

    def _prepare_flat_scene(self) -> Path:
        for label, path, kind in runtime_requirements():
            if not requirement_is_ready(path, kind):
                raise RuntimeError(f"{label} is unavailable: {path}")

        self._scene_workspace = tempfile.TemporaryDirectory(
            prefix="go2w-lowlevel-gestures-mujoco-"
        )
        scene_dir = Path(self._scene_workspace.name)
        shutil.copy2(FLAT_SCENE, scene_dir / "scene_flat.xml")
        (scene_dir / "go2w.xml").symlink_to(MODEL_XML)
        (scene_dir / "assets").symlink_to(MODEL_ASSETS, target_is_directory=True)
        return scene_dir / "scene_flat.xml"

    def _stop_simulator(self) -> None:
        if self._simulator is not None and self._simulator.poll() is None:
            self._simulator.terminate()
            try:
                self._simulator.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._simulator.kill()
                self._simulator.wait(timeout=1.0)
        if self._scene_workspace is not None:
            self._scene_workspace.cleanup()
            self._scene_workspace = None

    def request_stop(self, _signum=None, _frame=None) -> None:
        self._stop.set()

    def on_low_state(self, message: LowState_) -> None:
        with self._lock:
            self._low_state = message
            self._last_low_state_time = time.monotonic()

    def on_sport_state(self, message: SportModeState_) -> None:
        with self._lock:
            self._base_height = float(message.position[2])

    def on_lowcmd(self, _message: LowCmd_) -> None:
        if not self._publishing:
            self._lowcmd_seen_before_start = True

    def _configure_command(self) -> None:
        self._command.head[0] = 0xFE
        self._command.head[1] = 0xEF
        self._command.level_flag = 0xFF
        self._command.gpio = 0
        for motor in self._command.motor_cmd:
            motor.mode = 0x01
            motor.q = 2.146e9
            motor.kp = 0.0
            motor.dq = 16000.0
            motor.kd = 0.0
            motor.tau = 0.0

    def _set_pose(self, pose: list[float]) -> None:
        for index in range(12):
            motor = self._command.motor_cmd[index]
            motor.mode = 0x01
            motor.q = pose[index]
            motor.kp = KP[index]
            motor.dq = 0.0
            motor.kd = KD[index]
            motor.tau = 0.0

        # Wheels are velocity-damped at zero speed; position gain stays disabled.
        for index in range(12, 16):
            motor = self._command.motor_cmd[index]
            motor.mode = 0x01
            motor.q = 0.0
            motor.kp = 0.0
            motor.dq = 0.0
            motor.kd = 2.0
            motor.tau = 0.0

    def _publish(self, publisher: ChannelPublisher, pose: list[float]) -> None:
        self._set_pose(pose)
        self._command.crc = self._crc.Crc(self._command)
        publisher.Write(self._command)
        self._record_tracking_sample(pose)

    def _record_tracking_sample(self, target: list[float]) -> None:
        if not self._save_plot or self._tracking_complete:
            return
        timestamp = time.monotonic()
        with self._lock:
            if self._low_state is None:
                return
            actual = [
                float(self._low_state.motor_state[index].q) for index in range(12)
            ]
        if self._tracking_start_time is None:
            self._tracking_start_time = timestamp
        self._tracking_times.append(timestamp - self._tracking_start_time)
        self._tracking_targets.append(list(target))
        self._tracking_actuals.append(actual)

    @staticmethod
    def _downsample_indices(sample_count: int, maximum: int = 1800) -> list[int]:
        if sample_count <= maximum:
            return list(range(sample_count))
        step = math.ceil((sample_count - 1) / (maximum - 1))
        indices = list(range(0, sample_count, step))
        if indices[-1] != sample_count - 1:
            indices.append(sample_count - 1)
        return indices

    def _write_tracking_plot(
        self,
        final_hold_start: float,
        plot_end: float,
    ) -> Path:
        if self._tracking_start_time is None or not self._tracking_times:
            raise RuntimeError("no joint tracking samples were recorded")

        self._tracking_complete = True
        final_hold_time = final_hold_start - self._tracking_start_time
        end_time = plot_end - self._tracking_start_time
        included = [
            index
            for index, timestamp in enumerate(self._tracking_times)
            if timestamp <= end_time
        ]
        if not included:
            raise RuntimeError("no joint tracking samples fall inside the plot range")
        selected = self._downsample_indices(len(included))
        indices = [included[index] for index in selected]

        times = [self._tracking_times[index] for index in indices]
        targets = [self._tracking_targets[index] for index in indices]
        actuals = [self._tracking_actuals[index] for index in indices]
        x_max = max(end_time, times[-1], 1e-6)

        width = 1440
        height = 1100
        left = 70.0
        right = 25.0
        top = 85.0
        bottom = 55.0
        column_gap = 22.0
        row_gap = 25.0
        panel_width = (width - left - right - 2.0 * column_gap) / 3.0
        panel_height = (height - top - bottom - 3.0 * row_gap) / 4.0

        svg = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            (
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
                f'height="{height}" viewBox="0 0 {width} {height}">'
            ),
            '<rect width="100%" height="100%" fill="white"/>',
            '<style>text { font-family: sans-serif; fill: #222; }</style>',
            (
                f'<text x="{width / 2:.1f}" y="28" text-anchor="middle" '
                f'font-size="20" font-weight="bold">'
                f'{escape(self._plot_stem)} joint tracking</text>'
            ),
            (
                f'<text x="{width / 2:.1f}" y="50" text-anchor="middle" '
                f'font-size="13">target q vs actual q; final hold + '
                f'{FINAL_HOLD_PLOT_S:.1f} s</text>'
            ),
            '<line x1="1040" y1="68" x2="1080" y2="68" '
            'stroke="#d62728" stroke-width="2"/>',
            '<text x="1087" y="72" font-size="12">target</text>',
            '<line x1="1160" y1="68" x2="1200" y2="68" '
            'stroke="#1f77b4" stroke-width="2"/>',
            '<text x="1207" y="72" font-size="12">actual</text>',
            '<line x1="1270" y1="68" x2="1310" y2="68" '
            'stroke="#666" stroke-width="1.5" stroke-dasharray="5 4"/>',
            '<text x="1317" y="72" font-size="12">final hold</text>',
        ]

        for joint_index, joint_name in enumerate(JOINT_NAMES):
            row = joint_index // 3
            column = joint_index % 3
            panel_x = left + column * (panel_width + column_gap)
            panel_y = top + row * (panel_height + row_gap)
            plot_left = panel_x + 50.0
            plot_right = panel_x + panel_width - 12.0
            plot_top = panel_y + 24.0
            plot_bottom = panel_y + panel_height - 34.0
            plot_width = plot_right - plot_left
            plot_height = plot_bottom - plot_top

            target_values = [pose[joint_index] for pose in targets]
            actual_values = [pose[joint_index] for pose in actuals]
            finite_values = [
                value
                for value in target_values + actual_values
                if math.isfinite(value)
            ]
            if not finite_values:
                y_min, y_max = -1.0, 1.0
            else:
                y_min = min(finite_values)
                y_max = max(finite_values)
                padding = max(0.05, (y_max - y_min) * 0.08)
                y_min -= padding
                y_max += padding

            def x_coordinate(timestamp: float) -> float:
                return plot_left + plot_width * timestamp / x_max

            def y_coordinate(value: float) -> float:
                return plot_bottom - plot_height * (value - y_min) / (y_max - y_min)

            svg.extend(
                [
                    (
                        f'<rect x="{panel_x:.1f}" y="{panel_y:.1f}" '
                        f'width="{panel_width:.1f}" height="{panel_height:.1f}" '
                        'fill="none" stroke="#bbb"/>'
                    ),
                    (
                        f'<text x="{panel_x + panel_width / 2:.1f}" '
                        f'y="{panel_y + 17:.1f}" text-anchor="middle" '
                        f'font-size="13" font-weight="bold">'
                        f'{escape(joint_name)} (joint {joint_index})</text>'
                    ),
                ]
            )

            for tick in range(5):
                fraction = tick / 4.0
                x = plot_left + plot_width * fraction
                timestamp = x_max * fraction
                svg.append(
                    f'<line x1="{x:.1f}" y1="{plot_top:.1f}" x2="{x:.1f}" '
                    f'y2="{plot_bottom:.1f}" stroke="#eee"/>'
                )
                svg.append(
                    f'<text x="{x:.1f}" y="{plot_bottom + 17:.1f}" '
                    f'text-anchor="middle" font-size="10">{timestamp:.1f}</text>'
                )

                y = plot_bottom - plot_height * fraction
                value = y_min + (y_max - y_min) * fraction
                svg.append(
                    f'<line x1="{plot_left:.1f}" y1="{y:.1f}" '
                    f'x2="{plot_right:.1f}" y2="{y:.1f}" stroke="#eee"/>'
                )
                svg.append(
                    f'<text x="{plot_left - 5:.1f}" y="{y + 3:.1f}" '
                    f'text-anchor="end" font-size="10">{value:.2f}</text>'
                )

            hold_x = x_coordinate(final_hold_time)
            svg.append(
                f'<line x1="{hold_x:.1f}" y1="{plot_top:.1f}" '
                f'x2="{hold_x:.1f}" y2="{plot_bottom:.1f}" '
                'stroke="#666" stroke-width="1.2" stroke-dasharray="5 4"/>'
            )

            for values, color in (
                (target_values, "#d62728"),
                (actual_values, "#1f77b4"),
            ):
                points = " ".join(
                    f"{x_coordinate(timestamp):.1f},{y_coordinate(value):.1f}"
                    for timestamp, value in zip(times, values)
                    if math.isfinite(value)
                )
                svg.append(
                    f'<polyline points="{points}" fill="none" stroke="{color}" '
                    'stroke-width="1.5" stroke-linejoin="round"/>'
                )

            if row == 3:
                svg.append(
                    f'<text x="{panel_x + panel_width / 2:.1f}" '
                    f'y="{panel_y + panel_height - 5:.1f}" text-anchor="middle" '
                    'font-size="11">Time [s]</text>'
                )
            if column == 0:
                svg.append(
                    f'<text x="{panel_x + 11:.1f}" '
                    f'y="{panel_y + panel_height / 2:.1f}" '
                    'text-anchor="middle" font-size="11" '
                    f'transform="rotate(-90 {panel_x + 11:.1f} '
                    f'{panel_y + panel_height / 2:.1f})">Angle [rad]</text>'
                )

        svg.append("</svg>")
        PLOT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        output_path = PLOT_OUTPUT_DIR / (
            f"{self._plot_stem}_joint_tracking_{timestamp}.svg"
        )
        output_path.write_text("\n".join(svg) + "\n", encoding="utf-8")
        print(
            f"joint tracking plot saved ({len(included)} samples): {output_path}",
            flush=True,
        )
        return output_path

    def _run_final_hold(
        self,
        publisher: ChannelPublisher,
        pose: list[float],
        final_hold_start: float,
    ) -> None:
        plot_end = final_hold_start + FINAL_HOLD_PLOT_S
        next_tick = time.monotonic()
        while not self._stop.is_set():
            self._check_runtime()
            self._publish(publisher, pose)
            now = time.monotonic()
            if self._save_plot and not self._tracking_complete and now >= plot_end:
                self._write_tracking_plot(final_hold_start, plot_end)

            next_tick += CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -CONTROL_PERIOD_S:
                # Do not emit catch-up bursts after an OS scheduling delay.
                next_tick = time.monotonic()

    @staticmethod
    def _interpolate(source: list[float], target: list[float], alpha: float) -> list[float]:
        alpha = max(0.0, min(1.0, alpha))
        blend = alpha * alpha * (3.0 - 2.0 * alpha)
        return [source[i] + (target[i] - source[i]) * blend for i in range(12)]

    def _check_runtime(self) -> None:
        if self._stop.is_set():
            raise InterruptedError
        if self._simulator is None or self._simulator.poll() is not None:
            raise RuntimeError("MuJoCo exited before the sequence completed")
        with self._lock:
            last_state_time = self._last_low_state_time
        if last_state_time is None or time.monotonic() - last_state_time > STATE_TIMEOUT_S:
            raise RuntimeError("rt/lowstate timed out")

    def _run_for(
        self,
        publisher: ChannelPublisher,
        duration_s: float,
        pose_at,
    ) -> list[float]:
        start = time.monotonic()
        next_tick = start
        last_pose = pose_at(0.0)
        while True:
            self._check_runtime()
            now = time.monotonic()
            elapsed = now - start
            alpha = min(1.0, elapsed / duration_s)
            last_pose = pose_at(alpha)
            self._publish(publisher, last_pose)
            if elapsed >= duration_s:
                return last_pose

            next_tick += CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -CONTROL_PERIOD_S:
                # Do not emit catch-up bursts after an OS scheduling delay.
                next_tick = time.monotonic()

    def _transition(
        self,
        publisher: ChannelPublisher,
        name: str,
        source: list[float],
        target: list[float],
        duration_s: float,
    ) -> list[float]:
        print(f"transition -> {name} ({duration_s:.1f} s)", flush=True)
        return self._run_for(
            publisher,
            duration_s,
            lambda alpha: self._interpolate(source, target, alpha),
        )

    def _hold(
        self,
        publisher: ChannelPublisher,
        name: str,
        pose: list[float],
        duration_s: float,
    ) -> None:
        with self._lock:
            base_height = self._base_height
        height_text = "unknown" if base_height is None else f"{base_height:.3f} m"
        print(f"hold {name} ({duration_s:.1f} s), base_z={height_text}", flush=True)
        self._run_for(publisher, duration_s, lambda _alpha: pose)

    def _wait_for_first_state(self, timeout_s: float = 10.0) -> list[float]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._simulator is not None and self._simulator.poll() is not None:
                raise RuntimeError("MuJoCo exited during startup")
            with self._lock:
                if self._low_state is not None:
                    return [float(self._low_state.motor_state[i].q) for i in range(12)]
            time.sleep(0.01)
        raise RuntimeError("timed out waiting for rt/lowstate")

    def run(self) -> None:
        flat_scene = self._prepare_flat_scene()

        ChannelFactoryInitialize(DDS_DOMAIN, DDS_INTERFACE)
        low_state_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        sport_state_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        lowcmd_subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        low_state_subscriber.Init(self.on_low_state, 10)
        sport_state_subscriber.Init(self.on_sport_state, 10)
        lowcmd_subscriber.Init(self.on_lowcmd, 10)

        # Refuse to create a second command owner or a second simulator on domain 0.
        time.sleep(0.25)
        with self._lock:
            simulator_already_present = self._last_low_state_time is not None
        if self._lowcmd_seen_before_start or simulator_already_present:
            raise RuntimeError(
                "DDS activity already exists on domain 0/lo; stop the existing "
                "controller and simulator before running this script"
            )

        simulator_command = [
            str(SIMULATOR),
            "-i",
            str(DDS_DOMAIN),
            "-n",
            DDS_INTERFACE,
            "-r",
            "go2w",
            "-s",
            str(flat_scene),
        ]
        print("starting flat Go2W MuJoCo GUI", flush=True)
        self._simulator = subprocess.Popen(
            simulator_command,
            cwd=UNITREE_MUJOCO_ROOT,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

        publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        publisher.Init()

        try:
            current_pose = self._wait_for_first_state()
            self._publishing = True

            current_pose = self._transition(
                publisher,
                "standard",
                current_pose,
                STANDARD,
                STANDARD_TRANSITION_S,
            )
            self._hold(publisher, "standard", STANDARD, STANDARD_HOLD_S)

            for cycle in range(1, HEIGHT_CYCLES + 1):
                print(f"height cycle {cycle}/{HEIGHT_CYCLES}", flush=True)
                current_pose = self._transition(
                    publisher,
                    "low",
                    current_pose,
                    LOW,
                    HEIGHT_TRANSITION_S,
                )
                self._hold(publisher, "low", LOW, HEIGHT_HOLD_S)
                current_pose = self._transition(
                    publisher,
                    "high",
                    current_pose,
                    HIGH,
                    HEIGHT_TRANSITION_S,
                )
                self._hold(publisher, "high", HIGH, HEIGHT_HOLD_S)

            current_pose = self._transition(
                publisher,
                "standard",
                current_pose,
                STANDARD,
                STANDARD_TRANSITION_S,
            )
            final_hold_start = time.monotonic()
            self._hold(publisher, "standard", STANDARD, STANDARD_HOLD_S)

            with self._lock:
                base_height = self._base_height
            height_text = "unknown" if base_height is None else f"{base_height:.3f} m"
            if self._save_plot:
                plot_status = (
                    f"The joint plot will be written after {FINAL_HOLD_PLOT_S:.1f} s "
                    "of final hold; press Ctrl+C after that to close MuJoCo."
                )
            else:
                plot_status = (
                    "Joint plot recording is disabled; press Ctrl+C to close MuJoCo."
                )
            print(
                "sequence complete; holding standard pose at 500 Hz "
                f"(base_z={height_text}). {plot_status}",
                flush=True,
            )
            self._run_final_hold(publisher, STANDARD, final_hold_start)
        except InterruptedError:
            print("stop requested", flush=True)
        finally:
            self._publishing = False
            self._stop_simulator()


def print_plan(save_plot: bool = False) -> None:
    print("joint order: FR, FL, RR, RL; each leg is hip, thigh, calf")
    print(f"STANDARD = {STANDARD}")
    print(f"LOW      = {LOW}")
    print(f"HIGH     = {HIGH}")
    print("Go2W simulated height sequence:")
    print(f"  startup -> standard: {STANDARD_TRANSITION_S:.1f} s")
    print(f"  hold standard: {STANDARD_HOLD_S:.1f} s")
    print(f"  repeat {HEIGHT_CYCLES} times:")
    print(
        f"    -> low/high: {HEIGHT_TRANSITION_S:.1f} s transition, "
        f"{HEIGHT_HOLD_S:.1f} s hold"
    )
    print(f"  high -> standard: {STANDARD_TRANSITION_S:.1f} s")
    print("  hold standard at 500 Hz until Ctrl+C")
    print(
        "  joint tracking plot: "
        + ("save after final hold" if save_plot else "disabled (use --save-plot)")
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--describe",
        action="store_true",
        help="print joint targets and sequence without requiring MuJoCo",
    )
    parser.add_argument(
        "--doctor",
        action="store_true",
        help="check the external unitree_mujoco runtime without launching it",
    )
    parser.add_argument(
        "--save-plot",
        action="store_true",
        help="record target/actual joint angles and save an SVG after final hold",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.doctor:
        return 0 if doctor() else 1
    if args.describe:
        print_plan(args.save_plot)
        return 0

    try:
        reexec_with_simulator_python()
        load_unitree_sdk()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1

    print_plan(args.save_plot)
    controller = SequenceController(save_plot=args.save_plot)
    signal.signal(signal.SIGINT, controller.request_stop)
    signal.signal(signal.SIGTERM, controller.request_stop)
    try:
        controller.run()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
