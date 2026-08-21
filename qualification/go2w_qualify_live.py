#!/usr/bin/env python3
"""Fail-closed Jetson software and physical qualification runner.

The runner never sends LowCmd itself.  It executes the read-only preflight
before asking for a case-specific physical confirmation, and invokes a live
Make target only when ``--live`` was explicitly supplied and every prior stage
passed.  It does not retry, clean, stash, reset, or select a diagnostic
no-tracking-stop profile.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Iterable, Optional


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = WORKSPACE_ROOT / "runs" / "qualification"
EXPECTED_BRANCH = "feat/adaptive-wbc-gestures"
EXPECTED_ARCHITECTURE = "aarch64"
DEFAULT_INTERFACE = "eth0"
DEFAULT_EXPECTED_IP = "192.168.123.18"
CONTROLLERS = ("adaptive", "wbc")
GESTURES = ("height", "roll")

LIVE_CONFIRMATIONS = {
    ("adaptive", "height"): "RUN ADAPTIVE HEIGHT AT 100 PERCENT FOR 3 CYCLES",
    ("adaptive", "roll"): "RUN ADAPTIVE ROLL AT 100 PERCENT FOR 3 CYCLES",
    ("wbc", "height"): "RUN WBC HEIGHT AT 100 PERCENT FOR 3 CYCLES",
    ("wbc", "roll"): "RUN WBC ROLL AT 100 PERCENT FOR 3 CYCLES",
}


class QualificationFailure(RuntimeError):
    pass


class QualificationRun:
    def __init__(self, output_root: Path, controller: str, gesture: str) -> None:
        created = datetime.now().astimezone()
        stem = "{}_{}_{}".format(
            created.strftime("%Y%m%dT%H%M%S_%f%z"), controller, gesture
        )
        self.directory = output_root / stem
        self.directory.mkdir(parents=True, exist_ok=False)
        self.created_at = created
        self.controller = controller
        self.gesture = gesture
        self.log_path = self.directory / "terminal.log"
        self.log_file = self.log_path.open("w", encoding="utf-8", buffering=1)
        self.commands = []
        self.stages = []

    def emit(self, text: str, *, error: bool = False) -> None:
        line = str(text)
        print(line, file=sys.stderr if error else sys.stdout, flush=True)
        self.log_file.write(line + "\n")

    def run_command(
        self,
        stage: str,
        command: Iterable[str],
        *,
        environment: Optional[dict[str, str]] = None,
        interactive_output: bool = False,
    ) -> None:
        argv = [str(value) for value in command]
        self.commands.append(argv)
        self.emit("[stage:{}] $ {}".format(stage, " ".join(argv)))
        started = datetime.now().astimezone()
        process = subprocess.Popen(
            argv,
            cwd=str(WORKSPACE_ROOT),
            env=environment,
            stdin=None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        if interactive_output:
            # The hardware controller's input() prompts intentionally have no
            # trailing newline.  Character-wise mirroring keeps those prompts
            # visible while preserving the exact live transcript in the run
            # artifact.  stdin remains inherited from the operator's TTY.
            while True:
                character = process.stdout.read(1)
                if character == "":
                    break
                sys.stdout.write(character)
                sys.stdout.flush()
                self.log_file.write(character)
                self.log_file.flush()
        else:
            for line in process.stdout:
                self.emit(line.rstrip("\n"))
        return_code = int(process.wait())
        self.stages.append(
            {
                "name": stage,
                "command": argv,
                "started_at": started.isoformat(),
                "finished_at": datetime.now().astimezone().isoformat(),
                "exit_code": return_code,
            }
        )
        if return_code != 0:
            raise QualificationFailure(
                "stage {!r} failed with exit code {}".format(stage, return_code)
            )

    def close(self) -> None:
        if not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()


def capture(command: Iterable[str]) -> str:
    completed = subprocess.run(
        [str(value) for value in command],
        cwd=str(WORKSPACE_ROOT),
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return completed.stdout.strip()


def check_repository(expected_sha: Optional[str], expected_branch: str) -> tuple[str, str]:
    status = capture(("git", "status", "--porcelain", "--untracked-files=normal"))
    if status:
        raise QualificationFailure(
            "worktree is dirty; refusing to stash, reset, delete, or continue:\n{}".format(
                status
            )
        )
    branch = capture(("git", "branch", "--show-current"))
    sha = capture(("git", "rev-parse", "HEAD"))
    if branch != expected_branch:
        raise QualificationFailure(
            "branch mismatch: current {!r}, expected {!r}".format(
                branch, expected_branch
            )
        )
    if expected_sha and sha != expected_sha:
        raise QualificationFailure(
            "Git SHA mismatch: current {}, expected {}".format(sha, expected_sha)
        )
    if expected_sha:
        remote_ref = "refs/remotes/origin/{}".format(expected_branch)
        try:
            remote_sha = capture(("git", "rev-parse", remote_ref))
        except subprocess.CalledProcessError as error:
            raise QualificationFailure(
                "remote-tracking ref {} is unavailable; fetch without modifying dirty work first".format(
                    remote_ref
                )
            ) from error
        if remote_sha != expected_sha:
            raise QualificationFailure(
                "remote-tracking SHA mismatch: {} is {}, expected {}".format(
                    remote_ref, remote_sha, expected_sha
                )
            )
    return branch, sha


def check_platform_and_network(interface: str, expected_ip: str) -> str:
    architecture = platform.machine()
    if architecture != EXPECTED_ARCHITECTURE:
        raise QualificationFailure(
            "architecture mismatch: current {!r}, expected {!r}".format(
                architecture, EXPECTED_ARCHITECTURE
            )
        )
    import go2w_gesture_real as base

    actual_ip = base.interface_ipv4(interface)
    if actual_ip != expected_ip:
        raise QualificationFailure(
            "NIC/IP mismatch: {} has {}, expected {}".format(
                interface, actual_ip, expected_ip
            )
        )
    return actual_ip


def relative_controller_log_dir(run_directory: Path) -> str:
    try:
        relative = run_directory.resolve().relative_to(WORKSPACE_ROOT.resolve())
    except ValueError as error:
        raise QualificationFailure(
            "live output must be inside the repository so Compose can persist it"
        ) from error
    return str(relative / "controller")


def physical_checklist(run: QualificationRun) -> None:
    run.emit("実機チェックリスト（すべて確認してから入力）:")
    run.emit("  1. Go2Wの電源が入り、belly-downで静止している")
    run.emit("  2. 4輪を固定し、腹部クリアランス用の支持具を配置した")
    run.emit("  3. spotterが付き、ハードウェアE-stopを即時操作できる")
    run.emit("  4. rt/lowcmdの単独所有を確認し、他の運動制御を停止した")
    run.emit("  5. 異常時は残りのliveを中止し、再試行しない")


def write_hash_manifest(directory: Path) -> None:
    entries = []
    for path in sorted(directory.rglob("*")):
        if not path.is_file() or path.name == "SHA256SUMS":
            continue
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        entries.append("{}  {}".format(digest, path.relative_to(directory)))
    (directory / "SHA256SUMS").write_text(
        "\n".join(entries) + ("\n" if entries else ""), encoding="utf-8"
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Fail-closed Go2W Jetson qualification runner"
    )
    parser.add_argument("--controller", choices=CONTROLLERS, required=True)
    parser.add_argument("--gesture", choices=GESTURES, required=True)
    parser.add_argument("--expected-sha")
    parser.add_argument("--expected-branch", default=EXPECTED_BRANCH)
    parser.add_argument("--interface", default=DEFAULT_INTERFACE)
    parser.add_argument("--expected-ip", default=DEFAULT_EXPECTED_IP)
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument(
        "--live",
        action="store_true",
        help="after all read-only stages, allow the matching live Make target",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    output_root = Path(args.output_root).expanduser().resolve()
    run = QualificationRun(output_root, args.controller, args.gesture)
    exit_code = 1
    outcome = "failed"
    error_text = None
    branch = None
    sha = None
    actual_ip = None
    software_preflight_pass = False
    try:
        if args.live and not args.expected_sha:
            raise QualificationFailure(
                "--live requires --expected-sha from the desktop-qualified commit"
            )
        branch, sha = check_repository(args.expected_sha, args.expected_branch)
        run.emit("[ok] clean Git state: {} {}".format(branch, sha))
        actual_ip = check_platform_and_network(args.interface, args.expected_ip)
        run.emit(
            "[ok] platform/network: {} {}={}".format(
                EXPECTED_ARCHITECTURE, args.interface, actual_ip
            )
        )

        describe_target = "describe-{}-{}".format(args.controller, args.gesture)
        preflight_target = "preflight-{}-{}".format(args.controller, args.gesture)
        live_target = "live-{}-{}".format(args.controller, args.gesture)
        for stage, command in (
            ("build", ("make", "build")),
            ("tests", ("make", "test")),
            ("pip-check", ("make", "pip-check")),
            ("describe", ("make", describe_target)),
            ("read-only-preflight", ("make", preflight_target)),
        ):
            run.run_command(stage, command)
        software_preflight_pass = True

        if not args.live:
            outcome = "software-preflight-passed-no-live"
            exit_code = 0
            run.emit(
                "SOFTWARE PREFLIGHT PASS; --live was not supplied, so no LowCmd motion was invoked"
            )
        else:
            if not sys.stdin.isatty():
                raise QualificationFailure("--live requires an interactive TTY")
            physical_checklist(run)
            phrase = LIVE_CONFIRMATIONS[(args.controller, args.gesture)]
            entered = input("Type {!r} to authorize this one live case: ".format(phrase))
            if entered.strip() != phrase:
                raise QualificationFailure(
                    "qualification confirmation did not match; live target was not invoked"
                )
            run.emit(
                "[ok] qualification authorization phrase matched for {} {}".format(
                    args.controller, args.gesture
                )
            )
            controller_log_dir = relative_controller_log_dir(run.directory)
            run.run_command(
                "live",
                (
                    "make",
                    "TRACKING_LOG_DIR={}".format(controller_log_dir),
                    live_target,
                ),
                interactive_output=True,
            )
            outcome = "live-case-completed"
            exit_code = 0
            run.emit(
                "LIVE CASE COMPLETED; inspect controller summaries and physical observations before marking pass"
            )
    except (QualificationFailure, OSError, subprocess.SubprocessError) as error:
        error_text = str(error)
        run.emit("qualification failed: {}".format(error_text), error=True)
    finally:
        summary_path = run.directory / "qualification.summary.json"
        command_path = run.directory / "commands.json"
        git_path = run.directory / "git_sha.txt"
        exit_path = run.directory / "exit_code.txt"
        command_path.write_text(
            json.dumps(run.commands, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        git_path.write_text((sha or "unavailable") + "\n", encoding="utf-8")
        exit_path.write_text(str(exit_code) + "\n", encoding="utf-8")
        summary = {
            "schema_version": 1,
            "created_at": run.created_at.isoformat(),
            "finished_at": datetime.now().astimezone().isoformat(),
            "controller": args.controller,
            "gesture": args.gesture,
            "live_requested": bool(args.live),
            "outcome": outcome,
            "error": error_text,
            "exit_code": exit_code,
            "git_branch": branch,
            "git_sha": sha,
            "expected_sha": args.expected_sha,
            "architecture": platform.machine(),
            "interface": args.interface,
            "actual_ip": actual_ip,
            "expected_ip": args.expected_ip,
            "stages": run.stages,
            "software_preflight_pass": software_preflight_pass,
            "physical_pass": False,
            "physical_pass_requires_manual_review": bool(args.live),
            "automatic_retry": False,
            "no_tracking_stop_fallback": False,
        }
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        run.close()
        write_hash_manifest(run.directory)
        print("qualification artifacts: {}".format(run.directory), flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
