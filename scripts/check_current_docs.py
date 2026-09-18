#!/usr/bin/env python3
"""Reject retired, copy-pasteable command lines from current Markdown docs."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
DOCS = [ROOT / "README.md", ROOT / "HANDOVER.md", *sorted((ROOT / "docs").glob("*.md"))]
RETIRED_COMMANDS = (
    "cd ~/ur_learn",
    "ros2 run ur_link ur_command_node",
    "python3 scripts/ur_pick_place_full.py",
    "python3 ~/UR10/scripts/ur_pick_place_full.py",
)


def main() -> int:
    failures = []
    for path in DOCS:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            for retired in RETIRED_COMMANDS:
                if retired in stripped:
                    failures.append(f"{path.relative_to(ROOT)}:{number}: retired command: {retired}")
            match = re.search(r"(?:python3|/usr/bin/python3)\s+(scripts/[A-Za-z0-9_.-]+\.py)", stripped)
            if match and not (ROOT / match.group(1)).is_file():
                failures.append(
                    f"{path.relative_to(ROOT)}:{number}: missing script target: {match.group(1)}"
                )
            match = re.search(r"(?:bash|sh)\s+(scripts/[A-Za-z0-9_.-]+\.sh)", stripped)
            if match and not (ROOT / match.group(1)).is_file():
                failures.append(
                    f"{path.relative_to(ROOT)}:{number}: missing script target: {match.group(1)}"
                )
    handover = (ROOT / "HANDOVER.md").read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    if "2026-09-18" not in handover:
        failures.append("HANDOVER.md: missing current update date")
    if "docs/00-当前可用操作.md" not in readme:
        failures.append("README.md: missing current-operation entry point")
    if failures:
        print("Current-doc check failed:", file=sys.stderr)
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"Current-doc check passed: {len(DOCS)} Markdown files scanned.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
