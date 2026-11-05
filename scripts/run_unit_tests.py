#!/usr/bin/env python3
"""Build and run every L1 unit test, locally or in CI.

Each test file already carries its complete gcc command in the header comment
(see tests/README.md), so this script reads the command from the header instead
of keeping a second copy that would drift. Run from anywhere:

    python scripts/run_unit_tests.py

Exit code is 0 only if every test builds and passes.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def command_from_header(source: Path) -> str | None:
    """Return the gcc command embedded in the file header, if any."""
    lines = source.read_text(encoding="utf-8").splitlines()
    collected: list[str] = []
    inside = False
    for line in lines[:80]:
        text = line.strip()
        if text.startswith("*") and "gcc " in text:
            inside = True
        if inside:
            body = text.lstrip("*").strip()
            if not body:
                continue
            collected.append(body)
            if not body.endswith("\\"):
                break
        if text.startswith("*/") and inside:
            break
    if not collected:
        return None
    return " ".join(part[:-1] if part.endswith("\\") else part
                    for part in collected)


def run_test(source: Path) -> tuple[str, str]:
    command = command_from_header(source)
    if not command or not command.startswith("gcc"):
        return "NO-CMD", "no gcc command in the header"

    match = re.search(r"-o\s+(\S+)", command)
    if not match:
        return "NO-OUT", "gcc command has no -o output"
    output = Path(match.group(1))
    output.parent.mkdir(parents=True, exist_ok=True)

    build = subprocess.run(command, shell=True, capture_output=True, text=True)
    if build.returncode != 0:
        return "BUILD-FAIL", (build.stderr or build.stdout)[-2000:]

    binary = output if output.exists() else Path(str(output) + ".exe")
    if not binary.exists():
        return "NO-BIN", f"{output} was not produced"
    result = subprocess.run([str(binary)], capture_output=True, text=True)
    if result.returncode != 0:
        return "RUN-FAIL", (result.stdout or result.stderr)[-2000:]
    return "PASS", ""


def main() -> int:
    os.chdir(ROOT)
    tests = sorted(Path("tests/unit").glob("*/*.c"))
    if not tests:
        print("no tests found under tests/unit", file=sys.stderr)
        return 1

    failed = 0
    for test in tests:
        status, detail = run_test(test)
        print(f"{status:12} {test}")
        if status != "PASS":
            failed += 1
            if detail:
                print(detail)

    print(f"SUMMARY {len(tests) - failed}/{len(tests)} passed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
