#!/usr/bin/env python3
"""Entry point for the Mobile-Manipulator host console.

    python tools/host_gui/main.py                 # pick a port and connect
    python tools/host_gui/main.py --sim           # virtual MCU, no hardware
    python tools/host_gui/main.py --demo          # simulator + enabling + motion
    python tools/host_gui/main.py --screenshot ui.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Running as a script puts tools/host_gui on sys.path, not tools/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PySide6.QtCore import QTimer  # noqa: E402

from host_gui.app import create_app  # noqa: E402


def _size(text: str) -> tuple[int, int]:
    try:
        width, height = text.lower().split("x")
        return int(width), int(height)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("size must look like 1440x900") from exc


def run(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="host_gui",
        description="Mobile-Manipulator serial console (docs/protocol.md v1)")
    parser.add_argument("--sim", action="store_true",
                        help="start with the built-in virtual MCU")
    parser.add_argument("--demo", action="store_true",
                        help="simulator + enable + a little motion")
    parser.add_argument("--screenshot", metavar="PATH",
                        help="save the window to PATH after a short demo and quit")
    parser.add_argument("--size", type=_size, default=(1440, 900),
                        help="window size, e.g. 1440x900")
    args = parser.parse_args(argv)

    app, window = create_app(sys.argv[:1])
    app.aboutToQuit.connect(window.shutdown)
    window.resize(*args.size)
    window.show()

    if args.sim or args.demo or args.screenshot:
        window.start_simulator(connect=True)
    if args.demo or args.screenshot:
        window.start_demo()

    if args.screenshot:
        def capture() -> None:
            saved = window.grab().save(args.screenshot)
            print(f"{'saved' if saved else 'could not save'} {args.screenshot}")
            app.quit()

        QTimer.singleShot(7000, capture)

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run())
