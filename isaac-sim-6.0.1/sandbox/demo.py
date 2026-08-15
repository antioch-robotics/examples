"""
A naked Isaac Sim session that idles forever, so the GUI is yours.

    antioch run sandbox/demo.py                        stream the GUI for 15 min
    antioch run --timeout 86400 sandbox/demo.py        keep it up all day
    antioch run sandbox/demo.py -- --seconds 3600      let the script stop itself

Nothing is scripted here: the stage starts empty and the loop only pumps the
Kit event loop, so everything happens through the streamed GUI — build a
scene, drop in assets, press Play, and inspect prims exactly as in a local
Isaac Sim. The outer bound is `antioch run`'s --timeout (default 900 s);
--seconds adds an inner one when you want the script to exit on its own.
"""

from __future__ import annotations

import argparse
import time

import antioch

HEARTBEAT_S = 300.0


def main() -> None:
    """Boot the bare engine and pump the event loop until a deadline, if any."""

    parser = argparse.ArgumentParser(description="Idle a naked Isaac Sim session for GUI use")
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after this long; 0 means run until the CLI timeout")
    arguments = parser.parse_args()

    antioch.boot()
    application = antioch.application()

    deadline = time.monotonic() + arguments.seconds if arguments.seconds > 0 else None
    print("sandbox is live — open the machine stream and build away", flush=True)

    next_heartbeat = time.monotonic() + HEARTBEAT_S
    updates = 0
    while antioch.is_running() and (deadline is None or time.monotonic() < deadline):
        application.update()
        updates += 1
        if time.monotonic() >= next_heartbeat:
            print(f"sandbox alive: {updates} updates", flush=True)
            next_heartbeat = time.monotonic() + HEARTBEAT_S
    print(f"sandbox closed after {updates} updates", flush=True)


if __name__ == "__main__":
    main()
