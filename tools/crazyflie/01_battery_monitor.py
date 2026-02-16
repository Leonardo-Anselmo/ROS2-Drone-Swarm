#!/usr/bin/env python3
"""
battery_monitor.py

Connects to a Bitcraze Crazyflie via Crazyradio and prints battery voltage.
Safe: does NOT arm or send thrust commands.
"""

import argparse
import logging
import sys
import time

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Crazyflie battery voltage monitor")
    parser.add_argument(
        "--uri",
        default=DEFAULT_URI,
        help=f"Radio URI (default: {DEFAULT_URI})",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="How often to print (Hz). Default 10 Hz.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="How long to run before exiting. Default 5 seconds.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging for debugging.",
    )
    return parser.parse_args()


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )


def main() -> int:
    args = parse_args()
    setup_logging(args.verbose)

    # This initializes the low-level drivers used by cflib (radio/USB).
    cflib.crtp.init_drivers(enable_debug_driver=False)

    logging.info("Connecting to: %s", args.uri)

    # SyncCrazyflie is a convenience wrapper:
    # - connects on entering the 'with' block
    # - disconnects automatically when leaving it
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf

            # Configure logging from the Crazyflie.
            # "pm.vbat" is the battery voltage variable exposed by the firmware.
            logconf = LogConfig(name="Battery", period_in_ms=int(1000 / args.rate))
            logconf.add_variable("pm.vbat", "float")

            latest = {"vbat": None}

            def on_log_data(timestamp, data, logconf):
                latest["vbat"] = data.get("pm.vbat", None)

            def on_log_error(logconf, msg):
                logging.error("Log error: %s", msg)

            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(on_log_data)
            logconf.error_cb.add_callback(on_log_error)

            logconf.start()
            logging.info("Logging started. Running for %.1f seconds...", args.seconds)

            t_end = time.time() + args.seconds
            while time.time() < t_end:
                v = latest["vbat"]
                if v is not None:
                    print(f"Battery: {v:.3f} V")
                else:
                    print("Battery: (waiting for first packet...)")
                time.sleep(1.0 / args.rate)

            logconf.stop()
            logging.info("Done.")

        return 0

    except Exception as e:
        logging.exception("Failed: %s", e)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
