#!/usr/bin/env python3
"""
takeoff_hover_land.py

Has the drone take off, hover, then land.
Safe: does NOT arm or send thrust commands.
"""

import argparse     # Allows passage of options from terminal like --seconds 10
import logging      # Gives structured status/error messages
import time         # Used for timing and sleeping

import cflib                                                # Crazyflie main python library
from cflib.crazyflie import Crazyflie                       # The drone object itself
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie     # Wrapper that makes connection cleaner and easier


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"      # The address/path used to connect to the drone


def parse_args() -> argparse.Namespace:                                                 # Creates the command line parser
    parser = argparse.ArgumentParser(description="Crazyflie battery voltage monitor")
    parser.add_argument(
        "--uri",
        default=DEFAULT_URI,
        help=f"Radio URI (default: {DEFAULT_URI})",
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging for debugging.",
    )
    return parser.parse_args()


def setup_logging(verbose: bool) -> None:                   # Sets the format style of log messages
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )


def main() -> int:
    args = parse_args()
    setup_logging(args.verbose)
                                                        # CONNECTION SETUP
    cflib.crtp.init_drivers(enable_debug_driver=False)  # This initializes the low-level drivers used by cflib (radio/USB).

    logging.info("Connecting to: %s", args.uri)     # Prints where it's going to connect

    # SyncCrazyflie is a convenience wrapper:
    # - connects on entering the 'with' block
    # - disconnects automatically when leaving it
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:  # Connection address and creating crazyflie object
            cf = scf.cf                     # Pulls out the crazyflie object (scf is drone wrapper, cf is the crazyflie object)
            hlc = cf.high_level_commander   # Accesses the highest level logic for the crazyflie

            hlc.takeoff(0.5, 3.0)           # Tells the crazyflie to take off (z, t) to (z) height for (t) seconds
            time.sleep(6.0)                 # Tells the crazyflie to wait for (t) seconds
            hlc.land(0.0, 5.0)              # Tells the crazyflie to land (z, t) at (z) meters for (t) seconds
            time.sleep(3.0)

        return 0

    except Exception as e:                  # Anything goes wrong log the full error
        logging.exception("Failed: %s", e)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
