#!/usr/bin/env python3
"""
battery_monitor.py

Connects to a Bitcraze Crazyflie via Crazyradio and prints battery voltage.
Safe: does NOT arm or send thrust commands.
"""

import argparse     # Allows passage of options from terminal like --seconds 10
import logging      # Gives structured status/error messages
import sys          # Used for system level behavior
import time         # Used for timing and sleeping

import cflib                                                # Crazyflie main python library
from cflib.crazyflie import Crazyflie                       # The drone object itself
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie     # Wrapper that makes connection cleaner and easier
from cflib.crazyflie.log import LogConfig                   # Tells the crazyflie what telemetry variables we want and how often


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"      # The address/path used to connect to the drone


def parse_args() -> argparse.Namespace:                                                 # Creates the command line parser
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
            cf = scf.cf     # Pulls out the crazyflie object (scf is drone wrapper, cf is the crazyflie object)

            # LOGGING CONFIGURATION
            # "pm.vbat" is the battery voltage variable exposed by the firmware.
            logconf = LogConfig(name="Battery", period_in_ms=int(1000 / args.rate)) # Telemetry request setup (which variable to gather and how often)
                                                                                    # Rate = 10Hz, then 1000 / 10 = 100ms

            logconf.add_variable("pm.vbat", "float")            # Add the battery voltage reading as a float           
            logconf.add_variable("stateEstimate.z", "float")    # Add the z height reading as a float

            latest = {"vbat": None, "z": None}     # Place to store the newest value when callback runs asynchronously for both vars

            def on_log_data(timestamp, data, logconf):              # CALLBACK FUNCTION that triggers when telemtry packet arrives, grabs and stores latest data
                latest["vbat"] = data.get("pm.vbat", None)          # pm = power management, vbat = battery voltage
                latest["z"] = data.get("stateEstimate.z", None)     # stateEstimate = state variable, z = z distance (height)
                                                            
            def on_log_error(logconf, msg):                 # If callback has problem then print error
                logging.error("Log error: %s", msg)
                                                                # Register the log config and callbacks
            cf.log.add_config(logconf)                          # Adds config to crazyflie logging susbsystem
            logconf.data_received_cb.add_callback(on_log_data)  # When log data arrives call on_log_data
            logconf.error_cb.add_callback(on_log_error)         # When logging fails, call on_log_error

            logconf.start()                                                                 # Start sending telemetry packets
            logging.info("Logging started. Running for %.1f seconds...", args.seconds)      # In this configuration

            t_end = time.time() + args.seconds          # Sets when script should stop
            while time.time() < t_end:                  # This loop does not receive telemtry data directly (only the callback does)
                v = latest["vbat"]                      # Reads the latest values, prints them, waits, then repeats
                z = latest["z"]

                if v is not None:
                    vprint = f"Battery: {v:.3f} V"                        # Prints if it is not "None"
                else:
                    vprint = "Battery: (waiting...)"

                if z is not None:
                    zprint = f"Height (z): {z:.3f} m"
                else:
                    zprint = "Height (z): (waiting...)"
                    
                if z is not None:
                    if z > 0.05:
                        flying = "Is Flying: TRUE"
                    else:
                        flying = "Is Flying: FALSE"
                else:
                    flying = "Is Flying: (waiting...)"
                
                print(vprint + " | " + zprint + " | " + flying)
                
                time.sleep(1.0 / args.rate)

            logconf.stop()              # Tells the drone to stop sending log packets
            logging.info("Done.")

        return 0

    except Exception as e:                  # Anything goes wrong log the full error
        logging.exception("Failed: %s", e)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
