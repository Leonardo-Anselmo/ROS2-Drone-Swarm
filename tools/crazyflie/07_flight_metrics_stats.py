#!/usr/bin/env python3
"""
flight_metrics_stats.py

Records and saves metrics of several flight types including final stats and cleaned functions
Warning: this script sends flight commands.
"""

import argparse
import logging
import time
import csv

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig 


DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"


def parse_args() -> argparse.Namespace:                                         
    parser = argparse.ArgumentParser(description="Crazyflie telemtry monitoring")
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
    parser.add_argument(
        "--mode",
        default="takeoff",
        choices=["takeoff", "waypoint"],
        help="Flight test mode"
    )
    return parser.parse_args()


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

def run_phase(duration, phase_name, latest, csv_writer, rate):
    end_time = time.time() + duration

    max_z = 0
    max_y = 0
    max_x = 0

    start_v = None
    end_v = None

    while time.time() < end_time:
        v = latest["vbat"]
        z = latest["z"]
        y = latest["y"]
        x = latest["x"]

        if v is not None and x is not None and y is not None and z is not None:
            print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
            if start_v is None:
                start_v = v
            
            end_v = v

            if abs(z) > max_z:
                max_z = abs(z)

            if abs(y) > max_y:
                max_y = abs(y)

            if abs(x) > max_x:
                max_x = abs(x)

            csv_writer.writerow([
                time.time(),
                phase_name,   # change this depending on phase
                v,
                x,
                y,
                z
            ])
        else:
            print("Waiting for telemetry...")

        time.sleep(1.0 / rate)

    print("==", phase_name, "==")

    if start_v is not None and end_v is not None:
        print("Voltage Drop: ", start_v - end_v)
    else:
        print("Voltage Drop: Unavailable")

    print("Max x: ", max_x)
    print("Max y: ", max_y)
    print("Max z: ", max_z)

def run_waypoint_test(args, csv_writer):                                                            
    cflib.crtp.init_drivers(enable_debug_driver=False)

    logging.info("Connecting to: %s", args.uri)

    # SyncCrazyflie is a convenience wrapper:
    # - connects on entering the 'with' block
    # - disconnects automatically when leaving it
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            hlc = cf.high_level_commander

            logconf = LogConfig(name="Battery", period_in_ms=int(1000 / args.rate))

            logconf.add_variable("pm.vbat", "float")         
            logconf.add_variable("stateEstimate.z", "float")
            logconf.add_variable("stateEstimate.y", "float")
            logconf.add_variable("stateEstimate.x", "float")

            latest = {"vbat": None, "x": None, "y": None, "z": None}

            def on_log_data(timestamp, data, logconf):
                latest["vbat"] = data.get("pm.vbat", None)
                latest["z"] = data.get("stateEstimate.z", None)
                latest["y"] = data.get("stateEstimate.y", None)
                latest["x"] = data.get("stateEstimate.x", None)
                                                                
            def on_log_error(logconf, msg):
                logging.error("Log error: %s", msg)


            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(on_log_data)
            logconf.error_cb.add_callback(on_log_error)

            logconf.start()
            logging.info("Logging started. Running for %.1f seconds...", args.seconds)

            hlc.takeoff(0.5, 3.0)
            run_phase(3, "takeoff", latest, csv_writer, args.rate)

            hlc.go_to(0.7, 0.0, 0.5, 0.0, 5.0)  # Forward waypoint on the square (top right corner)
            run_phase(5, "waypoint_1", latest, csv_writer, args.rate)

            hlc.go_to(0.7, 0.7, 0.5, 0.0, 5.0)  # Left waypoint on the square (top left corner)
            run_phase(5, "waypoint_2", latest, csv_writer, args.rate)
                
            hlc.go_to(0.0, 0.7, 0.5, 0.0, 5.0)  # Back waypoint on the square (bottom left corner)
            run_phase(5, "waypoint_3", latest, csv_writer, args.rate)

            hlc.go_to(0.0, 0.0, 0.5, 0.0, 5.0)  # Right waypoint on the square (bottom right corner)
            run_phase(5, "waypoint_4", latest, csv_writer, args.rate)
                
            hlc.land(0.0, 3.0)
            run_phase(3, "land", latest, csv_writer, args.rate)

            logconf.stop()
            logging.info("Done.")

        return 0

    except Exception as e:
        logging.exception("Failed: %s", e)
        return 1
    
def run_takeoff_land_test(args, csv_writer):
    cflib.crtp.init_drivers(enable_debug_driver=False)  # This initializes the low-level drivers used by cflib (radio/USB).

    logging.info("Connecting to: %s", args.uri)     # Prints where it's going to connect

    # SyncCrazyflie is a convenience wrapper:
    # - connects on entering the 'with' block
    # - disconnects automatically when leaving it
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:  # Connection address and creating crazyflie object
            cf = scf.cf
            hlc = cf.high_level_commander

            logconf = LogConfig(name="Battery", period_in_ms=int(1000 / args.rate))

            logconf.add_variable("pm.vbat", "float")         
            logconf.add_variable("stateEstimate.z", "float")
            logconf.add_variable("stateEstimate.y", "float")
            logconf.add_variable("stateEstimate.x", "float")

            latest = {"vbat": None, "x": None, "y": None, "z": None}

            def on_log_data(timestamp, data, logconf):
                latest["vbat"] = data.get("pm.vbat", None)
                latest["z"] = data.get("stateEstimate.z", None)
                latest["y"] = data.get("stateEstimate.y", None)
                latest["x"] = data.get("stateEstimate.x", None)
                                                                
            def on_log_error(logconf, msg):
                logging.error("Log error: %s", msg)


            cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(on_log_data)
            logconf.error_cb.add_callback(on_log_error)

            logconf.start()
            logging.info("Logging started. Running for %.1f seconds...", args.seconds)

            hlc.takeoff(1.0, 5.0)
            run_phase(5, "takeoff", latest, csv_writer, args.rate)

            run_phase(3, "hover", latest, csv_writer, args.rate)
                
            hlc.land(0.0, 5.0)
            run_phase(5, "land", latest, csv_writer, args.rate)

            logconf.stop()
            logging.info("Done.")

        return 0

    except Exception as e:                  # Anything goes wrong log the full error
        logging.exception("Failed: %s", e)
        return 1


def main() -> int:
    args = parse_args()
    setup_logging(args.verbose)

    csv_file = open("board_sbat_takeoff_land_log_6.csv", mode="w", newline="")
    csv_writer = csv.writer(csv_file)

    csv_writer.writerow([
        "timestamp",
        "phase",
        "battery_v",
        "x_m",
        "y_m",
        "z_m"
    ])

    if args.mode == "takeoff":
        return run_takeoff_land_test(args, csv_writer)

    elif args.mode == "waypoint":
        return run_waypoint_test(args, csv_writer)

    csv_file.close()

if __name__ == "__main__":
    raise SystemExit(main())
