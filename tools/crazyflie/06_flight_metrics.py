#!/usr/bin/env python3
"""
flight_metrics.py

Records and saves metrics of several flight types
Safe: does NOT arm or send thrust commands.
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
    return parser.parse_args()


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )




def main() -> int:
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

    def run_waypoint_test():
        args = parse_args()
        setup_logging(args.verbose)
                                                            
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
                end_time = time.time() + 3.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "takeoff",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                hlc.go_to(0.7, 0.0, 0.5, 0.0, 5.0)  # Forward waypoint on the square (top right corner)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "waypoint_1",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                hlc.go_to(0.7, 0.7, 0.5, 0.0, 5.0)  # Left waypoint on the square (top left corner)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "waypoint_2",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                hlc.go_to(0.0, 0.7, 0.5, 0.0, 5.0)  # Back waypoint on the square (bottom left corner)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "waypoint_3",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                hlc.go_to(0.0, 0.0, 0.5, 0.0, 5.0)  # Right waypoint on the square (bottom right corner)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "waypoint_4",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                hlc.land(0.0, 3.0)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "land",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                logconf.stop()
                logging.info("Done.")
                csv_file.close()

            return 0

        except Exception as e:
            logging.exception("Failed: %s", e)
            return 1
    
    def run_takeoff_land_test():
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
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "takeoff",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                end_time = time.time() + 3.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "hover",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)
                
                hlc.land(0.0, 5.0)
                end_time = time.time() + 5.0
                while time.time() < end_time:
                    v = latest["vbat"]
                    z = latest["z"]
                    y = latest["y"]
                    x = latest["x"]

                    if v is not None and x is not None and y is not None and z is not None:
                        print(f"Battery: {v:.3f} V" + " | " + f"X: {x:.3f} m" + " | " + 
                            f"Y: {y:.3f} m" + " | " + f"Z: {z:.3f} m")
                        csv_writer.writerow([
                            time.time(),
                            "land",   # change this depending on phase
                            v,
                            x,
                            y,
                            z
                        ])
                    else:
                        print("Waiting for telemtry...")
                    
                    time.sleep(1.0 / args.rate)

                logconf.stop()
                logging.info("Done.")
                csv_file.close()

            return 0

        except Exception as e:                  # Anything goes wrong log the full error
            logging.exception("Failed: %s", e)
            return 1

    run_takeoff_land_test()
    # run_waypoint_test()

if __name__ == "__main__":
    raise SystemExit(main())
