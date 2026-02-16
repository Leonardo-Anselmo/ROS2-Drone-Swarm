# CREATED BY:   	Leonardo Anselmo
# DATE CREATED: 	08/04/2025
# LAST REVISION:	10/01/2025 


## 08/04/2025
- Started this project and repo. Have the parts ready to order but waiting to return from vacation.


## Tuesday 09/02/2025
### Goals Completed
- Setup & Hello World  
- Publisher & Subscriber Basics  
- Putting It Together  

### Key Notes
- Workspace built cleanly with new `learn_ros2_py` package.  
- Learned node basics: publishers, subscribers, timers, callbacks.  
- Wrote and ran my own pub/sub nodes (not just demos).  
- Verified nodes communicate correctly when run in parallel.  

### Next Steps
- Explore QoS settings in more depth. 


## Wednesday 09/3/2025
### Goals Completed
- Quality of Service (QoS)  

### Key Notes
- Learned default QoS profile shorthand in `create_publisher(..., 10)`.  
- Built explicit `QoSProfile` objects for pub/sub.  
- Tested Reliability: Reliable vs Best-Effort combinations.  
- Tested Durability: Volatile vs Transient Local (late joiner behavior).  
- Confirmed when message delivery succeeds vs fails.  

### Next Steps
- Move to Module 5 (Custom Messages).


## Saturday 09/06/2025
### Goals Completed
- Learned about services in ROS2: one-off request/response instead of continuous streams.
- Created adder_service.py:
    - Built a Node with create_service.
    - Wrote a callback to process a and b from the request and return their sum.
    - Registered it in setup.py as a console script.
    - Built and ran the node in one terminal, then used ros2 service call in another to test it.
    - Verified correct output: sum=8 for inputs 3 and 5.

Takeaway: I can now write and run my own ROS2 service nodes, and I understand how services differ from publishers/subscribers.


## Wednesday 09/10/2025
### Goals Completed
- Declared parameters in a node (declare_parameter), giving them default values.
- Read parameters at startup (get_parameter(...).get_parameter_value()).
- Overrode defaults at runtime using:
    ros2 run learn_ros2_py parameter_node --ros-args -p my_parameter:=universe
- Added a parameter callback (add_on_set_parameters_callback) to detect and react when parameters are updated.
- Verified live updates using:
    ros2 param set /parameter_node my_parameter galaxy
- Node immediately logged the change.
- Learned that callbacks must return a SetParametersResult to confirm success.

Takeaway: I can now configure nodes at startup or dynamically while running, making them more flexible and interactive.


## Tuesday 09/16/2025
### Goals Completed
- Learned how actions extend services for long-running tasks with feedback and a final result.
- Created an Action Server (FibonacciActionServer) using ActionServer from rclpy.action.
- Implemented the execute_callback:
- Read the goal (goal_handle.request.order).
- Generated Fibonacci numbers in a loop.
- Published incremental feedback (goal_handle.publish_feedback).
- Returned a final result (Fibonacci.Result).
- Fixed goal status by calling goal_handle.succeed() before returning.
- Verified with CLI:
    - ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 7}" --feedback
        → saw feedback increments + final result sequence.
- Looked into cancel handling

Takeaway: I can now create and run ROS2 action servers, understand how they differ from services, and test them with feedback and results using the CLI.


## Wednesday 09/17/2025
### Goals Completed
- Created a launch/bringup.launch.py file inside the package.
- Learned that every launch file must define generate_launch_description().
- Started with one Node() (talker), then added listener.
- Fixed setup.py so launch files install correctly.
- Declared launch arguments: topic_name for remapping and publish_rate for the talker’s parameter.
- Used remappings=[("chatter", topic_name)] so both nodes could switch topic names from CLI.
- Passed publish_rate into talker so we can change its timer period at launch.
- Verified by running ros2 launch learn_ros2_py bringup.launch.py topic_name:=news publish_rate:=5.0 and checking messages.

Takeaway: I can now remap topics and pass parameters at launch time, so I don’t have to edit node code or run each node manually.


## Tuesday 09/23/2025
### Goals Completed
- Learned what a ComposableNode is: a node designed to be loaded into a shared container process.
- Saw how to start a container process with component_container_mt.
- Loaded the demo Talker component into the container from another terminal.
- Verified that the Talker ran inside the container.
- Loaded the demo Listener component into the same container.
- Confirmed Talker and Listener communicated as usual, but now they lived inside a single process.
- Discussed why this matters: reduced memory footprint, fewer terminals/processes to manage, and faster message passing via intra-process communication (zero-copy).

Takeaway: Compositions let you pack multiple nodes into one container process. From the outside, the ROS2 graph looks the same (topics, subs, pubs), but under the hood you save resources and gain speed. It’s the same nodes, just co-located more efficiently.


## Wednesday 09/24/2025
### Goals Completed
- Built a LifecycleTalker that starts unconfigured, configures to inactive (sets up pub + timer), then active (publishes only when active).
- Drove transitions with CLI: configure → activate → deactivate → shutdown.
- Learned that shutdown finalizes the node but does not auto-exit the Python process (use Ctrl+C or implement a small “exit-on-shutdown” pattern).

Takeaway: Lifecycle nodes give you explicit control over when work starts/stops. Configure resources in on_configure(), actually do work only when active, and use transitions to cleanly pause or shut down.

Future Work: This now marks the end of my personal ROS2 Node introduction course. I will now be continuing to learn simulation with Gazebo.


## Thursday 09/25/2025
### Goals Completed
- Created a clean sim_ws workspace (out of cloud-sync — good).
- Built ros_gz_interfaces from source (OK).
- Installed Gazebo/Ignition libs via conda-forge (libignition-transport11, libignition-msgs8, etc.).
- Cloned and built missing deps: actuator_msgs, gps_msgs, vision_msgs.
- Built ros_gz_bridge on macOS/Apple Silicon.
- Hit runtime crash (_PyExc_RuntimeError) when launching the bridge — traced to Python 3.11 vs. libs expecting 3.10.
- Verified gz sim runs server-only on macOS; GUI complained about OGRE (not needed for our goal).

Takeaways
- On macOS: run gz sim -s (server) and skip GUI for now.
- Use conda-forge for Ignition/Gazebo libs on Apple Silicon.
- Keep one unified env for ROS2 and Gazebo deps; mixing envs caused the symbol error.

Blocker: Bridge binary links into Python symbols; current env is Python 3.11. We need a unified Python 3.10 env with ROS 2 + Ignition.

Future Work: Once we unify on Python 3.10, the /clock demo should work immediately.


## Wednesday 10/01/2025
### Goals Completed
- Worked on ROS2 ↔ Gazebo Bridge Setup (Mac M4, ros_humble311 env)
- Created and tested a new Conda/Mamba environment (ros_humble311) with Python 3.11 to stabilize builds.
- Successfully rebuilt ros_gz_interfaces, actuator_msgs, gps_msgs, vision_msgs, and ros_gz_bridge.
- Fixed the dyld: symbol not found '_PyExc_RuntimeError' error by using a Python shim trick:
    - export DYLD_INSERT_LIBRARIES="$CONDA_PREFIX/lib/libpython3.11.dylib"
    - This allowed the parameter_bridge binary to run without crashing.
- Confirmed in Terminal C that the /clock topic was being published and bridged (saw correct publisher/subscriber info).
- Troubleshot Terminal D (ros2 topic echo /clock):
    - Initially blocked by daemon errors → switched to --no-daemon.
    - Then hit discovery issues due to inconsistent ROS_LOCALHOST_ONLY.
    - Found that some shells had ROS_LOCALHOST_ONLY=0 instead of =1, which broke communication.
    - Verified that ros_gz_bridge is functional (Terminal C prints “Creating GZ->ROS Bridge” and /clock is discoverable).
- The last blocker: Terminal D still not echoing messages reliably, even though publisher/subscriber relationships show up correctly.

Status
- Environment and packages are now successfully built.
- Bridge launches and connects.
- Topic subscription still flaky on Terminal D (likely QoS or env mismatch).
- Session ended after ~2hrs of troubleshooting to avoid burnout.

Future Work:
- Double-check ROS_LOCALHOST_ONLY=1 is exported in all four terminals before launching.
- Run in order:
    1. A: gz sim -s
    2. B: gz sim -g
    3. C: run bridge with shim (DYLD_INSERT_LIBRARIES).
    4. D: ros2 topic echo /clock --no-daemon
- If still silent → try ros2 topic hz /clock --no-daemon (to see if it’s timing out or just not printing).
- If that fails → test with another simple topic (/rosout or /parameter_events) to see if ros2 topic echo is working at all.


## Saturday 02/14/2026
### Crazyflie Hardware Bring-Up

Objective: Move from simulation-only development into real-world experimentation using Crazyflie 2.1 (brushed) platform.

Hardware Setup
Platform: Crazyflie 2.1 (brushed)
Radio: Crazyradio 2.0
Decks: Flow Deck v2, Multi-Ranger Deck
Environment: 12x16 ft indoor living room test space, Apple Silicon (M4) MacBook, Python CFClient

Assembly
- Installed 4 brushed motors (correct orientation verified)
- Mounted Flow Deck v2
- Mounted Multi-Ranger Deck
- Installed firmware on Crazyradio 2.0 (CRPA emulation)
- Verified USB detection on macOS
- Confirmed radio discovery via cfclient
- All motors successfully spun on initial power-up.

First Flight Attempts
- Initial Behavior
- Drone connected successfully
- Hover mode enabled
- Drone attempted stabilization
- Instability observed
- Drone fell after ~3–5 seconds

Diagnosis
- Flow Deck active
- Hover mode engaged
- Poor lighting conditions in room
- Optical flow algorithm requires sufficient surface texture + lighting.

Fix
- Turned on overhead lighting.
- Stable hover achieved
- Maintained position
- Controlled landing successful
- No hardware damage
- Battery ~3.76V

System Confirmed
- IMU fusion working
- Optical flow velocity estimation working
- Z-height control working
- PID stabilization loop healthy

Key Insight
- Hover instability was not tuning-related.
- It was environmental (optical flow sensor performance).

Next Steps
- Short Term
    - Repeat hover test
    - Introduce small manual yaw adjustments
    - Test small XY displacement and recovery
    - Measure hover drift

- Mid Term
    - Log sensor data during hover
    - Build minimal Python control script (no autopilot)
    - Begin structured control experiments

- Long Term
    - Multi-agent formation experiments
    - ROS2 interface layer
    - Swarm coordination architecture

Status
- Day 1: Hardware validated.
- Flight confirmed.
- Experimental phase officially started.


## Sunday 02/15/2026
### Telemetry Pipeline Online

- Implemented and successfully ran a Python battery monitoring tool using cflib.
- Connected to Crazyflie over Crazyradio 2.0
- Completed full connection handshake (TOC + param update)
- Started log stream and received live battery telemetry
- Verified stable voltage (~4.10 V)
- Confirmed end-to-end: hardware → radio → Python API → terminal output
- No flight testing today — focus was purely on programmatic connection and telemetry.