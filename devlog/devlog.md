# CREATED BY:   	Leonardo Anselmo
# DATE CREATED: 	08/04/2025
# LAST REVISION:	09/23/2025 


## 08/04/2025 (Day 0)
- Started this project and repo. Have the parts ready to order but waiting to return from vacation.


## Tuesday 9/2 (Day 1)
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


## Wednesday 9/3 (Day 2)
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


## Saturday 9/6 (Day 3)
### Goals Completed
- Learned about services in ROS2: one-off request/response instead of continuous streams.
- Created adder_service.py:
    - Built a Node with create_service.
    - Wrote a callback to process a and b from the request and return their sum.
    - Registered it in setup.py as a console script.
    - Built and ran the node in one terminal, then used ros2 service call in another to test it.
    - Verified correct output: sum=8 for inputs 3 and 5.

Takeaway: I can now write and run my own ROS2 service nodes, and I understand how services differ from publishers/subscribers.


## Wednesday 9/10 (Day 4)
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


## Tuesday 9/16
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


## Wednesday 9/17
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


## Tuesday 9/23
### Goals Completed
- Learned what a ComposableNode is: a node designed to be loaded into a shared container process.
- Saw how to start a container process with component_container_mt.
- Loaded the demo Talker component into the container from another terminal.
- Verified that the Talker ran inside the container.
- Loaded the demo Listener component into the same container.
- Confirmed Talker and Listener communicated as usual, but now they lived inside a single process.
- Discussed why this matters: reduced memory footprint, fewer terminals/processes to manage, and faster message passing via intra-process communication (zero-copy).

Takeaway: Compositions let you pack multiple nodes into one container process. From the outside, the ROS2 graph looks the same (topics, subs, pubs), but under the hood you save resources and gain speed. It’s the same nodes, just co-located more efficiently.


## Wednesday 9/24
### Goals Completed
- Built a LifecycleTalker that starts unconfigured, configures to inactive (sets up pub + timer), then active (publishes only when active).
- Drove transitions with CLI: configure → activate → deactivate → shutdown.
- Learned that shutdown finalizes the node but does not auto-exit the Python process (use Ctrl+C or implement a small “exit-on-shutdown” pattern).

Takeaway: Lifecycle nodes give you explicit control over when work starts/stops. Configure resources in on_configure(), actually do work only when active, and use transitions to cleanly pause or shut down.

Future Work: This now marks the end of my personal ROS2 Node introduction course. I will now be continuing to learn simulation with Gazebo.