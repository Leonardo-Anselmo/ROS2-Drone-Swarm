# CREATED BY:   	Leonardo Anselmo
# DATE CREATED: 	08/04/2025
# LAST REVISION:	09/04/2025 


## 08/04/2025 (Day 0)
- Started this project and repo. Have the parts ready to order but waiting to return from vacation.


## Tuesday 9/2 (Day 1)
### Modules Completed
- Module 1 – Setup & Hello World  
- Module 2 – Publisher & Subscriber Basics  
- Module 3 – Putting It Together  

### Key Notes
- Workspace built cleanly with new `learn_ros2_py` package.  
- Learned node basics: publishers, subscribers, timers, callbacks.  
- Wrote and ran my own pub/sub nodes (not just demos).  
- Verified nodes communicate correctly when run in parallel.  

### Next Steps
- Explore QoS settings in more depth. 


## Wednesday 9/3 (Day 2)
### Modules Completed
- Module 4 – Quality of Service (QoS)  

### Key Notes
- Learned default QoS profile shorthand in `create_publisher(..., 10)`.  
- Built explicit `QoSProfile` objects for pub/sub.  
- Tested Reliability: Reliable vs Best-Effort combinations.  
- Tested Durability: Volatile vs Transient Local (late joiner behavior).  
- Confirmed when message delivery succeeds vs fails.  

### Next Steps
- Move to Module 5 (Custom Messages).


## Saturday 9/6 (Day 3)
### Modules Completed
- Learned about services in ROS2: one-off request/response instead of continuous streams.
- Created adder_service.py:
    - Built a Node with create_service.
    - Wrote a callback to process a and b from the request and return their sum.
    - Registered it in setup.py as a console script.
    - Built and ran the node in one terminal, then used ros2 service call in another to test it.
    - Verified correct output: sum=8 for inputs 3 and 5.

Takeaway: I can now write and run my own ROS2 service nodes, and I understand how services differ from publishers/subscribers.


## Saturday 9/10 (Day 4)
### Modules Completed
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