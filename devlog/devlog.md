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

### GitHub
- Committed initial nodes and working pub/sub examples.  

### Next Steps
- Explore QoS settings in more depth. 

## Wednesday 9/3 (Day 2)
### Modules Completed
- ✅ Module 4 – Quality of Service (QoS)  

### Key Notes
- Learned default QoS profile shorthand in `create_publisher(..., 10)`.  
- Built explicit `QoSProfile` objects for pub/sub.  
- Tested Reliability: Reliable vs Best-Effort combinations.  
- Tested Durability: Volatile vs Transient Local (late joiner behavior).  
- Confirmed when message delivery succeeds vs fails.  

### GitHub
- Updated pub/sub nodes with QoS profile examples.  

### Next Steps
- Move to Module 5 (Custom Messages).