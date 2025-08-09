# ROS2-Drone-Swarm
An in-progress project involving development of a dual drone swarm system programmed using ROS2 and ultra-wideband (UWB) localization for precise relative positioning. The primary goal is to design, simulate, and deploy a cooperative control framework where two autonomous drones maintain formation and execute coordinated maneuvers such as orbiting and following behaviors.

The work emphasizes:
System-Level Robotics Engineering: Integrating sensors, embedded firmware, and high-level coordination algorithms
Simulation vs. Experimentation: Validating control logic in Gazebo before deploying to real hardware
Robustness and Maintainability: Designing for modularity, scalability, and future expansion with more agents

When complete, this project will serve as an example of applied robotics, embedded systems, and autonomous navigation, demonstrating practical skills in ROS2, embedded control, multi-agent systems, and hardware/software integration.


***Project Roadmap***
This project aims to design and implement an autonomous dual-drone swarm system using ROS2 and ultra wideband (UWB) localization. The goal is to demonstrate reliable flight behavior both in simulation and hardware testing.

The roadmap is divided into progressive milestones, each building toward a maintainable system that reflects industry best practices in robotics, embedded systems, and autonomous navigation.

**Phase 1 — System Design & Environment Setup**

- Define system architecture, including control loops, communication layers, and fail safe mechanisms
- Establish ROS2 workspace with dynamic node structure
- Configure Gazebo simulation environment with drone models and UWB anchor emulation

**Phase 2 — Simulation Development**

- Implement single drone control in Gazebo, integrating ROS2 nodes for:
    - State estimation
    - Command and control
    - Basic trajectory following
- Develop and test relative positioning algorithms using simulated UWB data
- Visualize swarm behavior in RViz and validate message passing via ROS2 topics

**Phase 3 — Hardware Integration**
- Assemble Crazyflie drones with UWB positioning decks
- Establish ROS2 to Crazyflie communication link with radio interface
- Test individual drone flight stability and control responsiveness in a lab environment

**Phase 4 — Swarm Coordination**
- Extend control algorithms to enable coordinated motion between drones
- Implement orbit and follow behaviors while maintaining relative positioning
- Add safety protocols (collision avoidance, emergency landing triggers)

**Phase 5 — Validation & Demonstration**
- Compare simulation and experimental performance, refining parameters
- Record demonstration video showcasing swarm behavior and technical architecture
- Publish technical write up detailing design choices, challenges, and solutions

**Key Technologies:**
- ROS2 (node-based control, DDS communication)
- Gazebo (physics-based simulation)
- Crazyflie 2.1 drones with UWB localization
- Python / C++ for control logic and hardware interfacing
- RViz for visualization and debugging

*Status*: In progress — system architecture defined, simulation environment setup underway.
