# ROS2-Drone-Swarm
An in-progress project involving development of a dual drone swarm system programmed using ROS2 and ultra-wideband (UWB) localization for precise relative positioning. The primary goal is to design, simulate, and deploy a cooperative control framework where two autonomous drones maintain formation and execute coordinated maneuvers such as orbiting and following behaviors.

The work emphasizes:
System-Level Robotics Engineering: Integrating sensors, embedded firmware, and high-level coordination algorithms
Simulation vs. Experimentation: Validating control logic in Gazebo before deploying to real hardware
Robustness and Maintainability: Designing for modularity, scalability, and future expansion with more agents

When complete, this project will serve as an example of applied robotics, embedded systems, and autonomous navigation, demonstrating practical skills in ROS2, embedded control, multi-agent systems, and hardware/software integration.


***Project Roadmap***
This project follows a hardware-first experimental approach using Crazyflie platforms as the initial development vehicle. Simulation will support experimentation when necessary, but real-world testing drives system validation.

The roadmap emphasizes incremental capability building — progressing from single-drone control to coordinated multi-agent behavior.


**Phase 1 — Single Drone Foundations (Experimental Control Layer)**

Objective: Establish full programmatic control and telemetry over one Crazyflie.

- Validate radio communication and firmware configuration
- Develop standalone Python tools for:
    - Telemetry logging (battery, state estimates)
    - Parameter inspection
    - Safe takeoff and landing routines
- Understand Crazyflie firmware architecture and control stack
- Implement scripted hover and basic motion commands

Outcome: A reproducible software pipeline capable of commanding and monitoring a single drone.
Current Status: Active — telemetry and manual flight verified.


**Phase 2 — Controlled Autonomy (Single Agent)**

Objective: Move from manual control to scripted behaviors.

- Implement:
    - Programmatic takeoff / hover / land
    - Position hold using Flow Deck
    - Velocity-based control primitives
- Develop modular Python control scripts structured like ROS nodes
- Log and analyze stability performance

Outcome: A stable, software-controlled single drone with repeatable behavior.


**Phase 3 — Relative Localization (UWB Integration)**

Objective: Introduce UWB positioning for spatial awareness.

- Integrate Loco Positioning Deck
- Configure anchor nodes
- Validate absolute positioning indoors
- Analyze noise and drift characteristics

Outcome: Reliable indoor position estimates for autonomous control.


**Phase 4 — Dual Drone Communication & Coordination**

Objective: Establish synchronized control across two agents.

- Implement separate control pipelines for two Crazyflies
- Validate simultaneous flight
- Implement:
    - Leader–Follower behavior
    - Relative distance maintenance
    - Simple orbit maneuver

Outcome: Coordinated multi-agent behavior in a controlled indoor space.


**Phase 5 — Robustness & Expansion**

Objective: Harden the system and extend its capabilities.

- Add safety mechanisms:
    - Low battery auto-land
    - Loss-of-signal failsafe
    - Distance threshold abort
- Compare experimental performance to lightweight simulation models
- Record technical demonstration

Outcome: A documented dual-drone swarm prototype suitable for portfolio and research presentation.


**Key Technologies**

- Crazyflie 2.1 (Brushed platform)
- Crazyradio 2.0
- Flow Deck v2
- Multi-Ranger Deck
- UWB Loco Positioning System (planned)
- Python (cflib)
- ROS2 (future integration layer)
- Embedded control theory principles

**Status**

- Hardware operational
- Telemetry pipeline functional
- Entering Phase 1: Experimental Control Layer
    - See the devlog for more!
