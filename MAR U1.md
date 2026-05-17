# Mobile and Autonomous Robotics — Unit 1: Complete Notes
### Course Code: UE23CS343BB7 | PES University

---

> **How to use these notes:** Every concept below is explained from first principles with analogies, formal definitions, and practical examples. You do not need the slides.

---

## Table of Contents

1. [Introduction to Autonomous Robots](#1-introduction-to-autonomous-robots)
2. [Historical Overview and Future](#2-historical-overview-and-future)
3. [Basic Concepts and Terminology](#3-basic-concepts-and-terminology)
4. [Sensors — Types, Classification, Applications](#4-sensors--types-classification-applications)
5. [Actuators — Types and Applications](#5-actuators--types-and-applications)
6. [Motors and Controllers](#6-motors-and-controllers)
7. [Power Sources and Management](#7-power-sources-and-management)
8. [Range Finders](#8-range-finders)
9. [Encoders](#9-encoders)
10. [Vision Sensors](#10-vision-sensors)
11. [ROS2 — Architecture and Communication](#11-ros2--architecture-and-communication)
12. [AI-Powered Robotics](#12-ai-powered-robotics)
13. [Locomotion — Fundamentals](#13-locomotion--fundamentals)
14. [Legged Mobile Robots](#14-legged-mobile-robots)
15. [Wheeled Mobile Robots](#15-wheeled-mobile-robots)
16. [Aerial Mobile Robots (UAVs)](#16-aerial-mobile-robots-uavs)
17. [Degrees of Freedom (DOF)](#17-degrees-of-freedom-dof)
18. [Robot Kinematics — Forward and Inverse](#18-robot-kinematics--forward-and-inverse)
19. [Mathematical Foundations for Kinematics](#19-mathematical-foundations-for-kinematics)

---

## 1. Introduction to Autonomous Robots

### What is a Robot?
A robot is a machine that can sense its environment, process that information, and take actions to accomplish goals. Think of it as a physical agent — similar to a person — that can observe the world (via sensors), think about what to do (via algorithms), and then act (via motors and actuators).

### What is Robotics?
Robotics is the multidisciplinary field that covers the design, construction, programming, and operation of robots. It draws from mechanical engineering, electrical engineering, computer science, AI, and even biology.

### What is an Autonomous Robot?

**Definition:** An autonomous robot is a robotic system that can perform tasks and make decisions *without continuous human intervention*. It combines hardware (sensors, motors, chassis) and software (algorithms, AI) to perceive, interpret, and act in its environment entirely on its own.

**Analogy:** Think of an autonomous robot like a self-driving taxi driver. A regular robot is like a puppet — someone pulls the strings. An autonomous robot is like a taxi driver who knows the city, handles traffic, avoids potholes, and gets you to your destination without you giving turn-by-turn directions.

### The Four Fundamental Questions Every Autonomous Robot Must Answer

| Question | Technical Term | What It Means |
|---|---|---|
| **What is around me?** | Sensor Interpretation / Perception | Using cameras, LiDAR, etc. to detect nearby objects |
| **Where am I?** | Localization | Finding its own position in the map |
| **Where am I going?** | Map Building | Integrating sensor info to build/use a map |
| **How do I get there?** | Path Planning | Choosing the best route to a target |

### Three Core Characteristics of Autonomous Robots

**1. Perception**
The ability to sense the environment. Sensors like cameras detect objects; LiDAR measures distances; microphones pick up sound. Without perception, the robot is blind.

*Analogy:* Perception is like your eyes and ears. Before you cross the road, you look both ways — that's perception.

**2. Decision-Making**
Processing sensory input and choosing what action to take — following a plan, adapting to new conditions, or even learning from experience.

*Analogy:* Decision-making is like your brain deciding whether to cross the road based on what your eyes saw.

**3. Actuation**
Physically doing something — moving wheels, opening a gripper, rotating a joint. This is the "muscle" of the robot.

*Analogy:* Actuation is your legs moving you across the road.

### Why Are Robots Needed?
- **Efficiency and productivity:** Tireless repetition, 24/7 operation
- **Safety:** Handling hazardous tasks (bomb disposal, nuclear environments, disaster zones)
- **Precision:** Surgery, microelectronics assembly
- **Exploration:** Mars rovers, deep-sea probes
- **Care:** Elder care, assistive devices for the disabled

### Applications Summary
Industrial assembly and inspection, Search and rescue, Hazardous operations (radiation, mines), Medical robotics, Rehabilitation, Education, Transportation (self-driving), Entertainment.

---

## 2. Historical Overview and Future

### Timeline of Autonomous Robots

**1912** — *El Ajedrecista* by Leonardo Torres Quevedo. The first truly autonomous machine — an electromechanical chess-playing device that could checkmate a human in an endgame. A machine that "thought" in a limited domain.

**1948–1949** — *Elmer and Elsie* (William Grey Walter). These "tortoise" robots explored their environment using light sensors and simple reactive behaviors. They are considered the dawn of true autonomous robotics — the first robots "programmed to think" like a biological brain, with free will.

**1950s** — *Unimate* (George Devol). The first industrial robot arm. Programmable, factory-deployable. Ushered in the automation of manufacturing.

**1960s** — *SHAKEY* (Stanford Research Institute). Used cameras, tactile sensors, and rudimentary AI planning. Could navigate obstacle courses and perform basic object manipulation. The first robot that combined perception, reasoning, and action.

**1970s** — Rise of "pick-and-place" industrial robots. Repetitive but precise. Became the backbone of automotive manufacturing.

**1980s–1990s** — AI enters robotics. Learning algorithms, improved sensing. Robots begin to adapt to changing environments rather than following fixed programs.

**2000s** — *AIBO* (Sony's dog robot), *Paro* (therapeutic seal robot). Robots designed for human emotional interaction. *Roomba* popularizes autonomous vacuum cleaning in homes.

**2010s–Present** — Self-driving cars (Waymo, Tesla), delivery drones (Amazon Prime Air), warehouse robots (Amazon Kiva). Massive advances in sensors, compute, and deep learning.

**The Future:**
- **AI + ML Integration:** Robots that continuously learn and adapt
- **Human-Robot Collaboration (HRC):** Working safely alongside humans in factories, hospitals
- **Swarm Robotics:** Coordinated groups of simple robots accomplishing complex goals (like a colony of ants)
- **Autonomous Vehicles:** Widespread self-driving transport
- **Healthcare Robotics:** AI-powered surgery assistants, diagnostics
- **Soft Robotics:** Robots with flexible, deformable bodies for delicate environments
- **Edge AI:** AI running directly on the robot, without cloud dependency
- **Ethical & Regulatory Frameworks:** As robots become more capable, society must govern their use

---

## 3. Basic Concepts and Terminology

This section defines every key term you'll encounter throughout the course.

**Sensor** — A device that detects physical properties of the environment (light, distance, temperature, sound) and converts them into signals the robot can process. The robot's "sense organs."

**Actuator** — A device that converts control signals into physical motion or action. The robot's "muscles." Examples: motors, servos, pneumatic cylinders.

**Control Algorithm** — A set of instructions (software) governing the robot's behavior — what to do, when to do it, and how. Think of it as the robot's rulebook or decision procedure.

**Navigation** — The process of moving from one location to another. It includes *both* path planning (strategic) and real-time movement (operational). Like planning and then driving a road trip.

**Localization** — Determining the robot's exact position and orientation within its environment. The robot equivalent of GPS + compass. Without localization, the robot doesn't know where it is.

**Mapping** — Creating a representation (a map) of the environment, typically using sensor data. Often done simultaneously with localization.

**SLAM (Simultaneous Localization and Mapping)** — The robot builds a map of an unknown environment while *simultaneously* figuring out its own position within that map. Extremely hard because both the map and position are unknown and interdependent.

*Analogy for SLAM:* Imagine waking up in an unknown building with no phone, and you're trying to figure out both the floor plan *and* where you are in it at the same time, using only what you can see from your current position.

**Path Planning** — Calculating the optimal (or safe) route from the robot's current position to a goal, while avoiding obstacles. Strategic, computed in advance or updated in real-time.

**Collision Avoidance** — Techniques for preventing the robot from hitting obstacles during movement. Reactive and real-time. Think of it as defensive driving.

**Human-Robot Interaction (HRI)** — The study of how humans and robots communicate and collaborate effectively. Design principles cover ergonomics, safety, natural language, gesture, and trust.

**Swarm Robotics** — Multiple robots coordinating to achieve a shared goal. Individual robots are simple; collective intelligence is powerful. Inspired by ant colonies, bee swarms, bird flocks.

**Teleoperation** — Controlling a robot remotely (from a distance), often used in hazardous environments (bomb disposal, space, deep-sea). The human remains in the loop.

**Feedback Control System** — A system that uses sensor feedback to constantly compare the current state with the desired state and applies corrections. The thermostat is the classic example: it senses room temperature and turns heating on/off to maintain the set point.

**End-Effector** — The tool or device at the tip of a robot arm that interacts with the world — a gripper, welder, drill, suction cup, or camera. The robot's "hand."

**Task Planning** — Determining a high-level sequence of actions to accomplish a goal. Different from path planning (which is spatial); task planning is about *what* to do, not just *where* to go.

---

## 4. Sensors — Types, Classification, Applications

### What Are Sensors?

**Definition:** Sensors are devices that detect and measure physical properties or changes in the environment and convert this information into signals that can be interpreted by a computing system.

*Analogy:* Your body has sensors everywhere — eyes (light), ears (sound), skin (touch, temperature, pressure), nose (chemistry), inner ear (balance and acceleration). A robot's sensors serve the same roles.

The key characteristics of sensors:
- They interface the physical world with electronic/computational systems
- Output can be analog (continuous) or digital (discrete)
- They enable real-time feedback and decision-making in autonomous systems
- They are fundamental to IoT and autonomous systems

### Sensor Classification

Sensors are classified on **two axes**:

#### Axis 1: Proprioceptive vs. Exteroceptive

**Proprioceptive Sensors** — Measure values *internal* to the robot itself.
- Motor speed, wheel load, joint angles, battery voltage
- *Analogy:* Like your sense of where your own limbs are (proprioception in humans — you can touch your nose with your eyes closed because of this sense).

**Exteroceptive Sensors** — Measure values from the *external environment*.
- Distance to objects, light intensity, sound, temperature
- *Analogy:* Like your eyes and ears — they sense the world outside your body.

#### Axis 2: Passive vs. Active

**Passive Sensors** — Measure ambient energy *entering* the sensor from the environment; do not emit anything.
- Examples: Temperature probes (feel existing heat), microphones (capture existing sound), CCD/CMOS cameras (capture existing light)
- *Analogy:* Like sitting quietly and listening — you receive information without broadcasting anything.

**Active Sensors** — *Emit* energy into the environment, then measure the reaction/return.
- Examples: LiDAR (emits laser pulses), Ultrasonic sensors (emits sound waves), Radar (emits radio waves)
- Advantage: More controlled, often more accurate
- Risk: The emitted energy can interfere with what you're trying to measure, or interfere with other sensors
- *Analogy:* Like shouting in a cave and listening for the echo to estimate the cave's size.

### Complexity Hierarchy (ascending complexity, descending maturity)

Tactile/Proprioceptive → Wheel encoders → Sonar/Ultrasonic → Laser rangefinders → Radar → Vision (Camera-based)

The simpler sensors are well-understood and cheap. Vision sensors are the most powerful but most complex to process.

### Specific Sensor Types

**IMU (Inertial Measurement Unit)**
- Contains gyroscopes (measure angular velocity / rotation rate) and accelerometers (measure linear acceleration)
- Used for: Navigation, balancing, motion control
- *Analogy:* The inner ear in your head — it tells you if you're tilting, spinning, or accelerating without looking at the outside world.

**LiDAR (Light Detection and Ranging)**
- Emits rapid pulses of laser light; measures the time for reflected light to return → calculates distance
- Produces 3D "point clouds" — a dense map of distances in all directions
- Used for: 3D mapping, obstacle detection, autonomous vehicle navigation
- *Analogy:* Like a bat using echolocation but with light instead of sound, and spinning 360° many times per second.

**Radar (Radio Detection and Ranging)**
- Emits radio frequency waves; measures reflections
- Works in adverse weather (rain, fog, dust) where LiDAR and cameras struggle
- Used for: Long-range object detection, adverse-condition navigation
- *Analogy:* Like LiDAR but uses radio waves instead — less precise, but penetrates weather.

**Ultrasonic Sensors**
- Emits ultrasonic sound waves (above human hearing); measures echo time → distance
- Short range (typically < 5 m), cheap, widely used
- Used for: Proximity detection, parking sensors, obstacle avoidance at close range
- *Analogy:* The parking sensor beep in a car — gets louder as you get closer to a wall.

**Camera Systems**
- Optical sensors capturing visual information (RGB color images or grayscale)
- Used for: Object recognition, scene interpretation, navigation, mapping, face recognition
- Combined with AI for: Object detection, semantic understanding, visual odometry
- *Analogy:* The robot's eyes. Rich in information but requires heavy computation to interpret.

**GPS (Global Positioning System)**
- Satellite-based positioning — receives signals from multiple satellites, triangulates position
- Works outdoors only; poor indoors, urban canyons, or underground
- Used for: Outdoor localization, waypoint navigation
- *Analogy:* Like using the sun and stars to navigate — only works when you have line-of-sight to the sky.

**Wheel Encoders**
- Rotary encoders attached to wheels that count rotations
- Calculate speed, distance traveled, and heading (odometry)
- Used for: Dead reckoning navigation — estimating position from known starting point + movement
- *Analogy:* Like counting your steps while walking blindfolded to estimate how far you've gone.

**Force/Torque Sensors**
- Measure force and torque (twisting force) at robot joints or grippers
- Used for: Delicate object manipulation, tactile feedback
- *Analogy:* The sensitivity in your fingertips that tells you how hard you're gripping something.

**Touch/Tactile Sensors**
- Detect physical contact or pressure
- Used for: Object grasping, collision detection, surface sensing

**Gas and Chemical Sensors**
- Detect specific gases or chemicals in the environment
- Used for: Gas leak detection, environmental monitoring, hazardous area inspection

---

## 5. Actuators — Types and Applications

### What Are Actuators?

**Definition:** Actuators are devices that convert control signals or energy into physical action, motion, or mechanical work. They are the "output" side of a robot — they are what makes the robot *do* things.

*Analogy:* If sensors are eyes and ears, actuators are hands and legs. They interact with the world physically based on the brain's (controller's) instructions.

Key points:
- Take input from sensors or controllers
- Produce mechanical motion: linear (back-and-forth), rotary (spinning), or complex combinations
- Choice depends on: required force, speed, precision, environment, cost

### Types of Actuators

**Electric Motors**
- DC motors, AC motors, stepper motors, servo motors
- Most common in robotics
- Provide controlled rotational motion
- Used for: Wheel propulsion, joint actuation, conveyor systems, consumer electronics
- *Analogy:* The engine in an electric car — converts electrical energy into rotation.

**Pneumatic Actuators**
- Use compressed air (gas) to create linear or rotary motion
- Fast, strong, but imprecise without feedback
- Used for: Factory automation, industrial grippers, pick-and-place robots
- *Analogy:* Like a bicycle pump — push air in, push something out.

**Hydraulic Actuators**
- Use pressurized fluid (oil) to generate very high forces
- Slower but extremely powerful
- Used for: Heavy machinery (excavators, cranes), aerospace (landing gear, flight controls)
- *Analogy:* Like a hydraulic car jack — a small pump creates enormous force.

**Electromagnetic Actuators (Solenoids)**
- Use magnetic fields from electric current to produce motion
- Fast switching (on/off)
- Used for: Valves, relays, automotive systems, robotics
- *Analogy:* The clicking mechanism inside a camera shutter — fast electromagnetic snap.

**Shape Memory Alloy (SMA) Actuators**
- Special alloys that change shape when heated or stressed, then return to original shape when cooled
- Used for: Biomedical devices, micro-robots, aerospace
- *Analogy:* Like a metal that "remembers" its original shape — bend it, heat it, and it springs back.

**Thermal Actuators**
- Use thermal expansion (things expand when heated) to generate motion
- Used for: Micro-scale systems (MEMS), microvalves
- Used in very small-scale applications

**Mechanical Actuators**
- Screws, gears, levers, cams — convert one type of motion to another
- Used as part of larger systems (e.g., a lead screw converting motor rotation to linear motion)
- *Analogy:* A clock's gear train — gears convert motor rotation into slow, precise pointer movement.

---

## 6. Motors and Controllers

### Motors

**Definition:** A motor converts electrical energy into mechanical energy (motion). It exploits the interaction between magnetic fields and electric currents.

#### Types of Motors Used in Autonomous Robotics

**DC Motors (Direct Current)**
- Brushed (physical contact via carbon brushes) or Brushless (electronic switching)
- Simple, controllable, widely used
- Used for: Wheel drive, joint actuation
- Speed controlled by varying voltage/current

**Servo Motors**
- DC motor + built-in gearbox + position sensor (encoder) + feedback controller
- Excellent for precise angular position control
- Used for: Robot arms, grippers, camera pan-tilt systems
- *Analogy:* Like a motor with a ruler attached — it knows exactly where it is and goes exactly where you tell it.

**Stepper Motors**
- Move in precise discrete steps (e.g., 1.8° per step)
- No feedback needed for basic position control
- Used for: CNC machines, 3D printers, precise robot joint control
- *Analogy:* Like a clock hand that clicks to each exact position rather than sweeping continuously.

### Controllers

**Definition:** Controllers are devices or software systems that manage and regulate a robot's behavior — they receive sensor data, make decisions, and generate commands sent to actuators.

*Analogy:* The controller is the brain. It receives signals from eyes and ears (sensors), processes them, and sends signals to hands and legs (actuators).

#### Types of Controllers

**Microcontrollers/Microprocessors**
- Small embedded computers executing control algorithms
- Examples: Arduino (simple tasks), Raspberry Pi, NVIDIA Jetson (AI tasks)
- The "brain" of the robot — processes all data and issues commands

**Motor Controllers**
- Electronic circuits that regulate speed and direction of motors
- Interpret commands from the microcontroller and apply appropriate voltage/current
- Examples: ESC (Electronic Speed Controller), H-Bridge, ROS-compatible motor drivers

**Feedback Systems (Encoders + IMUs)**
- Sense the robot's actual state (position, speed, angle)
- Feed this back to the controller for closed-loop control
- Without feedback: *open-loop control* (like driving blindfolded)
- With feedback: *closed-loop control* (like driving with eyes open, correcting continuously)
- *Analogy:* Feedback is like checking your speedometer while driving and adjusting the gas pedal.

**Motion Planning and Control Algorithms**
- Software (e.g., PID controllers, trajectory planners, reinforcement learning agents)
- Execute navigation, obstacle avoidance, manipulation tasks
- Work with sensor data in real-time

#### Real-World Controller Examples
- Siemens SIMATIC S7 — Industrial PLCs
- Allen-Bradley PLCs — Manufacturing automation
- Arduino Uno, Raspberry Pi — Educational/hobby robots
- ABB IRC5, FANUC R-30iB — Industrial robot controllers

---

## 7. Power Sources and Management

### Why Power is Critical
Autonomous robots must operate independently — they carry their own energy. Battery life directly determines operational endurance. A robot that runs out of power mid-task is useless.

### Power Sources

**Batteries** (most common)
- *Lithium-Ion (Li-ion):* High energy density, rechargeable, moderate weight. Used in mobile robots, humanoids.
- *Lithium-Polymer (LiPo):* Flexible form factor, very high energy density, used in drones (lightweight, high discharge rate)
- *NiMH (Nickel Metal Hydride):* Older, heavier, still used in some applications
- *Lead-Acid:* Heavy, cheap, used where weight isn't critical

**Fuel Cells (Hydrogen Fuel Cells)**
- Convert hydrogen + oxygen directly into electricity + water
- Longer endurance than batteries
- Used in: UAVs requiring long flight times, underwater robots

**Solar Power (Photovoltaic Cells)**
- Solar panels on robot surface convert sunlight → electricity
- Used in: Outdoor robots, environmental monitoring, some UAVs
- Limitation: Requires sunlight; limited power density

**Internal Combustion Engines**
- Gasoline, diesel engines for very high-power, long-endurance applications
- Used in: Large field robots, some military robots

**Tethered Power**
- Robot connected to external power source via cable
- Unlimited operation time, but limited mobility
- Used in: Industrial inspection robots, underwater ROVs, certain manufacturing robots

### Power Management Techniques

**Voltage Regulation** — Keeps voltage stable despite varying load; prevents component damage.

**Current Limiting** — Prevents excessive current that could burn out motors or circuits. Components: fuses, current limiters.

**Battery Management System (BMS)** — Monitors battery charge state, prevents overcharging/deep-discharging, temperature monitoring, balances cells. Extends battery life dramatically.

**Energy Harvesting** — Captures ambient energy (vibrations, heat, light) to supplement or trickle-charge the main battery. Used in low-power sensor nodes.

**Power Distribution Boards** — Cleanly route power from battery to all subsystems (CPU, sensors, motors, communication).

**Sleep Modes / Low-Power States** — Put unused components to sleep when idle. The CPU might sleep between sensor readings. Critical for extending battery life.
- *Analogy:* Your phone dims its screen and disables Wi-Fi when idle — same principle.

**Emergency Shutdown Systems** — Safety mechanism that cuts power in critical situations (robot about to fall, overheating, loss of control signal). Hardware switches + software watchdogs.

---

## 8. Range Finders

### What Are Range Finders?

Range finders measure the *distance* between the robot and objects in its environment. They are the robot's "sixth sense" — constantly pinging the environment to reveal how far away things are.

*Analogy:* Imagine being blindfolded and using a tape measure in every direction to build a mental map of a room. Range finders do this automatically, thousands of times per second.

### Types of Range Finders

**LiDAR (Light Detection and Ranging)** — Best for outdoor navigation, obstacle detection, long-range 3D mapping. Emits laser pulses, measures return time (Time-of-Flight). Generates dense 3D point clouds. Used in self-driving cars.

**Ultrasonic** — Best for short-range tasks (<5m). Cheap and simple. Emits high-frequency sound waves, measures echo return time. Used for proximity sensing, parking systems, indoor robot navigation.

**Radar** — Best for adverse weather conditions (fog, rain, dust, smoke) and long-range detection. Uses radio waves. Penetrates conditions where LiDAR and cameras fail. Used in automotive adaptive cruise control.

**Vision-Based (Stereo Cameras, Depth Cameras)** — Estimates distance through triangulation (two cameras at different positions) or structured light / time-of-flight depth sensors. Used indoors and for human-robot interaction.

### Applications

**Obstacle Avoidance** — A self-driving car's LiDAR spins 360°, scanning 100+ meters ahead. It detects vehicles, pedestrians, and debris in real-time, feeding this map to the car's decision system to brake or steer.

**Precision Manipulation** — A robotic arm uses ultrasonic sensors to measure the exact distance to a fragile component before gripping it, ensuring controlled contact force.

**Mapping and Exploration** — Mars rovers use laser rangefinders to create 3D maps of the Martian terrain before the rover moves, preventing dangerous navigation decisions.

**Search and Rescue** — Drones with radar can see through dust, debris, and smoke in disaster zones to locate survivors.

---

## 9. Encoders

### What Are Encoders?

Encoders are sensors that translate *mechanical motion* (rotation or linear movement) into digital signals that a computer can understand. They track **position, speed, direction, and rotation** of moving parts.

*Analogy:* Imagine marking your car's tire with a red dot and counting how many times that dot passes a fixed point — you're building a primitive encoder. Encoders do this electronically, with thousands of counts per revolution, at very high speed.

### What Encoders Do

**Measure Position and Motion** — Track the exact angle or linear displacement of a joint, wheel, or motor shaft.

**Provide Feedback** — Send this position/speed information to the controller for closed-loop control.

**Enable Closed-Loop Control** — The controller can compare actual position (from encoder) to desired position and make corrections in real-time. Without encoders, you'd have no idea if the motor actually reached the target.

### Types of Encoders

#### Rotary Encoders (for rotating shafts)

**Incremental Encoders**
- Generate pulses as they rotate. Each pulse represents a small angular increment.
- Count pulses to calculate *relative* position from a starting point.
- They don't know absolute position on power-up; they only know how far they've moved since start.
- Sub-types:
  - *Optical:* A disk with slots spins between a light source and photodetector. Slots pass light; gaps block it → pulses.
  - *Magnetic:* Uses magnetic strips and Hall-effect sensors to detect rotation.
- *Analogy:* Like counting floor tiles as you walk — you know you've moved 10 tiles, but not your absolute address.

**Absolute Encoders**
- Output a unique digital code for every possible angular position.
- Know their exact position even after power loss — no homing needed.
- Used where knowing absolute position at all times is critical (e.g., surgical robots, precise industrial arms).
- *Analogy:* Like a GPS address — tells you exactly where you are, not just how far you've moved.

#### Linear Encoders (for linear (straight-line) motion)
- Work on the same principles as rotary, but along a straight scale
- Used in: CNC machines, precision manufacturing, linear robot axes

### Applications of Encoders

**Robotics** — Measure joint angles in robot arms (for forward kinematics), wheel rotations for odometry (dead reckoning navigation).

**Industrial Automation** — Monitor motor speed in assembly lines, CNC machines, conveyor systems.

**Medical Devices** — Control movement in MRI machines, surgical robots, prosthetics.

**Automotive** — ABS (Anti-lock Braking System) uses wheel encoders to detect wheel lockup; power steering uses steering wheel encoders.

**Consumer Electronics** — Volume knobs, scroll wheels in gaming controllers.

### Choosing the Right Encoder: Key Factors
- **Resolution** — How many counts per revolution? More = finer position detail.
- **Accuracy** — How close to true position?
- **Operating Speed** — Can it keep up with fast-moving parts?
- **Environment** — Dust, moisture, vibration resistance?
- **Cost** — Higher precision = higher cost.

---

## 10. Vision Sensors

### The Two Steps of Robotic Vision

**Step 1 — Capturing the world:** Sensors mimic the eye — they collect light data and convert it into digital images. This is the raw input.

**Step 2 — Making sense of the scene:** AI algorithms process these images to extract meaning — depth, motion, color, objects, scene context. This is what makes vision useful.

### Types of Vision Sensors

**Standard Cameras (RGB / Grayscale)**
- Most common, capture color or grayscale images at various resolutions and frame rates
- Used for: Object recognition, tracking, navigation, face detection
- *Analogy:* Your eyes — they give you rich visual information in color, but distance must be inferred.

**Depth Cameras (RGB-D)**
- Capture color image + depth (distance per pixel), creating a 3D scene
- Examples: Intel RealSense, Microsoft Kinect
- Used for: Obstacle avoidance, robot-human interaction, 3D object manipulation
- *Analogy:* Having two eyes (stereo vision) — you can judge distance because each eye sees from a slightly different angle.

**Thermal Cameras**
- Detect infrared radiation (heat) rather than visible light
- Work in complete darkness; identify objects by temperature signature
- Used for: Search and rescue (find warm bodies), security, firefighting robots

**Time-of-Flight (ToF) Sensors**
- Emit modulated light pulses; measure phase shift of reflected light to calculate per-pixel depth
- High-resolution depth data at shorter ranges (<10 m)
- Better for indoor, object manipulation tasks
- More compact and cheaper than LiDAR

**Event Cameras**
- Revolutionary new sensor type — instead of capturing full frames at fixed intervals, they capture *changes in light intensity* at each pixel independently
- Ultra-fast temporal resolution (microseconds vs milliseconds)
- Used for: High-speed moving objects, dynamic environments, low-latency robotics
- *Analogy:* Rather than taking a full photo every 1/30 second, each pixel fires independently the moment it detects any change — like a grid of individual motion detectors.

### Applications of Vision Sensors

**Navigation and Obstacle Avoidance** — Cameras + LiDAR + ToF together create a rich environmental model for safe path planning.

**Object Recognition and Manipulation** — The robot identifies what an object is, its orientation, and plans a grasp accordingly (a robotic hand picking up a cup identifies the cup's handle first).

**Human-Robot Interaction** — Cameras + depth sensors enable gesture recognition, facial expression reading, gaze tracking — all crucial for natural interaction.

**Inspection and Quality Control** — Factory cameras inspect products at sub-millimeter precision, detecting surface defects faster and more consistently than humans.

**Autonomous Vehicles** — Front cameras for lane detection; 360° camera arrays for object tracking; combined with LiDAR and radar for full environmental awareness.

### Choosing a Vision Sensor
- **Application:** What task? Indoors/outdoors?
- **Resolution and Accuracy:** How fine does the detail need to be?
- **Field of View:** Wide-angle for navigation; narrow telephoto for distant object identification.
- **Cost and Power:** Tradeoff between capability and resources.

### Future of Vision Sensors
- Miniaturization → smaller, more embeddable sensors
- AI integration → better real-time understanding of scenes
- Sensor fusion → combining vision with LiDAR, IMU, radar for comprehensive perception

---

## 11. ROS2 — Architecture and Communication

### What is ROS?

**ROS (Robot Operating System)** is *not actually an operating system*. It is an open-source framework and middleware for robot software development. It provides:

- **Communication System:** Publish-subscribe (Pub-Sub) + Remote Procedure Call (RPC/Service)
- **Framework and Tools:** Build system, dependency management, visualization (RViz), data recording and replay
- **Ecosystem:** Language bindings (C++, Python, Java, Go), hardware drivers, simulation (Gazebo), libraries for navigation, perception, motion planning

*Analogy:* ROS is like the Android/iOS platform for robots. Just as Android provides common APIs so app developers don't rebuild the phone's networking from scratch, ROS provides common robot APIs so roboticists don't rebuild communication, visualization, and drivers from scratch.

### ROS 1 vs ROS 2

**ROS 1** — Primarily academic. Single master node (Roscore) managed discovery. Not real-time. Not industrial-grade.

**ROS 2** — Production-ready. Decentralized architecture. Uses **DDS (Data Distribution Service)** middleware — an industry-standard for real-time, distributed communication. No Roscore needed. Multiple language support. Real-time capable.

Key new libraries: `rcl` (C), `rclcpp` (C++), `rclpy` (Python)

### ROS 2 Architecture: The Graph Structure

The fundamental architecture of ROS 2 is called the **Computation Graph** — it consists of:

- **Nodes** — the processing units
- **Topics** — the channels through which nodes share data
- **Messages** — the data types passed through topics
- **Services** — request-response calls between nodes
- **Actions** — long-running tasks with progress feedback

#### Nodes

**Definition:** A Node is the basic unit of computation in ROS 2. Each node is an independent process (or component within a process) that handles one piece of the robot's functionality.

Examples of nodes in a robot:
- LiDAR driver node → publishes range data
- Camera node → publishes images
- SLAM node → subscribes to range+camera, publishes map
- Navigation node → subscribes to map, publishes velocity commands
- Motor controller node → subscribes to velocity commands, drives motors

*Analogy:* Think of a node like a team member in an office. Each person has one job and communicates with others via memos (messages) or phone calls (services). Nobody needs to know everything — they just do their job and pass information along.

Key properties:
- Nodes can run on a single computer or distributed across multiple computers
- Multiple nodes can run within a single OS process
- They discover each other automatically (via DDS) — no central broker needed

#### Topics and Pub-Sub Communication

A **Topic** is a named channel for data streaming. Nodes can **publish** data to a topic or **subscribe** to receive data from a topic.

- One publisher can have many subscribers
- One subscriber can listen to multiple topics
- Topics are decoupled — publisher doesn't know who's subscribing

*Analogy:* A topic is like a radio frequency. A news station broadcasts (publishes) on 101.5 FM. Any radio tuned to 101.5 FM (subscriber) receives the broadcast. The station doesn't know who's listening, and listeners don't know how the station works internally.

Example: A camera node publishes images to `/camera/image_raw`. A computer vision node subscribes to this topic to detect objects.

#### Messages

**Messages** are the data structures used to pass information through topics. They are language-independent typed data definitions.

Example — a camera info message:
```
sensor_msgs/CameraInfo
  Header header (timestamp, frame_id)
  uint32 height, width
  float64[9] K  (intrinsic matrix)
  float64[12] P (projection matrix)
```

Standard messages enable interoperability — a camera from Sony and a camera from Logitech can both publish `sensor_msgs/Image` messages, and any subscriber can handle either without modification.

#### Services

**Services** use a **request-response** model (unlike topics which are one-way streaming). One node sends a request; another responds.

Used for: Discrete, transactional operations — "Are you ready?", "Reset your state", "Get current position"

*Analogy:* A phone call — you call someone, ask something, wait for an answer, hang up.

#### Actions

**Actions** are for **long-running tasks** where you need progress updates.

Flow: Client sends a goal → Server starts the task → Server sends intermediate feedback → Server sends final result when done.

Used for: "Navigate to (x,y,z)", "Pick up the red box", "Scan the room"

*Analogy:* Ordering food at a restaurant — you place an order (goal), the kitchen gives updates ("your starter is ready", "main course in 5 min"), and finally delivers the complete meal (result).

#### QoS (Quality of Service)

DDS provides rich QoS policies that ROS 2 exposes. This determines how reliably data is delivered.

| Profile | Use Case | QoS |
|---|---|---|
| Sensor data | Camera, LiDAR streams | Best-Effort (some loss OK for speed) |
| Commands | Robot motion commands | Reliable (must not be lost) |
| Parameters | Configuration | Reliable |

### Discovery (Decentralized)

In ROS 1: A central service called *Roscore* handled discovery. If Roscore died, the entire robot system collapsed.

In ROS 2 (via DDS): Nodes *dynamically discover each other* using special DDS discovery topics. No central broker. The system is fully decentralized and fault-tolerant. Roscore is retired.

### ROS 2 Filesystem

**Workspace** — A folder where you create, modify, build, and install ROS packages. Tool used: `colcon` (build tool).

**ROS 2 Packages** — The smallest build unit. Two types:
- *Binary packages:* Pre-built, installed via `apt` (Ubuntu's package manager)
- *Source packages:* Your own code or third-party code you compile from source — placed in the `src/` folder of a workspace

### Key Tools

- **RViz** — 3D visualization tool. See robot model, sensor data, planned paths, point clouds in real-time.
- **Gazebo** — Physics-based robot simulator. Test your robot in simulation before deploying on hardware.
- **OpenCV** — Computer vision library, used for image processing in ROS 2 perception nodes.
- **Qt** — GUI library for building user interfaces for ROS 2 applications.

---

## 12. AI-Powered Robotics

### AI Perception Systems

Autonomous robots use AI to make sense of raw sensor data — turning pixels, point clouds, and IMU readings into meaningful understanding of the environment.

**Computer Vision (with AI)**
- Convolutional Neural Networks (CNNs) analyze camera images to identify objects, people, scenes
- Includes: Facial recognition, gesture interpretation, scene understanding
- *Analogy:* Your visual cortex — takes raw light signals and makes you recognize your mother's face.

**Object Detection and Tracking**
- AI locates objects in images (bounding boxes) and tracks their movement across frames
- Differentiates static objects from moving ones
- Used in: Surveillance, logistics (finding packages on conveyors), autonomous driving

**Sensor Fusion**
- AI combines data from multiple sensors (camera + LiDAR + radar + IMU) to create a richer, more robust world model than any single sensor could provide
- Each sensor has weaknesses (cameras fail in dark, LiDAR fails in rain, radar is imprecise) — fusion covers everyone's weaknesses
- *Analogy:* Using all your senses simultaneously — smell, sight, hearing, touch together give you more confidence than any one alone.

**Environmental Mapping and Navigation (SLAM + Deep Learning)**
- AI-enhanced SLAM builds detailed, semantic maps (not just geometry, but "this is a door", "this is a chair")
- Deep learning helps robots recognize previously seen places for re-localization

### AI-Enhanced Sensing

**Neural Networks for Interpretation**
- Raw sensor data (1000s of LiDAR points, megapixel images) is fed into neural networks
- Networks learn to recognize complex patterns (a pedestrian, a stop sign, a pothole)
- Far beyond what traditional rule-based systems can handle

**Real-Time Data Processing**
- AI runs inference (making predictions) on sensor data as fast as it arrives
- Critical for dynamic environments — a pedestrian stepping off a curb must be detected in milliseconds, not seconds

**Intelligent Filtering**
- AI removes noise, irrelevant data, sensor glitches before they corrupt decision-making
- Like having an experienced interpreter who knows when to ignore static on a radio

### Machine Learning Models in Robotics

**Supervised Learning for Task Execution**
- Training with labeled data: input (camera image) → output (object label, bounding box)
- Used for: Object recognition, path classification, manipulation skill learning
- The robot learns from human-labeled examples
- *Analogy:* Studying from a textbook with an answer key — you learn the right answer for each question.

**Unsupervised Learning for Pattern Discovery**
- No labels needed — the algorithm finds structure in raw data
- Used for: Anomaly detection (something unusual happened), clustering sensor patterns, creating internal environment representations
- *Analogy:* Exploring a new city without a guide — you figure out which areas are similar, which are different, based on your own observations.

**Reinforcement Learning (RL) for Autonomous Decision-Making**
- Robot learns by trial and error — it tries actions, receives reward/penalty, and gradually learns optimal behavior
- Used for: Robot locomotion (learning to walk), game playing, navigation in dynamic environments, adaptive manipulation
- *Analogy:* Learning to ride a bicycle — you fall (penalty), adjust, and eventually learn the right balance (reward) through experience, not from reading a manual.

### Deep Learning in Robotics

**CNNs (Convolutional Neural Networks) for Vision**
- Specialized neural network for image processing
- Uses convolution operations to detect edges, shapes, textures, and eventually whole objects
- Used for: Object detection, image classification, lane detection in autonomous driving
- *Analogy:* The visual processing areas of the brain — neurons respond to specific visual features (edges, curves, faces) in a hierarchical way.

**RNNs (Recurrent Neural Networks) for Sequential Data**
- Process sequences of data with memory of previous steps
- Used for: Predicting future robot states, controlling movements based on history, time-series sensor processing
- *Analogy:* Reading a sentence — understanding word 5 requires remembering words 1-4. RNNs maintain this memory.

**Transformer Models for Advanced Reasoning**
- State-of-the-art architecture for understanding context (originally from NLP, now widespread in vision too)
- Enables robots to understand complex instructions, reason about the environment at a high level
- Used for: Language-guided manipulation ("Pick up the blue cup near the sink"), multi-step task planning

### Natural Language Processing (NLP) in Robotics

NLP allows robots to understand and generate human language — enabling natural, voice-based interaction.

**Human-Robot Interaction (HRI)** — Robots engage in natural conversations, answer queries, give status updates.

**Voice Commands** — "Pick up the box", "Go to room 3", "Stop" — hands-free control.

**Instruction Understanding** — Robots parse ambiguous or complex instructions: "Put the thing on the left next to the window" requires context understanding, not just keyword matching.

**Collaborative Communication** — Robots and humans share goals verbally during collaborative tasks.

### AI-Optimized Control Systems

Traditional robot control uses classical algorithms:
- **PID (Proportional-Integral-Derivative) Control** — Most common feedback controller. Simple, robust, fast.
- **LQR (Linear Quadratic Regulator)** — Optimal control using state-space formulations.
- **MPC (Model Predictive Control)** — Predicts future states and optimizes control over a time horizon. Constraint-aware.

AI enhances these by:
- **Adaptive Algorithms** — AI tunes PID/LQR/MPC gains in real-time as conditions change (heavy load, surface change)
- **Predictive Control** — AI improves system models for better MPC predictions
- **Learning-Based Optimization** — RL continuously refines control policies to reduce energy use, wear, and execution time

### Edge AI and On-Robot Processing

**Edge AI** = Running AI inference directly on the robot's hardware (not in the cloud).

Benefits:
- **Low latency** — No round-trip to cloud → millisecond response times
- **Privacy** — Sensitive data (camera feeds of people) never leaves the robot
- **Offline operation** — Works in remote, low-connectivity environments (disaster zones, underground, space)
- **Reduced bandwidth** — No need to stream raw sensor data to cloud

### AI Computing Platforms (Hardware for On-Robot AI)

| Platform | Type | Strengths | Examples |
|---|---|---|---|
| **GPU** | Graphics Processing Unit | Parallel processing for deep learning training & inference. Excels at vision, SLAM, object detection. High compute throughput. | NVIDIA Jetson Nano/Xavier/Orin/Thor |
| **TPU** | Tensor Processing Unit | Specialized for tensor operations (neural networks). High performance-per-watt. Good for fixed, optimized AI models on robots. | Google Coral Edge TPU |
| **NPU** | Neural Processing Unit | Designed for low-power AI inference. Efficient for vision, speech, sensor fusion on embedded/mobile robots. | Qualcomm Snapdragon RB5, ARM Ethos-U, Huawei Ascend |

### AI Applications Across Domains

**Autonomous Driving and Mobile Robots** — Self-driving cars, warehouse AMRs (Autonomous Mobile Robots)

**Collaborative Robots (Cobots)** — Working alongside humans safely; Universal Robots, ABB YuMi, FANUC CR series

**Medical and Healthcare** — Surgical robots (da Vinci), hospital logistics (Moxi by Diligent Robotics), DNA Nanobots

**Agricultural Robotics (AgriBots)** — John Deere autonomous tractors, Blue River precision weeding, Agrobot harvesting

**Defence/Military** — MQ-9 Reaper UAV (ISR), TALON EOD robot (bomb disposal), underwater UUVs (REMUS, Saab Sabertooth)

---

## 13. Locomotion — Fundamentals

### What is Locomotion?

**Locomotion** is the mechanism by which a robot moves through its environment. Without locomotion, a robot is a stationary machine, not a mobile robot.

A mobile robot must be able to move "unbounded throughout its environment" — meaning it shouldn't be restricted to a fixed location like a robotic arm bolted to a factory floor.

### Types of Locomotion in Nature (and Robotics)

**Terrestrial (land-based):**
Walking, galloping, running, hopping, crawling, sliding, rolling

**Flight:**
Flapping wings (birds, insects), gliding (birds, gliders), perching (drones that land on structures)

**Aquatic:**
Undulatory swimming (fish body movement), flagellar propulsion (like bacteria), jet propulsion (squid, jellyfish)

Most robot locomotion is *biologically inspired* — engineers look at how animals move and adapt those mechanisms.

*Note:* While biological systems achieve locomotion through structural replication (cells divide and muscles grow), we cannot easily replicate this in machines. Also, biological energy storage (fat, glucose) and generation (muscles) cannot be trivially transferred to man-made systems.

### Wheeled vs. Legged: Efficiency Comparison

On **flat, hard surfaces:** Wheeled locomotion is 1–2 orders of magnitude more energy-efficient than legged locomotion. Rolling friction is tiny on hard surfaces.

On **soft, rough, or uneven surfaces:** Wheels accumulate rolling friction and sink. Legged locomotion is far better because legs only make point contacts with the ground — the ground quality *between* contact points doesn't matter.

**Efficiency hierarchy** (most to least efficient on flat ground):
1. Railway wheels on steel (virtually no rolling friction)
2. Rubber wheels on hard ground
3. Walking/Running
4. Crawling/Sliding

### Key Issues for Any Locomotion System

**1. Stability**
- Number and geometry of contact points
- Center of gravity (CoG) position
- Static stability (stable when stationary) vs. dynamic stability (stable only when moving)
- Terrain inclination

**2. Contact Characteristics**
- Size and shape of contact patch
- Angle of contact with ground
- Friction (too little → slip; too much → energy loss)

**3. Type of Environment**
- Structured vs. unstructured
- Medium: water, air, soft ground, hard ground, stairs

---

## 14. Legged Mobile Robots

### Overview

Legged locomotion uses a series of *point contacts* with the ground — the robot lifts and places legs sequentially to move forward.

**Advantages:**
- Excellent adaptability on rough, irregular terrain
- Can step over gaps, climb stairs, navigate debris
- Ground quality between contact points is irrelevant — only the contact points themselves matter
- Potential for dexterous object manipulation with legs

**Disadvantages:**
- Higher energy consumption (complex actuation)
- Mechanical complexity (many joints and actuators)
- More complex control (balance, gait planning)
- Generally slower than wheels on smooth terrain

### Stability: Static vs. Dynamic

**Static Stability** — The robot is stable at *every instant*, even if all joints freeze. No active balance required.
- Requires at least 3 ground contact points forming a *support polygon*
- **Support Polygon:** The convex hull of all ground contact points. The CoG must lie within this polygon.
- *Analogy:* A three-legged stool. Push it slightly — when you let go, it returns to stable position on its own. No active balancing needed because gravity keeps the CoG within the tripod.
- To walk statically stable with 4 legs: lift one at a time (3 always on ground) — slow but stable.
- With 6 legs: Can use "tripod gait" — lift 3 alternate legs simultaneously (the other 3 form a stable tripod). Efficient and statically stable.

**Dynamic Stability** — The robot is stable only while in *motion*. It must actively keep moving to prevent falling.
- *Analogy:* A bicycle — stable while moving, falls immediately when stopped without support. Or an inverted pendulum — constantly requires corrections to stay upright.
- Bipedal (human-like) robots are dynamically stable — tiny support polygon (two feet), must constantly shift CoG.
- Allows for high-speed, agile motion
- Requires sophisticated real-time control

**Key Insight:** Static stability requires at minimum 4 legs (so one can always be lifted with 3 remaining). For most practical static walking, 6 legs are used. Two-legged and most four-legged walking is dynamically stable.

### Degrees of Freedom (DOF) per Leg

Minimum to lift and swing a leg forward: **2 DOF** (lift + swing)

More common: **3 DOF** (adds lateral movement for complex maneuvers)

Bipedal robots (humanoid): **4 DOF** per leg (adds ankle joint for ground adaptability)

The human leg: **7+ major DOF** with 15+ muscle groups across 8 complex joints. Extremely sophisticated.

*Rule:* More DOF per leg → greater maneuverability and terrain adaptability, but higher complexity, energy, and control requirements.

### Leg Coordination and Gaits

A **gait** is a periodic pattern of leg lift and release events.

If a robot has **k legs**, the number of distinct possible events is:
> **N = (2k - 1)!**

Examples:
- k = 2 legs: N = (4-1)! = 3! = **6 possible event sequences**
- k = 6 legs: N = (12-1)! = 11! = **39,916,800 possible event sequences**

This shows why six-legged robot control is complex, but also why nature (insects) chose six legs — richness of possible gaits allows adaptation to any terrain.

### One-Legged Robots (Monopods)

- Minimum possible legs; minimum body mass
- Requires *only* point contact with ground
- Can cross gaps wider than its stride (by hopping/running)
- Major challenge: balance — must be dynamically stable at all times, even when stationary
- Must constantly hop or actively balance
- **Example: Raibert's Hopper** — A classic research robot that hops continuously and uses three independent controllers:
  - **Hopping height controller** — maintains bounce amplitude
  - **Velocity controller** — controls forward speed
  - **Attitude controller** — maintains body orientation upright

### Two-Legged Robots (Bipeds)

- Anthropomorphic — resembles humans
- Capable of walking, running, jumping, dancing, climbing stairs
- Inherently dynamically unstable — small support polygon (two feet, nearly a line in single support phase)
- Must constantly shift CoG between footprints
- **ZMP (Zero Moment Point) Approach:** A planning method that ensures the robot's footprint placement keeps the ZMP (the point where all ground reaction forces balance out) within the support polygon.
- Advantage: Smaller total weight than multi-legged; good for human-environment interaction (doors, stairs designed for bipeds)
- Disadvantage: Each leg must support full robot weight; complex balance control

### Four-Legged Robots (Quadrupeds)

- Standing is inherently stable (4 contact points form a quadrilateral support polygon)
- Walking is more complex — shifting CoG during gait planning is required
- Must use *dynamic stability* for practical walking
- **Famous Example: AIBO (Sony)** — dog-like robot with color camera, sound detection, head tap sensor, emotional state lighting. Walking style simulates learning and maturation.
- **Modern Example: Boston Dynamics Spot** — highly capable quadruped used in industrial inspection, research.
- Configuration space: 5 positions per leg × 2 states (up/down) × 4 legs = 10,000 configurations (not all stable)

### Six-Legged Robots (Hexapods)

- Most popular configuration for statically stable legged locomotion
- **Tripod Gait:** Move 3 alternate legs simultaneously (front-right + middle-left + rear-right, then front-left + middle-right + rear-left). At all times, 3 legs form a stable tripod on the ground.
- Inspired by insects — highly successful terrestrial locomotors that can even walk upside down
- Each leg typically has 3 DOF (hip flexion, knee flexion, hip abduction)
- Control complexity: Coordinating 3 DOF across 6 legs (18 total DOF) is challenging, even though stability is simpler to achieve

### Legged vs. Wheeled Comparison (Detailed)

| Feature | Legged | Wheeled |
|---|---|---|
| Terrain adaptability | Excellent (rough, stairs, gaps) | Best on flat, smooth surfaces |
| Energy efficiency | Lower (complex actuation) | Higher on smooth terrain |
| Control complexity | High (balance, gait planning) | Lower (fewer DOF) |
| Speed | Lower | Higher on smooth terrain |
| Stability | Complex (bipeds) to easy (hexapods) | Inherently stable (3+ wheels) |
| Mechanical complexity | High | Low |
| Cost | High | Low |
| Applications | Rough terrain, exploration, humanoid | Warehouses, roads, indoor service |

---

## 15. Wheeled Mobile Robots

### Why Wheels Dominate

Wheels are the most popular locomotion mechanism in mobile robotics because:
- Simple mechanical implementation
- No balance problem if ≥3 wheels (all always in ground contact)
- Very energy-efficient, even at high speeds
- Mature technology with high reliability

Wheel research focuses on: traction, stability, maneuverability, and control.

### Types of Wheels

**Standard Wheel (2 DOF)**
- Rotates around the wheel axle (forward/backward)
- Steerable around a vertical contact axis
- Highly directional — must be steered before changing direction
- *Analogy:* A bicycle wheel — can only roll forward/backward along its axis.

**Castor Wheel (2 DOF)**
- Rotates around the wheel axle
- Swivels around an offset vertical axis (the swivel joint is *offset* from the contact point)
- Passive — follows direction of travel; no driving capability
- *Analogy:* The rear wheels of a supermarket trolley — they swivel freely and follow where you push.
- Issue during steering: The offset axis causes small forces on the chassis.

**Swedish Wheel / Omni Wheel (3 DOF)**
- Rotates around wheel axle (primary direction)
- Has small passive rollers around the rim that allow low-resistance motion perpendicular to the wheel axis
- Provides omnidirectional movement without requiring wheel steering
- 45° Swedish wheel: rollers at 45° to wheel axis; 90° (Omni) wheel: rollers perpendicular
- *Analogy:* Imagine a wheel with little conveyor belts around its rim — you can roll forward AND slide sideways simultaneously.

**Spherical / Ball Wheel (3 DOF)**
- Omnidirectional — can roll in any direction
- Technically challenging to implement (hard to drive a sphere reliably)
- Uses powered rollers pressing against the sphere to impart force
- *Analogy:* A computer mouse ball that you can spin in any direction — the rollers inside feel its motion.

### Wheel Geometry and Robot Configurations

The combination of wheel types and their arrangement determines the robot's three key characteristics:

**Maneuverability** — Can the robot reach any position and orientation?

**Controllability** — How easily and precisely can the robot's motion be directed?

**Stability** — Does the robot stay upright and not tip?

*Key Trade-off:* High maneuverability often comes at the cost of controllability, and vice versa.

#### Key Configurations

**Differential Drive (2 powered wheels + caster)**
- Most popular for indoor mobile robots
- Two wheels on a common axis, independently driven
- Turn by driving wheels at different speeds (inner wheel slower = turn toward it)
- Simple, cheap, effective
- *Analogy:* A tank — turn left by slowing the left track.
- Rotation center: midpoint between the two wheels

**Ackermann Steering (car-like)**
- 2 steerable front wheels + 2 driven rear wheels
- Front wheels steer along different radii (inner wheel turns more sharply) — reduces tire scrubbing
- Excellent controllability and stability
- Difficult to achieve holonomic (zero-radius) turning
- *Analogy:* A regular car.

**Synchro Drive**
- 3 wheels, all driven and steered by just 2 motors (one for translation, one for steering)
- One motor drives all wheels at the same speed via a belt
- Another motor steers all wheels simultaneously around their vertical axes
- Robot body orientation does NOT change when the robot turns (chassis drifts over time due to slippage — dead-reckoning error)
- Advantage: Simple 2-motor design, can move in any direction
- Disadvantage: Cannot control chassis orientation; belt backlash causes orientation errors in dead-reckoning

**Four-Castor Wheel Configuration**
- 4 castor wheels, each actively steered and powered
- Truly omnidirectional; circular chassis can spin without changing footprint
- Overcomes ground-clearance limitations of Swedish wheels

**Swedish Wheel (Omni) Configurations**
- 3 or 4 Swedish wheels in triangular/square arrangement
- Holonomic (can move in any direction without steering)
- Very maneuverable but hard to control precisely (all speeds must be exactly matched)
- *Example: Carnegie Mellon Uranus* — four Swedish wheels; driving perfectly straight requires exact same speed on all four wheels; small errors cause drift.

### Maneuverability

**Holonomic robots:** Can move in any direction at any time, regardless of their current orientation. Require 3-DOF wheels (Swedish or spherical). Ground clearance is limited due to wheel complexity.

**Non-holonomic robots:** Cannot move in all directions independently (e.g., a car cannot slide sideways). Must rotate to change direction. More common, more practical, easier to implement.

### Controllability

**Definition:** The degree to which the robot's motion can be precisely governed by control inputs.

**General Principle:** High maneuverability ↔ lower controllability. High controllability ↔ lower maneuverability.

- Ackermann (car-like): Excellent controllability, limited maneuverability
- Swedish wheel omni-drive: Excellent maneuverability, requires precise speed matching (harder to control)
- Synchro drive: Good maneuverability, poor controllability of chassis orientation

### Legged vs. Wheeled Summary

Already covered in Section 14 (comparison table). Core message: Wheels win on flat ground; legs win on rough terrain.

---

## 16. Aerial Mobile Robots (UAVs)

### What Are Aerial Mobile Robots?

**UAVs (Unmanned Aerial Vehicles)** — robotic systems designed to navigate and operate in the air. Unlike ground robots, they move in 3D space, covering large areas and accessing difficult terrain.

*Analogy:* A UAV is to a ground robot what a helicopter is to a car — freed from road constraints, but must actively fight gravity to stay airborne.

### Aircraft Classifications

**Lighter Than Air (LTA)**
- Blimps, aerostats
- Buoyancy provides lift — no need to constantly generate lift force
- Advantage: Energy-efficient hover, autolift, simple control
- Disadvantage: Large, slow, weather-sensitive

**Heavier Than Air (HTA)**
- Must actively generate lift (propellers, wings)
- Sub-categories:

**VTOL (Vertical Take-Off and Landing)** — Includes helicopters, multirotors (quadcopters, hexacopters)
- Key advantage: Can hover, take off/land vertically, fly slowly or stationary
- Most promising for miniaturization
- Examples: Quadcopters (4 rotors), Hexacopters (6 rotors), Octocopters (8 rotors)

**Fixed-Wing** — Traditional airplane configuration
- Efficient for long-range, high-speed flight
- Must maintain forward speed to generate lift (cannot hover)
- Examples: Military surveillance drones, agricultural survey planes

**Hybrid VTOL** — Takes off vertically, transitions to fixed-wing flight
- Best of both worlds: VTOL convenience + fixed-wing efficiency
- Complex transition control

### Common MAV (Micro Aerial Vehicle) Configurations

The most common configurations for small autonomous aerial robots:
- **Quadrotor (quadcopter):** 4 rotors in X or + configuration. Most popular due to simplicity, symmetry, and controllability.
- **Hexarotor:** 6 rotors. More payload capacity, redundancy (can lose one motor and still fly).
- **Octorotor:** 8 rotors. Even more payload, full motor redundancy.
- **Coaxial:** Two rotors stacked vertically, counter-rotating. Compact form factor.
- **Fixed-wing with elevons:** More efficient for distance, less agile.

### Key Features of Aerial Robots

**Autonomous Flight** — Navigation systems + sensors enable pre-planned or adaptive flight paths. The UAV follows waypoints or responds to sensor feedback.

**Multisensor Integration** — Camera (forward, downward, 360°), LiDAR, GPS, IMU — necessary for navigation, mapping, and obstacle avoidance in 3D space.

**Communication and Connectivity** — Radio link (telemetry, control), Wi-Fi, 4G/5G for longer range. Critical for maintaining control and data link.

**Energy Efficiency** — Drones fight gravity continuously → high energy consumption. Battery technology and lightweight materials are critical. Typical flight time: 20–40 min for consumer drones; hours for fuel cell drones.

### Applications

**Surveillance and Monitoring** — Police, border patrol, infrastructure inspection (power lines, pipelines)

**Mapping and Surveying** — High-resolution aerial maps, 3D models of buildings and terrain, construction site monitoring

**Search and Rescue** — Thermal cameras find survivors; radar penetrates rubble

**Delivery Services** — Amazon Prime Air, Wing (Google) for last-mile delivery to homes

**Agriculture** — Crop health monitoring (multispectral cameras), precision pesticide spraying (significantly reduces chemical usage), seed planting

---

## 17. Degrees of Freedom (DOF)

### What is DOF?

**Definition:** Degrees of Freedom (DOF) — also called *mobility M* — is the number of independent parameters needed to completely specify the position and orientation of a rigid body or mechanism.

*Analogy:* Think about how you'd describe a drone's position in the air. You need 6 numbers: X (left/right), Y (forward/backward), Z (up/down), Roll (tilt side to side), Pitch (tilt forward/backward), Yaw (rotate left/right). That's 6 DOF — the maximum for a rigid body in 3D space.

A train on a straight track has 1 DOF (it can only move forward/backward along the track).
A car on a flat road has 3 DOF (X, Y position, and orientation/heading angle).
A flying drone has 6 DOF.

### DOF Formula for Mechanisms

> **M = Σ fi** (sum over all joints)

Where `fi` is the mobility of each joint:
- **Revolute joint** (hinge, pin joint): fi = 1 (one rotation angle)
- **Prismatic joint** (sliding joint): fi = 1 (one linear displacement)
- **Helical joint** (screw): fi = 1 (rotation and translation coupled)
- **Spherical joint** (ball-and-socket): fi = 3 (rotation around 3 axes, like a shoulder)

### Joint Types and Their DOF

**Revolute Joint** — Rotates around a single axis (like a door hinge). 1 DOF = the joint angle θ.

**Prismatic Joint** — Slides along a single axis (like a piston or drawer). 1 DOF = linear displacement d.

**Spherical Joint** — Rotates freely around 3 perpendicular axes (like a human shoulder or hip). 3 DOF = represented by Euler angles.

### DOF Classifications for Robots

**2-DOF Robots** — Move in a plane (x, y) and rotate around the vertical axis. Simple planar tasks.
- Example: Planar 2-joint arm (SCARA type for simple assembly)

**3-DOF Robots** — Add the vertical axis. Can reach positions in 3D space but with limited orientation control.
- Example: Spherical coordinate robot; simple pick-and-place with limited wrist orientation

**5-DOF Robots** — 3 translational + 2 rotational. Can reach positions and partially control orientation.
- Example: Serial manipulator with 5 joints; some welding robots

**6-DOF Robots** — Full control over position (3 DOF) and orientation (3 DOF) of the end-effector.
- Example: Most industrial robot arms (FANUC, ABB, KUKA)
- Can reach any position and orient the tool in any way within the workspace

### DOF for Mobile Robots on a Plane

A robot on a flat plane has **3 DOF in Cartesian space**: x position, y position, and heading angle θ.

Only robots using **3-DOF wheels** (Swedish, spherical) can freely control all three independently at all times (holonomic motion).

Robots with standard or castor wheels have *kinematic constraints* — they cannot move in all 3 DOF simultaneously (e.g., a car cannot slide sideways).

### Applications by DOF

| DOF | Applications |
|---|---|
| 2 | Simple 2D tasks, basic conveyors |
| 3 | Simple pick-and-place, surface treatment |
| 5 | Assembly, some welding, material handling |
| 6 | Universal industrial automation, surgical robots, painting, inspection |

---

## 18. Robot Kinematics — Forward and Inverse

### What is Kinematics?

**Etymology:** From Greek *kinema* = motion.

**Definition:** Kinematics is the study of motion *without considering the forces or moments that cause it*. It describes *how* a robot moves (geometry), not *why* it moves (forces).

*Analogy:* Kinematics is like studying a bird's flight path — tracking how it turns, rises, descends — without caring about the aerodynamics of its wings or the force of gravity. Just the geometry of motion.

In robotics, kinematics explores how a robot's **joint coordinates** (angles, extensions) relate to its **spatial arrangement** (position and orientation of links and end-effector).

### Why Do We Need Kinematics?
- To control where the robot's end-effector goes in space
- To plan how the robot moves tools between specific points
- To check for collisions before the robot actually moves
- To understand the robot's workspace (reachable positions)

**Assumption:** Each joint has one degree of freedom — either a rotation angle (revolute joint) or a linear displacement (prismatic joint).

### Forward Kinematics (FK)

**Definition:** Given the joint angles/displacements, compute the position and orientation of the end-effector.

> **Input:** Joint angles (θ₁, θ₂, ..., θₙ) → **Output:** End-effector pose (x, y, z, roll, pitch, yaw)

*Analogy:* You know exactly how much you've bent each finger — forward kinematics tells you exactly where your fingertip is in 3D space.

**How it works:**
- Each link of the robot transforms the coordinate frame from one joint to the next
- Using *transformation matrices* (specifically, Homogeneous Transformation Matrices), you chain these transformations together
- The final result gives you the end-effector's position and orientation in the base (world) frame
- The process is deterministic — one unique answer for any set of joint angles

**Applications:**
- Simulation: Given a planned joint trajectory, visualize where the robot goes
- Verification: Before moving, compute where the robot will end up
- Workspace analysis: What positions can the robot reach?

### Inverse Kinematics (IK)

**Definition:** Given a desired position and orientation of the end-effector, compute the joint angles required to achieve it.

> **Input:** Desired end-effector pose (x, y, z, roll, pitch, yaw) → **Output:** Joint angles (θ₁, θ₂, ..., θₙ)

*Analogy:* You know where you want your fingertip to be — inverse kinematics figures out how to bend each finger joint to get your fingertip there.

**Why it's harder than FK:**
- FK is a direct mapping (one input → one output)
- IK is an inverse problem — may have *multiple solutions* (different joint configurations reaching the same end-effector pose), *no solution* (point outside workspace), or require iterative optimization
- For robots with >6 DOF, the system is *redundant* (infinitely many solutions)

**How it works:**
- **Analytical (closed-form) solutions:** For simple geometries (2-link planar arm), derive explicit equations
- **Numerical/Iterative methods:** For complex robots, use optimization algorithms (Jacobian pseudoinverse, Newton-Raphson) that iteratively converge to a solution
- Constraints must be considered: joint limits, singularities (configurations where the robot loses a DOF), collision avoidance

**Applications:**
- Motion planning: "Move the gripper to position (0.4, 0.2, 0.5) m, pointing downward"
- Teleoperation: Move end-effector by moving a 3D joystick
- Animation and simulation
- Surgical robots (precise end-effector placement)

### FK vs. IK: Comparison Table

| Property | Forward Kinematics (FK) | Inverse Kinematics (IK) |
|---|---|---|
| **Given** | Joint angles, link lengths | Link lengths, desired end-effector pose |
| **Find** | End-effector position and orientation | Required joint angles |
| **Difficulty** | Simple, direct | Complex, may have multiple or no solutions |
| **Uniqueness** | One unique answer | Multiple solutions possible |
| **Use case** | Simulation, verification | Motion planning, control |
| **Method** | Matrix multiplication chain | Analytical or iterative algorithms |

---

## 19. Mathematical Foundations for Kinematics

### Vector Operations Review

**Dot Product (Scalar Product)**
> **A · B = |A| |B| cos θ**

In component form:
> A · B = aₓbₓ + aᵧbᵧ

Used to find the projection of one vector onto another — critical for computing components of vectors in rotated coordinate frames.

**Unit Vector**
> û_B = B / |B|

A vector of magnitude 1 pointing in the direction of B. Used as basis vectors to express components.

**Matrix Multiplication**
- (m×n) matrix A × (n×p) matrix B = (m×p) matrix
- **Non-commutative:** AB ≠ BA in general
- Used for chaining coordinate transformations

### Coordinate Frames and Transformations

A robot's parts exist in different coordinate frames. To describe where the end-effector is in the world frame, we must *transform* between frames.

**Translation**
If frame NO is displaced by (Pₓ, Pᵧ) from frame XY, then a point V in NO coordinates becomes in XY:

> V_XY = P + V_NO

**Rotation (about Z-axis by angle θ)**
A point V_NO expressed in a frame rotated by θ becomes in the original XY frame:

```
[Vx]   [cos θ  -sin θ] [VN]
[Vy] = [sin θ   cos θ] [VO]
```

This **2D rotation matrix** is used to convert coordinates between rotated frames.

**Combined Translation and Rotation**

```
[Vx]   [cos θ  -sin θ  Px] [VN]
[Vy] = [sin θ   cos θ  Py] [VO]
[1 ]   [  0       0    1 ] [1 ]
```

This is the **Homogeneous Transformation Matrix H** — it encodes both rotation and translation in a single matrix multiplication.

### Homogeneous Transformation Matrices

**Why homogeneous?** Adding an extra dimension (making 2D a 3×3 matrix, 3D a 4×4 matrix) allows translations and rotations to both be expressed as matrix multiplications, which can then be chained.

**2D Homogeneous Matrix (translation + rotation about Z):**

```
H = [cosθ  -sinθ  Px]
    [sinθ   cosθ  Py]
    [0      0     1 ]
```

**3D Rotation Matrices:**

Rotation about Z-axis:
```
Rz = [cosθ  -sinθ  0]
     [sinθ   cosθ  0]
     [0      0     1]
```

Rotation about Y-axis:
```
Ry = [cosθ   0  sinθ]
     [0      1  0   ]
     [-sinθ  0  cosθ]
```

Rotation about X-axis:
```
Rx = [1   0     0   ]
     [0  cosθ  -sinθ]
     [0  sinθ   cosθ]
```

**3D Homogeneous Matrix (4×4):**

Pure Translation:
```
H = [1  0  0  Px]
    [0  1  0  Py]
    [0  0  1  Pz]
    [0  0  0  1 ]
```

Pure Rotation:
```
H = [nx  ox  ax  0]
    [ny  oy  ay  0]
    [nz  oz  az  0]
    [0   0   0   1]
```

Where (n, o, a) are unit vectors along the X, Y, Z axes of the rotated frame expressed in the reference frame.

Combined (rotation + translation):
```
H = [nx  ox  ax  Px]
    [ny  oy  ay  Py]
    [nz  oz  az  Pz]
    [0   0   0   1 ]
```

### Chaining Transformations (Forward Kinematics Procedure)

For a robot arm with n links:
1. Define a transformation matrix Hᵢ for each joint i (describing how link i+1 relates to link i)
2. Multiply all matrices in sequence: H_total = H₁ × H₂ × ... × Hₙ
3. The result gives the end-effector's position and orientation in the base frame

**Critical note:** Order matters. Translation then rotation ≠ rotation then translation. Always apply transformations from right to left (innermost first).

### Applying to a Planar Robot Arm (2-Link Example)

Given: Link lengths L1, L2. Joint angles θ1, θ2.

**FK (Geometric approach):**
- End of link 1: (L1·cos θ1, L1·sin θ1)
- End of link 2 (end-effector): (L1·cos θ1 + L2·cos(θ1+θ2), L1·sin θ1 + L2·sin(θ1+θ2))

**FK (Algebraic/Matrix approach):**
- Build H1 = Rot(θ1) × Trans(L1, 0)
- Build H2 = Rot(θ2) × Trans(L2, 0)
- H_total = H1 × H2
- Extract end-effector position from the last column of H_total

### Inverse Kinematics Example (2-Link Planar Arm)

Given: Desired end-effector position (x, y). Find θ1 and θ2.

Using the law of cosines:

**Step 1 — Find θ2:**
> cos θ2 = (x² + y² - L1² - L2²) / (2·L1·L2)
> θ2 = ± arccos(...)   ← Two solutions! Elbow-up and elbow-down configurations.

**Step 2 — Find θ1:**
> θ1 = arctan(y/x) - arctan(L2·sin θ2 / (L1 + L2·cos θ2))

This illustrates why IK is harder: there are typically two solutions (elbow up / elbow down) and complex trigonometry involved even for a simple 2-link case.

### Configuration Space vs. Cartesian Space

**Joint Space** — The space of all joint angle vectors (θ₁, θ₂, ..., θₙ). The robot "lives" here internally.

**Cartesian Space (Task Space)** — The 3D space where the end-effector operates. The robot "works" here externally.

**FK** maps Joint Space → Cartesian Space

**IK** maps Cartesian Space → Joint Space

Path planning can be done in either space:
- Planning in Joint Space: Simple (interpolate joint angles), but Cartesian path may be unexpected
- Planning in Cartesian Space: Intuitive (straight-line tool motion), but requires solving IK at each step along the path

---

## Quick Reference Summary

| Topic | Key Concept | One-Line Definition |
|---|---|---|
| Autonomous Robot | Self-operating machine | Perceive → Decide → Act without human control |
| Sensor | Input device | Converts physical world → electronic signals |
| Actuator | Output device | Converts control signals → physical motion |
| Controller | Brain | Processes sensors → commands actuators |
| SLAM | Mapping + Localization | Build map and find position simultaneously |
| ROS2 | Robot OS | Middleware for distributed robot software |
| Node (ROS2) | Processing unit | Single-purpose software component |
| Topic (ROS2) | Data channel | Pub-Sub stream of typed messages |
| Forward Kinematics | Joint angles → position | Given poses, find end-effector location |
| Inverse Kinematics | Position → joint angles | Given target, find required joint poses |
| DOF | Degrees of Freedom | Number of independent motion parameters |
| Static Stability | Always stable | CoG within support polygon at all times |
| Dynamic Stability | Stable in motion | Must actively balance; allows agile motion |
| Support Polygon | Stability boundary | Convex hull of ground contact points |
| Tripod Gait | 6-legged walking | Alternate tripods always in contact |
| ZMP | Balance point (bipeds | Point where ground reaction = 0 moment |
| Holonomic | Any-direction motion | Can move in x, y, θ independently |
| LiDAR | 3D laser scanner | Measures distances with pulsed laser light |
| IMU | Inertial sensor | Measures acceleration + angular velocity |
| Edge AI | On-robot AI | AI inference on local hardware, no cloud |
| CNN | Visual neural net | Processes images for object recognition |
| RL | Trial-and-error learning | Learns behavior by reward/penalty |
| DDS | ROS2 middleware | Decentralized data distribution service |

---

*Notes compiled from PES University MAR slides (UE23CS343BB7) — Course by Dr. Ashok Kumar Patil.*
*All slides content covered; supplemented with explanations, analogies, and extended context for complete understanding.*
