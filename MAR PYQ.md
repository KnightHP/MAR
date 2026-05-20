# UE21CS343BB7 — Mobile and Autonomous Robotics
## END SEMESTER ASSESSMENT (ESA) — Jan–May 2024
### Full Solved Paper | PES University | Total Marks: 100

---

> **How to use this document:** Every question has a **direct exam answer** (concise, marks-worthy) followed by a **deep explanation** of the underlying concepts for complete understanding.

---

## QUESTION 1

---

### Q1.a — Technical, Social, and Ethical Challenges in Autonomous Robots (7 Marks)

**Answer:**

Autonomous robots must overcome challenges at three distinct levels — technical (can we build it?), social (how does it fit into human life?), and ethical (should we build it this way?).

---

#### 1. Technical Challenges

These are engineering and scientific problems that limit what autonomous robots can currently do.

**a) Perception and Sensing**
Robots must accurately interpret the world through imperfect sensors. Sensor noise, aliasing (two different locations producing identical readings), and adverse conditions (rain blinding LiDAR, darkness blinding cameras) make reliable perception hard.
- *Example:* A self-driving car's LiDAR system fails in heavy rain — water droplets scatter the laser pulses, drastically reducing detection range. The car cannot safely navigate without a fallback strategy.

**b) Autonomy and Decision-Making**
Robots must handle unpredictable, real-world situations that designers never anticipated.
- *Example:* A surgical robot encounters unexpected anatomy (organ in an unusual position). Its pre-programmed protocols don't cover this case — it must decide whether to proceed, pause, or alert the surgeon. Wrong decisions have life-threatening consequences.

**c) Safety and Reliability**
Critical systems must operate flawlessly even in degraded conditions (sensor failure, hardware faults, software bugs).
- *Example:* An autonomous warehouse robot must still navigate safely if one wheel encoder fails — it must detect the fault, raise an alert, and stop safely rather than crashing into shelving or injuring workers.

**d) Power and Computational Constraints**
Mobile robots carry limited batteries. Running AI inference (deep neural networks for vision, SLAM) in real-time demands enormous computation — but powerful processors consume significant power.
- *Example:* A delivery drone must balance flight time (battery drain), processing power (obstacle detection), and payload — all competing for the same energy budget.

---

#### 2. Social Challenges

These are challenges arising from robots interacting with human society.

**a) Job Displacement**
Automation powered by autonomous robots displaces workers in manufacturing, logistics, delivery, and healthcare support roles.
- *Example:* Amazon's Kiva warehouse robots handle order fulfillment tasks that previously required thousands of warehouse pickers. Workers displaced by these systems may lack skills for available replacement jobs — creating economic hardship and inequality during transition.

**b) Human-Robot Interaction (HRI)**
Robots must communicate their intentions clearly to humans who share the same space. Misunderstandings between humans and robots can cause accidents or anxiety.
- *Example:* An elder-care robot must signal clearly when it is about to perform physical assistance (lifting a patient) to avoid startling them. Unexpected physical contact from a robot can cause falls or psychological distress.

**c) Trust and Acceptance**
Humans must be willing to work alongside, or be cared for by, robots. Acceptance varies widely by culture, age group, and prior experience with technology.
- *Example:* Studies show elderly patients in Japan accept robotic care assistants more readily than elderly patients in some Western countries, where physical care from a machine feels depersonalizing. Designers must account for cultural expectations.

---

#### 3. Ethical Challenges

These are moral questions about how robots should be designed, deployed, and governed.

**a) Accountability and Liability**
When an autonomous robot causes harm, who is responsible — the manufacturer, the programmer, the operator, or the owner?
- *Example:* A self-driving car's decision algorithm decides to swerve into a motorcycle rider rather than brake into a crowd of pedestrians. The car strikes the motorcyclist. Who is criminally liable? Current legal frameworks were not designed for this.

**b) Algorithmic Bias and Fairness**
AI systems trained on biased data can make discriminatory decisions.
- *Example:* A hiring robot trained on historical data from male-dominated industries may systematically underrank female candidates. A policing robot trained on biased arrest records may unfairly flag individuals from certain demographics.

**c) Privacy and Surveillance**
Robots equipped with cameras and microphones operating in homes, hospitals, or public spaces raise serious privacy concerns.
- *Example:* A home security robot continuously streams video of family activities. Who controls this data? Can the manufacturer sell it to advertisers? Can law enforcement access it without a warrant?

**d) Autonomous Weapons**
Lethal autonomous weapons systems (LAWS) — robots that can select and engage human targets without human oversight — raise profound ethical questions about removing humans from life-and-death decisions.
- *Example:* An autonomous military drone programmed to engage any vehicle approaching a boundary. A civilian ambulance crosses the boundary — the robot has no mechanism to request clarification from a human before engaging.

---

**Summary Table:**

| Category | Challenge | Example |
|---|---|---|
| Technical | Perception failure | LiDAR blinded by rain |
| Technical | Unexpected situations | Surgical robot with unusual anatomy |
| Technical | Safety/reliability | Encoder failure in warehouse |
| Social | Job displacement | Kiva robots replacing warehouse workers |
| Social | HRI misunderstanding | Elder-care robot startles patient |
| Ethical | Liability | Self-driving car accident |
| Ethical | Bias | Hiring robot discriminating |
| Ethical | Autonomous weapons | Drone engaging civilians |

---

> **Deep Explanation:** These three challenge categories interact. Technical solutions (better sensors) can address social concerns (fewer accidents → greater trust). Ethical frameworks (liability law) shape what technical features manufacturers prioritize (adding emergency stop systems). Social acceptance determines which technical investments are economically viable. Autonomous robotics cannot be designed by engineers alone — it requires lawyers, ethicists, sociologists, and policymakers working together.

---

### Q1.b — Forward and Inverse Kinematics (7 Marks)

**Answer:**

#### Definitions

**Kinematics** is the study of motion without consideration of the forces that cause it. In robotics, kinematics describes the geometric relationship between joint angles (or displacements) and the position/orientation of the robot's end-effector (its tool or hand).

**Forward Kinematics (FK):**
> Given the joint angles/displacements → compute the end-effector's position and orientation.

**Inverse Kinematics (IK):**
> Given the desired end-effector position and orientation → compute the joint angles required to achieve it.

---

#### Forward Kinematics (FK) — Explained

FK is conceptually straightforward: if you know how each link is oriented (joint angle θᵢ) and how long each link is (Lᵢ), you can calculate where the end-effector ends up.

**2-Link Planar Arm Example:**

For a robot arm with two links (L₁, L₂) and joint angles (θ₁, θ₂):

```
End-effector X = L₁·cos(θ₁) + L₂·cos(θ₁ + θ₂)
End-effector Y = L₁·sin(θ₁) + L₂·sin(θ₁ + θ₂)
```

FK is computed using **Homogeneous Transformation Matrices (HTM)**. Each joint contributes one transformation matrix. The total transformation is the product of all joint matrices:

```
H_total = H₁ × H₂ × ... × Hₙ
```

The final column of H_total gives the (x, y, z) position of the end-effector; the upper-left 3×3 submatrix gives its orientation.

**Properties of FK:**
- One unique answer for any set of joint angles
- Computationally simple (direct matrix multiplication)
- Used for: simulation, workspace analysis, verification of planned trajectories

---

#### Inverse Kinematics (IK) — Explained

IK is the reverse problem: given where you want the end-effector to be, find the joint angles.

**Why IK is harder than FK:**

| Issue | Explanation |
|---|---|
| Multiple solutions | Elbow-up and elbow-down configurations both reach the same point |
| No solution | Target point is outside the robot's workspace (too far, or joint limits prevent it) |
| Infinite solutions | Robot has more DOF than needed for the task (redundant robots — e.g., 7-DOF arm reaching a 6-DOF target) |
| Singularities | Configurations where the robot loses a degree of freedom — e.g., fully extended arm can only move in a circle, not radially |

**2-Link Arm IK (using law of cosines):**

Given desired position (x, y):

**Step 1 — Find θ₂:**
```
cos(θ₂) = (x² + y² − L₁² − L₂²) / (2·L₁·L₂)
θ₂ = ± arccos(...)    ← Two solutions: elbow-up (+) and elbow-down (−)
```

**Step 2 — Find θ₁:**
```
θ₁ = arctan(y/x) − arctan(L₂·sin(θ₂) / (L₁ + L₂·cos(θ₂)))
```

**IK Solution Methods:**

**1. Analytical (Closed-Form):** Direct trigonometric equations. Fast, exact, but only works for simple robot geometries (2-3 links, specific configurations).

**2. Numerical/Iterative:** Algorithms like Newton-Raphson or Jacobian pseudoinverse methods. Work for any robot geometry. Converge to a solution iteratively. May find different solutions depending on starting guess.

**3. Jacobian Pseudoinverse:**
The Jacobian J maps joint velocity to end-effector velocity:
```
ẋ = J · θ̇   (end-effector velocity = Jacobian × joint velocity)
```
Invert this: `θ̇ = J⁺ · ẋ` where J⁺ is the Moore-Penrose pseudoinverse. Integrate θ̇ to find θ.

---

#### Importance of IK in Robot Arm Manipulators

In practical applications, the robot operator (or the task planner) thinks in *Cartesian space* — "Move the welding torch to position (0.5, 0.3, 0.8) m, pointing downward at 45°." They do not manually compute joint angles.

IK bridges this gap. Applications:
- **Assembly:** "Place screw at location (x, y, z)" → IK converts this to actuator commands
- **Surgical robotics:** Surgeon moves a haptic handle in Cartesian space → IK computes joint angles for the robot arm at 1000 Hz
- **Teleoperation:** Remote operator drives the end-effector; IK runs in real-time
- **Animation:** Game characters' limbs are positioned using IK to match terrain (foot plants on uneven ground)

---

> **Deep Explanation:** FK and IK are dual problems. FK maps from the robot's internal "language" (joint angles — the space where motors actually exist) to the external "language" (Cartesian space — where tasks are defined). IK maps back. Most control happens in a loop: Task planner generates Cartesian targets → IK converts to joint targets → PID controllers track joint targets → encoders feed back actual joint angles → FK converts these back to Cartesian for monitoring. Understanding both transforms is essential to designing this entire control loop.

---

### Q1.c — Role and Functionality of Encoders in Robotics (7 Marks)

**Answer:**

#### What Are Encoders?

**Encoders** are sensors that convert mechanical motion (rotation or linear displacement) into digital signals that a computer can process. In robotics, they are the primary means of measuring joint angles, wheel rotations, and linear displacements — giving the robot precise feedback about its own physical state.

---

#### Role of Encoders in Robotics

**1. Position and Velocity Measurement**
Encoders continuously measure how much a shaft has rotated (angular position) and how fast (angular velocity). This is the raw data that control systems use.

**2. Enabling Closed-Loop Control**
Without encoders, motors run open-loop — you command a speed but never know if it was achieved. With encoders, the controller can compare the *commanded* position to the *actual* position and apply corrective torque — this is closed-loop (feedback) control.

**3. Odometry in Mobile Robots**
Wheel encoders count how many times each wheel has rotated → multiply by wheel circumference → total distance traveled. Combined with differential drive kinematics, the robot estimates its (x, y, θ) position over time (dead reckoning).

**4. Joint Angle Feedback in Manipulators**
Robot arms use encoders at every joint. FK uses these angles to compute the end-effector's current position. IK controllers use them to verify the arm has reached the commanded configuration.

---

#### How Optical Encoders Work

**Physical construction:**
- A rotor **disc** with alternating opaque and transparent sections, mounted on the rotating shaft
- An **LED** (infrared light source) on one side of the disc
- A **photodetector** (phototransistor) on the other side

**Operation:**
- As the disc rotates with the shaft, transparent slots allow light through → photodetector outputs HIGH
- Opaque sections block light → photodetector outputs LOW
- This creates a **square wave pulse train**
- Counting pulses → measures angular displacement
- Measuring pulse frequency → measures angular velocity

**Quadrature Encoding:**
Two detectors offset by 90° (quarter of one slot period) produce two channels A and B. These are 90° out of phase:
- If A leads B → shaft is rotating clockwise
- If B leads A → shaft is rotating counterclockwise
- Counting all four edges (rising + falling of both A and B) → 4× resolution improvement

**Incremental Encoders** count relative pulses from a starting reference — they don't know absolute position after power loss.

**Absolute Encoders** output a unique binary code for every angular position — position is known immediately at power-on, no homing needed.

---

#### Factors to Consider When Selecting an Encoder

| Factor | Description | Example |
|---|---|---|
| **Resolution** | Number of pulses per revolution (PPR/CPR). More counts = finer position resolution. | A 100-CPR encoder detects 3.6° per count; a 10,000-CPR encoder detects 0.036° per count. For a surgical robot joint, high resolution is critical. |
| **Accuracy** | How closely does the encoder output reflect the true angular position? | Manufacturing imperfections in the disc can cause systematic position errors. |
| **Maximum Speed** | The encoder must generate pulses faster than the counting hardware can process at the robot's maximum joint velocity. | At 3000 RPM with a 1000-CPR encoder: 50,000 pulses/second — the counter must handle this rate. |
| **Environmental Robustness** | The encoder must withstand the robot's operating conditions — dust, oil, moisture, vibration, temperature. | Optical encoders (using light) are sensitive to contamination; magnetic encoders are more robust in dirty industrial environments. |
| **Absolute vs. Incremental** | Does the robot need to know its exact position immediately on power-up (absolute) or can it home first (incremental)? | A medical robot on a patient should never need a homing procedure — absolute encoder required. A conveyor robot can home to a fixed position at startup — incremental acceptable. |
| **Physical Size and Weight** | Especially in mobile robots and drones, encoder mass and dimensions affect the robot's total weight and payload capacity. | A miniature encoder for a robotic finger weighing < 1 g is needed; a heavy encoder for a large industrial arm is acceptable. |
| **Cost** | High-resolution absolute encoders are expensive. Budget must be weighed against precision requirements. | A hobby robot uses a $5 incremental encoder; a precision surgical arm may use a $500+ absolute encoder. |
| **Interface/Protocol** | The encoder must communicate with the robot's controller — via analog voltage, digital quadrature (A/B), SSI, SPI, CANbus, etc. | The controller must support the chosen protocol. |

---

> **Deep Explanation:** Encoders are the "proprioception" of robots — the sense of where your own limbs are without looking. Just as humans cannot walk or pick up objects without proprioception (people with proprioceptive loss cannot perform coordinated movement even with perfect muscle strength), robots cannot perform coordinated motion without encoders. Every closed-loop control system — from a simple motor speed controller to a full 6-DOF robot arm — depends fundamentally on encoder feedback. The quality of the encoder directly determines the precision, smoothness, and safety of the robot's motion.

---

### Q1.d — Active vs. Passive Sensors (4 Marks)

**Answer:**

#### Passive Sensors

**Definition:** Passive sensors measure *energy that is already present in the environment*. They do not emit any energy themselves — they only receive.

**Working principle:** The sensor contains a detector that responds to naturally occurring energy (light, heat, sound, radiation) from the environment or the target.

| Example | What it measures | Passive energy source |
|---|---|---|
| **CCD/CMOS Camera** | Visible light | Ambient light (sun, artificial lighting) |
| **Microphone** | Sound waves | Ambient sound from environment |
| **Thermometer / Thermocouple** | Temperature | Thermal radiation from environment |
| **Infrared thermal camera** | Heat radiation | Body heat from objects/people |
| **Strain gauge** | Mechanical deformation | Force applied to the structure |

**Advantages:** Simple (no emitter needed); cannot interfere with other sensors; no energy emitted into environment.

**Disadvantages:** Performance depends on ambient conditions (camera fails in darkness); cannot control the sensing signal.

---

#### Active Sensors

**Definition:** Active sensors *emit energy into the environment* and measure the returning signal (reflection, transmission, or modification of the emitted signal).

**Working principle:** The sensor contains both an emitter (generates a known signal) and a receiver (detects the return). Distance, velocity, or material properties are inferred from the return signal.

| Example | Emitted energy | What it measures |
|---|---|---|
| **LiDAR** | Laser light pulses | Distance (time of flight of laser return) |
| **Ultrasonic sensor** | High-frequency sound waves | Distance (echo time of flight) |
| **Radar** | Radio frequency waves | Distance and velocity (Doppler effect) |
| **Structured light depth sensor** | Infrared pattern | Depth per pixel (pattern deformation) |
| **Optical encoder** | Infrared LED light | Angular position (light interrupted by disc) |

**Advantages:** Controlled sensing signal → better accuracy; works regardless of ambient conditions; can sense in darkness.

**Disadvantages:** Emitted energy may interfere with other active sensors on the same robot (e.g., two ultrasonic sensors on the same robot detecting each other's echoes); may disturb sensitive environments.

---

**Key Distinction Table:**

| Property | Passive Sensors | Active Sensors |
|---|---|---|
| Energy emission | None | Yes (controlled) |
| Ambient dependence | High | Low |
| Interference risk | None | Yes (with other active sensors) |
| Works in darkness | No (visible cameras) | Yes |
| Complexity | Lower | Higher (emitter + receiver) |

---

## QUESTION 2

---

### Q2.a — Locomotion vs. Manipulation; Key Issues in Locomotion (7 Marks)

**Answer:**

#### Locomotion vs. Manipulation

**Locomotion** is the mechanism by which a robot moves its *entire body* from one location to another in its environment. It addresses the robot's translational and rotational movement through space.
- *Examples:* A robot rolling across a warehouse floor (wheeled locomotion), a quadruped walking across rough terrain (legged locomotion), a drone flying through an airspace (aerial locomotion).

**Manipulation** is the ability of a robot to *interact with and change objects* in its environment using specialized effectors (grippers, arms, tools). The robot's body may remain stationary while the arm or gripper moves.
- *Examples:* A robotic arm picking and placing components on an assembly line, a surgical robot manipulating tissue during an operation, a gripper sorting packages by weight.

**Key Differences:**

| Aspect | Locomotion | Manipulation |
|---|---|---|
| **Body movement** | Entire robot moves through space | Robot body may stay fixed; arm/effector moves |
| **Purpose** | Transport the robot from A to B | Interact with, move, or transform objects |
| **Relevant DOF** | Typically 3 DOF (x, y, θ) for ground robots | Typically 6 DOF (full position + orientation) |
| **Primary sensors** | Range finders, GPS, IMU, wheel encoders | Force/torque sensors, vision, tactile sensors |
| **Performance metric** | Speed, range, energy, terrain adaptability | Precision, dexterity, force control |
| **Primary challenges** | Stability, traction, terrain, efficiency | Grasp planning, force control, singularities |

*Note:* **Mobile manipulation** (a robot arm mounted on a mobile base) combines both — the base navigates and the arm manipulates. Examples: Boston Dynamics Spot with an arm, hospital delivery robots with package loading systems.

---

#### Key Issues and Limitations of Locomotion

**1. Stability**

Stability is the robot's ability to maintain balance without falling over. It is determined by:
- Number and geometry of ground contact points
- Position of the center of gravity (CoG) relative to the support polygon
- Robot dynamics (static vs. dynamic stability)

**Static stability:** Robot is stable at every instant even if all motion stops. CoG must remain within the support polygon (convex hull of contact points). Minimum 3 contact points required (tripod). Six-legged robots can always maintain 3 contacts while walking — statically stable.

**Dynamic stability:** Robot is stable only while in motion. Bipedal robots (humans, humanoid robots) are dynamically stable — they constantly shift their CoG through small balance corrections. A stopped biped falls without active balance control.

**2. Contact Characteristics**

The interaction between the robot's feet/wheels and the ground directly affects locomotion performance.

**Friction:** Too little friction → wheels spin (ice, wet floors); too little for legs → slipping during stance phase. Too much friction → inefficient (energy wasted overcoming unnecessary resistance).

**Surface compliance:** Soft surfaces (sand, mud) cause wheels to sink and generate high rolling resistance. Hard surfaces (concrete, metal) are ideal for wheels; soft surfaces favor legs (point contacts that penetrate minimally).

**Contact patch geometry:** Wide, flat contacts distribute load → stable on soft ground. Narrow point contacts → better obstacle navigation but may sink into soft terrain.

**3. Type of Environment**

- **Flat, hard surfaces:** Wheeled robots are 1-2 orders of magnitude more energy-efficient than legged robots. Wheels have minimal rolling resistance on hard ground.
- **Rough, uneven terrain:** Legged robots excel — their discrete contact points allow them to step over gaps, climb stairs, and navigate debris. Wheels accumulate difficulty as ground irregularity increases.
- **Aquatic environments:** Propeller (boat), undulatory swimming (fish-inspired), or jet propulsion (squid-inspired) mechanisms.
- **Aerial environments:** Fixed-wing (efficient forward flight, no hover), rotary-wing/VTOL (hovering, low-speed maneuver), hybrid.

**4. Energy Efficiency**

Locomotion is the primary energy consumer on mobile robots. Battery capacity directly limits operational range.

**Efficiency ranking on flat hard ground:**
1. Railway wheels on steel tracks (lowest rolling resistance)
2. Rubber wheels on hard surfaces
3. Legged locomotion
4. Crawling/undulatory motion (least efficient)

**Trade-off:** The most energy-efficient mechanism (wheels) is least adaptable. The most adaptable (legs) is least efficient.

**5. Mechanical Complexity and Reliability**

Legged locomotion requires many joints, actuators, and sensors per leg. Each joint is a potential failure point. Wheeled robots have fewer moving parts → higher reliability, lower maintenance.

**6. Kinematic Constraints (Non-holonomic)**

Most wheeled robots are *non-holonomic* — they cannot move in all directions instantaneously. A car cannot slide sideways; it must turn to change lateral position. This limits maneuverability and complicates path planning.

---

> **Deep Explanation:** The locomotion-manipulation distinction maps onto the two fundamental robot behaviors: *be somewhere* (locomotion) and *do something* (manipulation). Early industrial robots were manipulators bolted to the floor — they could do things precisely but couldn't go anywhere. Early mobile robots could go anywhere but had no manipulators. Modern robotics increasingly demands both capabilities together — robots that can navigate to where they're needed and then precisely interact with objects there. The challenges of locomotion and manipulation also interact: a mobile manipulator must maintain balance while its arm exerts forces — the arm's movements shift the CoG, potentially destabilizing the mobile base.

---

### Q2.b — Static and Dynamic Stability; Support Polygon (7 Marks)

**Answer:**

#### Static Stability

**Definition:** A robot is *statically stable* if it remains balanced and does not fall over at *every instant* of its motion, including when all motion stops. The stability does not depend on the robot being in motion.

**Condition for static stability:**
The robot's **Center of Gravity (CoG)** must lie *within the support polygon* at all times.

**Support Polygon:**
The support polygon is the convex hull (the smallest convex shape) enclosing all ground contact points. For a robot with legs on the ground, it is the polygon formed by connecting the feet.

**Example 1 — Three-legged stool (3 contact points):**

```
        A
       / \
      /   \
     B-----C
     
  Support polygon = triangle ABC
  CoG (geometric center) lies inside triangle → statically stable
```

Push the stool slightly — when released, gravity pulls the CoG back within the triangle. The stool self-rights without active balance control. This is **unconditional static stability**.

**Example 2 — Four-legged robot standing:**

```
  FL-------FR
  |         |
  |   CoG   |     ← CoG inside the rectangle → statically stable
  |    ×    |
  RL-------RR
  
  Support polygon = rectangle FLFRRRRL
```

When one leg is lifted (say FR lifted for a step):

```
  FL    (FR lifted)
  |  \
  |   \         Support polygon shrinks to triangle FL-RL-RR
  RL---RR        CoG must shift toward the remaining three feet
                 to stay inside the new smaller triangle → careful weight shifting needed
```

For four-legged walking to be statically stable, the robot must shift its CoG over the remaining three-foot support triangle before lifting each foot. This makes statically stable four-legged walking slow.

**6-Legged Tripod Gait (Hexapod):**

```
  LF    RF
  |      |
  LM----RM     ← All 6 legs initially in contact
  |      |
  LR    RR

  Tripod A (moving): LF, RM, LR
  Tripod B (stance): RF, LM, RR → forms a stable triangle on the ground
```

The standing tripod (B) always forms a stable triangle. The robot is statically stable throughout the walk because 3 legs always support the body in a stable tripod.

---

#### Dynamic Stability

**Definition:** A robot is *dynamically stable* if it maintains balance only while in motion. The robot is inherently unstable when stationary (like a bicycle) and must actively maintain balance through continuous control.

**Condition:** The robot manages its *dynamic momentum* to keep balance — momentum provides the stabilizing effect that static CoG position does not.

**Example — Bipedal Robot (Two-legged walking):**

```
During single-support phase (one foot on ground):

   [Body]
     |
     |   ← CoG is NOT above the support polygon (one foot = a point or tiny area)
     |
  [Foot]   ← tiny support polygon

If the robot stopped here, it would fall sideways.
But momentum carries it forward, and the other foot swings forward to catch it.
```

Human walking is dynamically stable — we are constantly "falling forward" and catching ourselves with the next step. Stop walking mid-stride → fall.

**ZMP (Zero Moment Point):**
Used for bipedal robot balance. The ZMP is the point on the ground where the net ground reaction force has zero moment (no tendency to tip the robot). For stable bipedal walking, the ZMP must remain within the support polygon (footprint).

---

#### Static vs. Dynamic Stability Comparison

| Property | Static Stability | Dynamic Stability |
|---|---|---|
| **Stability when stopped** | Yes — robot is always stable | No — robot falls if stopped |
| **Condition** | CoG inside support polygon | ZMP inside support polygon; maintained by motion |
| **Minimum legs** | 4 (so one can always be lifted, 3 remain) | 2 (bipedal) |
| **Walking speed** | Slow (careful CoG shifts) | Fast (momentum assists) |
| **Control complexity** | Simpler | Complex (real-time balance control) |
| **Examples** | Hexapod in tripod gait, slowly walking quadruped | Human walking, MIT Cheetah, Boston Dynamics Atlas |

---

> **Deep Explanation:** Static stability is like balancing a book on a table — it doesn't need to be moving to stay up; its geometry ensures stability. Dynamic stability is like a spinning gyroscope or a moving bicycle — remove the motion and it falls. In practice, the line between static and dynamic stability is a spectrum. A hexapod walking with tripod gait is statically stable. A hexapod running (all legs off the ground momentarily) enters a dynamic stability regime. Most modern legged robots operate in the dynamic regime because it allows much faster and more natural movement — at the cost of requiring sophisticated real-time control systems.

---

### Q2.c — Degrees of Freedom (DOF) in Robotics (7 Marks)

**Answer:**

#### Definition of DOF

**Degrees of Freedom (DOF)**, also called *mobility M*, is the number of *independent parameters* needed to completely specify the position and orientation of a body or mechanism.

In robotics, DOF determines:
- How many directions the end-effector can move
- What tasks the robot can perform
- The complexity of both the robot's mechanics and control

**Formula:**
> **M = Σ fᵢ** (sum of mobility of each joint)

Where fᵢ is the DOF of each joint:
- Revolute joint (pin/hinge): fᵢ = 1
- Prismatic joint (sliding): fᵢ = 1
- Spherical joint (ball-and-socket): fᵢ = 3

---

#### 2-DOF Robots

**Description:** Can control two independent parameters — typically two position coordinates (x, y) in a plane, or two joint angles.

**Geometry:** Can reach any point in a 2D plane within the workspace, but cannot control orientation.

**Example:** A simple planar robot arm with two revolute joints (two rotations in the same plane). Used in:
- Simple pick-and-place on flat surfaces
- Drawing/painting machines
- X-Y positioning tables (e.g., CNC router with 2 linear axes)

**Limitation:** Cannot tilt or orient the end-effector — it can only be *at* a 2D location, not pointed in a specific direction.

**Application:** 2D path tracing, simple component placement where orientation doesn't matter.

---

#### 3-DOF Robots

**Description:** Adds a third independent parameter — can reach positions in 3D space, but still cannot fully control orientation.

**Example:** 
- **SCARA robot** (Selective Compliance Articulated Robot Arm) — 3 joints (two horizontal revolute + one vertical prismatic). Can position end-effector at any (x, y, z) but only in one fixed orientation relative to vertical.
- A spherical coordinate arm (two rotation + one translation = 3 DOF).

**Applications:**
- PCB assembly (place components at precise x,y,z locations on a flat board — fixed orientation is fine)
- Palletizing (stacking boxes — orientation fixed, position varies)
- Simple pick-and-place where the part's orientation always arrives the same way

**Limitation:** Cannot tilt end-effector — a screwdriver with 3-DOF can reach any screw position but cannot angle the screwdriver to align with a tilted screw head.

---

#### 5-DOF Robots

**Description:** 3 translational DOF (position) + 2 rotational DOF (partial orientation control). Can control pitch and yaw but not roll (or similar combination).

**Example:**
- A 5-axis CNC milling machine — can position the cutting tool at any (x, y, z) and tilt it in two angular directions, enabling cuts on surfaces that are not purely horizontal.
- Some welding robots — can position the torch and tilt it in two directions to weld non-flat surfaces.

**Geometry:** Missing the 6th DOF (one rotation is unconstrained) — for many tasks (welding, milling, some assembly) this missing DOF is around the tool's own axis, which may be irrelevant (a drill bit can freely rotate around its axis without affecting the hole).

**Applications:**
- 5-axis CNC machining (aerospace components, molds)
- Material handling where full 6-DOF is unnecessary
- Spray painting where tool rotation axis doesn't matter

---

#### 6-DOF Robots

**Description:** Full control of both position (3 DOF) and orientation (3 DOF) of the end-effector. Can place the end-effector at any reachable point in space AND orient it in any direction.

**Example:**
- **FANUC M-10iA**, **ABB IRB 2400**, **KUKA KR 10 R1100** — six-axis industrial robot arms
- Each has 6 revolute joints. The first 3 joints determine position; the last 3 (wrist joints) determine orientation.

**Why 6 DOF is the "magic number":**
A rigid body in 3D space has exactly 6 DOF (3 translation + 3 rotation). A 6-DOF robot can match any rigid body's pose within its workspace. Fewer DOF → cannot reach all orientations. More DOF → redundant (multiple joint configurations reach the same end-effector pose — useful for avoiding obstacles but computationally complex).

**Applications:**
- **Welding:** Precisely positions and orients the torch to follow complex 3D weld seams
- **Assembly:** Aligns parts for insertion, regardless of arbitrary 3D orientation
- **Painting/coating:** Controls spray direction over curved 3D surfaces
- **Medical/surgical:** Positions surgical instruments in any required orientation
- **Material handling:** Pick up parts in any orientation and place them in any other orientation

---

**DOF Summary Table:**

| DOF | Position Control | Orientation Control | Representative Robot | Key Application |
|---|---|---|---|---|
| 2 | 2D plane | None | 2-joint planar arm | 2D drawing, flat assembly |
| 3 | 3D space | None | SCARA, spherical arm | PCB assembly, palletizing |
| 5 | 3D space | Partial (2 axes) | 5-axis CNC machine | Machining complex surfaces |
| 6 | 3D space | Full (3 axes) | FANUC, ABB, KUKA 6-axis | Welding, full-freedom assembly |

---

> **Deep Explanation:** DOF is fundamentally about how many dimensions of the task space the robot can control. The task space has 6 DOF for fully general manipulation (3 position + 3 orientation). Any robot with fewer than 6 DOF cannot perform tasks that require arbitrary orientation — it is constrained to a subspace of all possible poses. Robots with more than 6 DOF are *kinematically redundant* — like having 7 fingers when 5 are needed — the extra DOF provides null-space flexibility (can reconfigure without moving the end-effector) that is exploited for obstacle avoidance, joint limit avoidance, and singularity avoidance. Understanding DOF directly tells you what tasks a robot can and cannot perform.

---

### Q2.d — Four Differences Between Legged and Wheeled Robots (4 Marks)

**Answer:**

| # | Aspect | Legged Robots | Wheeled Robots |
|---|---|---|---|
| **1. Terrain Adaptability** | Excel on rough, uneven terrain, stairs, and gaps — legs make discrete point contacts, so only foothold quality matters, not ground between feet | Best suited to flat, smooth, hard surfaces; performance degrades rapidly on rough, soft, or uneven ground |
| **2. Energy Efficiency** | Significantly less energy-efficient on flat ground — each step requires lifting and placing legs using multiple actuated joints | 1-2 orders of magnitude more energy-efficient on hard flat surfaces due to minimal rolling resistance; no energy wasted lifting limbs |
| **3. Mechanical Complexity and Reliability** | High complexity: many joints, actuators, and sensors per leg (typically 3 DOF × 4–6 legs = 12–18 motors); more potential failure points | Lower complexity: typically 2–4 motors plus simple steering mechanisms; fewer failure points, easier maintenance |
| **4. Stability** | Four-legged and above: statically stable when standing (CoG inside support polygon); two-legged: dynamically stable (must actively balance, falls if stopped) | Inherently statically stable with 3+ wheels — all wheels always on ground; no balance control needed; never falls under normal conditions |

---

## QUESTION 3

---

### Q3.a — Optical Encoders: Measurement, Working, and Sensor Reaction (7 Marks)

**Answer:**

#### What Optical Encoders Measure

Optical encoders measure:
1. **Angular position** — how much a shaft has rotated (in counts, which convert to degrees or radians)
2. **Angular velocity** — how fast the shaft is rotating (counts per second → RPM)
3. **Direction of rotation** — clockwise or counterclockwise (via quadrature encoding with two channels)

In mobile robots, wheel-mounted optical encoders additionally provide:
- **Linear distance traveled** (arc length = angle × wheel radius)
- **Robot heading** (differential encoder readings between left and right wheels)

---

#### How Optical Encoders Work

**Physical Components:**

```
    LED (Infrared light source)
         ↓ light
    ┌─────────────┐
    │ ░░░░░░░░░░░ │  ← Rotor disc (rotates with shaft)
    │ ░ Opaque  ░ │     Alternating transparent slots and opaque sections
    │ ░░░░░░░░░░░ │
    └─────────────┘
         ↓ (some light passes through slots)
    Photodetector (phototransistor / photodiode)
```

**Operation:**

1. As the shaft rotates, the disc spins with it
2. **Transparent slot** passes the beam → photodetector receives light → output goes **HIGH** (logic 1)
3. **Opaque section** blocks the beam → photodetector receives no light → output goes **LOW** (logic 0)
4. This generates a **square wave pulse train** — one complete HIGH-LOW cycle per slot pair
5. A counter circuit counts the pulses → total angle = (count / CPR) × 360°

**Resolution:** Determined by CPR (Counts Per Revolution) — the number of transparent/opaque slot pairs on the disc. Higher CPR = finer resolution:
- 100 CPR → 3.6° per count
- 2000 CPR → 0.18° per count

**Quadrature (A/B) Encoding:**

Two detectors are placed 90° apart in phase (offset by exactly ¼ of one slot period):

```
Channel A:  ─┐  ┌──┐  ┌──
             └──┘  └──┘
Channel B:    ─┐  ┌──┐  ┌──
              └──┘  └──┘
(B lags A by 90°) → clockwise rotation
```

- **A leads B** → clockwise rotation
- **B leads A** → counterclockwise rotation
- Counting all 4 edges (A rising, A falling, B rising, B falling) = **4× resolution** (quadrature decoding)

---

#### How the Sensor Reacts When the Disc Blocks the Light Beam

When the **opaque section** of the encoder disc passes between the LED and the photodetector:

1. The infrared light beam is **completely blocked** — no photons reach the detector
2. The photodetector (phototransistor) receives no photon flux → generates no photocurrent
3. The base of the transistor is not forward-biased → transistor is in **cutoff** (non-conducting state)
4. The output voltage rises to the supply voltage (pulled HIGH by a pull-up resistor) or drops to LOW depending on the circuit configuration
5. In standard active-HIGH circuits: blocked beam → output **LOW** (0V)

When the **transparent slot** passes:
1. Light passes freely → photodetector generates photocurrent
2. Transistor enters **saturation** (fully conducting)
3. Output goes **HIGH** (logic level 1)

**Transition time:** Modern optical encoders can switch between HIGH and LOW states in nanoseconds, enabling very high rotational speeds to be measured accurately.

---

> **Deep Explanation:** The optical encoder is essentially a very fast, very precise light switch that the rotating shaft controls. The fundamental principle — light modulation by a spinning disc — is simple, but the engineering challenges lie in: (1) disc manufacturing precision (slots must be exactly equal and regularly spaced), (2) optical alignment (LED and detector must be perfectly centered on the disc slots), (3) electronics speed (must resolve transitions at thousands of Hz at high RPM), and (4) noise immunity (optical encoders are sensitive to dust, oil, and ambient light). Understanding how the phototransistor physically switches from cutoff to saturation explains why the output is a clean digital square wave rather than a sine wave — the transition between fully blocked and fully transparent happens rapidly as the slot edge passes the beam.

---

### Q3.b — Corner Detection and Moravec's Algorithm; Harris Improvement (7 Marks)

**Answer:**

#### What is Corner Detection in Feature Extraction?

**Corner detection** is the process of identifying points in an image where the local intensity pattern changes significantly in *multiple directions simultaneously* — these points are called **corners** or **interest points**.

**Why corners are useful:**
- Corners are highly *distinctive* — they look different from all surrounding points
- Corners are *localizable* — their position can be precisely determined
- Corners appear in many natural and man-made environments
- Corners are used as landmarks for robot navigation (localization, SLAM), object recognition, image stitching, and 3D reconstruction

**Three types of image regions:**

| Region Type | Characteristic | Detectable? |
|---|---|---|
| **Flat region** | No intensity change in any direction | Not useful — every point looks the same |
| **Edge** | Intensity changes strongly in one direction only (perpendicular to edge) | Useful for boundary detection but poorly localized along the edge |
| **Corner** | Intensity changes strongly in *all* directions | Ideal — unique, well-localized, distinctive |

---

#### Moravec's Corner Detection Algorithm

**Proposed by Hans Moravec.** The core idea: a corner is a point where sliding a small observation window in *any direction* causes a large change in the image content.

**Algorithm — Step by Step:**

**Step 1 — Define the reference window:**
Choose a target pixel P. Extract a small rectangular neighborhood (e.g., 3×3 pixels) centered on P. Call this the **reference matrix**.

**Step 2 — Shift in 8 directions:**
Shift the reference window by 1 pixel in each of 8 directions:
- 4 cardinal: Up (N), Down (S), Left (W), Right (E)
- 4 diagonal: NW, NE, SW, SE

**Step 3 — Compute SSD for each shift:**
For each direction d, compute the **Sum of Squared Differences (SSD)**:

> **SSD_d = Σ (Reference_pixel_i − Shifted_pixel_i)²**

where the sum is over all pixels in the window.

A small SSD → the shifted window looks similar to the reference (similar image content in that direction).
A large SSD → the shifted window looks very different (large intensity change in that direction).

**Step 4 — Take the minimum SSD:**
> **C = min(SSD₁, SSD₂, ..., SSD₈)**

The *minimum* SSD represents the direction of *least change* — the direction where the image is most similar.

**Step 5 — Threshold:**
> If **C > threshold** → the pixel is a **CORNER** (even in its most similar direction, the image changes significantly → changes in all directions)
> If **C ≤ threshold** → the pixel is NOT a corner (at least one direction shows little change → flat region or edge)

**Diagram:**

```
        Reference window
        centered on P (×)
           ┌───────┐
     ↖  ↑  │ 6  6  6│  ↗
     ←     │15 [14] 0│     →
     ↙  ↓  │15  0   0│  ↘
           └───────┘

Shift 1 (Up):    SSD₁ = (6-15)² + (6-14)² + (6-0)² + ... = 395
Shift 2 (Down):  SSD₂ = 451
Shift 3 (Left):  SSD₃ = 422
Shift 4 (Right): SSD₄ = 576
Shift 5 (↖):    SSD₅ = 227  ← MINIMUM
Shift 6 (↗):    SSD₆ = 764
Shift 7 (↙):    SSD₇ = 493
Shift 8 (↘):    SSD₈ = 702

min SSD = 227
Threshold = 220

Since 227 > 220 → pixel at center is a CORNER ✓
```

---

#### Improvement Introduced by the Harris Corner Detector

**Limitations of Moravec's method:**

1. **Only 8 discrete directions:** Only shifts in 8 compass directions are tested. True corners between these discrete directions may be missed.

2. **Sensitive to noise:** SSD is computed on raw pixel values — small noise fluctuations cause large SSD changes, producing false detections.

3. **Non-smooth:** The binary decision (corner or not) is abrupt — no continuous confidence score.

**Harris's Key Improvement — Use of Image Gradients:**

Instead of finite discrete shifts, Harris uses *continuous partial derivatives* (image gradients Ix and Iy) — representing infinitesimal shifts in all directions simultaneously.

**Harris computes the Second Moment Matrix M:**

```
M = Σ w(x,y) × [Ix²    IxIy]
                [IxIy   Iy² ]
```

where Ix = ∂I/∂x, Iy = ∂I/∂y (image gradients), and w(x,y) is a Gaussian weighting window.

**Harris Corner Response Function:**

Instead of computing eigenvalues (expensive), Harris uses:

> **C = det(M) − k × trace(M)²**

where:
- det(M) = Ix²·Iy² − (IxIy)² = λ₁·λ₂
- trace(M) = Ix² + Iy² = λ₁ + λ₂
- k ≈ 0.04 to 0.15 (empirical constant)

**Interpreting C:**

| C value | λ₁, λ₂ values | Region type |
|---|---|---|
| Large positive | Both large | **Corner** |
| Large negative | One large, one small | **Edge** |
| Near zero | Both small | **Flat region** |

**Harris improvements over Moravec:**

| Property | Moravec | Harris |
|---|---|---|
| Directions tested | 8 discrete | All directions (continuous gradients) |
| Rotation invariance | No | Yes (eigenvalues of M are rotation-invariant) |
| Noise sensitivity | High | Lower (Gaussian windowing smooths noise) |
| Response function | Binary (corner/not) | Continuous score C |
| False detections from noise | Many | Few |

---

> **Deep Explanation:** The fundamental insight behind both Moravec and Harris is the same: a corner is a point of maximal local uniqueness — a point where you can't slide in any direction without seeing a significantly different patch. Moravec approximates this with 8 finite shifts; Harris formalizes it with an eigenvalue analysis of the gradient covariance structure. The eigenvalues of M capture exactly how much the patch changes in the directions of least and greatest change — the principal axes. When both eigenvalues are large, the patch changes a lot in every direction — it's a corner. This eigenvalue-based formulation is fundamentally more elegant, rotation-invariant, and noise-robust than Moravec's discrete shift approach.

---

### Q3.c — IMU: Definition, DOF, and Block Diagram (7 Marks)

**Answer:**

#### Definition of IMU

An **Inertial Measurement Unit (IMU)**, also called an **Inertial Navigation System (INS)**, is a self-contained sensor system that measures the *specific force* (linear acceleration including gravity) and *angular velocity* (rotation rate) acting on a body. By integrating these measurements over time, the IMU estimates the body's complete **6-DOF pose** — its 3D position and 3D orientation — without any external reference.

**Key point:** The IMU is a *proprioceptive* sensor — it measures internal/inertial quantities, requiring no external beacons, satellites, or landmarks. It works completely in isolation.

---

#### Number of DOF Estimated

An IMU estimates **6 DOF:**

| DOF | Component | Measured by |
|---|---|---|
| 1 | x-position (forward/backward) | Accelerometer → double integration |
| 2 | y-position (left/right) | Accelerometer → double integration |
| 3 | z-position (up/down) | Accelerometer → double integration |
| 4 | Roll (rotation about x-axis) | Gyroscope → integration |
| 5 | Pitch (rotation about y-axis) | Gyroscope → integration |
| 6 | Yaw (rotation about z-axis) | Gyroscope → integration |

**Hardware components to measure 6 DOF:**
- **3-axis accelerometer:** Measures linear acceleration along x, y, z axes simultaneously
- **3-axis gyroscope:** Measures angular velocity about x, y, z axes simultaneously
- **3-axis magnetometer** (optional, for yaw correction): Provides absolute heading reference (magnetic north)

A **6-DOF IMU** = 3-axis accelerometer + 3-axis gyroscope
A **9-DOF IMU** = 3-axis accelerometer + 3-axis gyroscope + 3-axis magnetometer

---

#### IMU Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         IMU SYSTEM                                      │
│                                                                         │
│  ┌───────────────┐    ω (Angular velocity)    ┌─────────────────────┐  │
│  │               │──────────────────────────→ │    Integration      │  │
│  │  3-axis       │    (rad/s about x, y, z)   │    ∫ ω dt           │→ Roll, Pitch, Yaw │
│  │  Gyroscope    │                            └─────────────────────┘  │
│  │               │                                        ↓             │
│  └───────────────┘                            Orientation (R, P, Y)    │
│                                                            ↓             │
│  ┌───────────────┐    a (Specific force)      ┌─────────────────────┐  │
│  │               │──────────────────────────→ │ Gravity subtraction │  │
│  │  3-axis       │    (m/s² along x, y, z)   │ using orientation   │  │
│  │Accelerometer  │                            └──────────┬──────────┘  │
│  │               │                                        ↓             │
│  └───────────────┘                            Net linear acceleration  │
│                                                            ↓             │
│  ┌───────────────┐    m (Magnetic field)      ┌─────────────────────┐  │
│  │               │──────────────────────────→ │  First integration  │  │
│  │  3-axis       │    (optional, for yaw      │     ∫ a dt          │→ Velocity (vx, vy, vz) │
│  │Magnetometer   │     drift correction)       └──────────┬──────────┘  │
│  │  (optional)   │                                        ↓             │
│  └───────────────┘                            ┌─────────────────────┐  │
│                                               │  Second integration │  │
│                                               │     ∫ v dt          │→ Position (x, y, z) │
│                                               └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘

OUTPUT: Complete 6-DOF pose = (x, y, z, Roll, Pitch, Yaw)
```

**Signal Processing Path:**

```
Gyroscope output (ω)
    → Integrate once → Orientation angles (roll φ, pitch θ, yaw ψ)
    → Use orientation to compute rotation matrix R

Accelerometer output (a_measured = a_true + g_body_frame)
    → Subtract gravity component (using orientation from gyro)
    → Net acceleration a_true in body frame
    → Rotate to world frame using R
    → Integrate once → Velocity (vx, vy, vz)
    → Integrate again → Position (x, y, z)
```

**Critical Challenge — Drift:**
Every integration step accumulates error. Accelerometer bias of 0.01 m/s² integrates to 0.6 m/min velocity error, and 18 m/min² position error. IMU drift requires correction by GPS (outdoors) or visual odometry (indoors).

---

> **Deep Explanation:** The IMU is the robotic equivalent of the human vestibular system — the inner ear's balance organs. The semicircular canals sense rotation (gyroscope) and the otoliths sense linear acceleration (accelerometer). Just as the human brain integrates these signals to maintain spatial awareness during walking, running, and rapid head movements, the IMU's signal processing chain integrates gyroscope and accelerometer data to estimate pose. The analogy also extends to drift — humans who close their eyes and spin around lose track of their orientation (gyroscope drift in the absence of visual correction). IMUs similarly drift and require periodic correction from absolute reference sensors.

---

### Q3.d — Dead Reckoning; Sensor Used (4 Marks)

**Answer:**

#### What is Dead Reckoning?

**Dead reckoning** is the process of estimating a robot's *current position* by starting from a known position and continuously integrating estimates of velocity, direction, and elapsed time — without using any external positional reference.

**Formula:**
```
New Position = Previous Position + (Speed × Elapsed Time × Direction)

xₜ = xₜ₋₁ + Δx = xₜ₋₁ + v·cos(θ)·Δt
yₜ = yₜ₋₁ + Δy = yₜ₋₁ + v·sin(θ)·Δt
θₜ = θₜ₋₁ + Δθ = θₜ₋₁ + ω·Δt
```

*Analogy:* A ship's navigator in the era before GPS: "We were at Port A, sailed east at 10 knots for 3 hours, then turned north at 8 knots for 2 hours → we are now approximately here." The navigator integrates speed and heading over time. Any error in the speed measurement, the heading, or the elapsed time accumulates → position estimate drifts from the true position.

**Key Property — Cumulative Error:**
Dead reckoning errors are *unbounded* — they grow without limit over time. Small, consistent errors (e.g., wheel diameter miscalibration) grow linearly with distance traveled. Angular errors cause lateral position error to grow quadratically. This is why dead reckoning alone is unsuitable for long-distance navigation — periodic correction using absolute position sensors is essential.

**Types of Dead Reckoning Error:**
1. **Range error** — total distance traveled is slightly wrong
2. **Turn error** — rotation angle is slightly wrong
3. **Drift error** — systematic heading offset from unequal wheels

#### Sensors Used for Dead Reckoning

**Primary sensor: Wheel Encoders (Optical or Magnetic)**
- Count wheel rotations → compute distance traveled per wheel
- For differential drive robot: left and right encoder counts → heading change (turn = difference between left and right travel) + distance traveled
- Most common, most widely used in indoor mobile robots

**Secondary sensors (often combined for better accuracy):**

| Sensor | Contribution |
|---|---|
| **Gyroscope** | Directly measures angular velocity → heading change. Corrects turn errors in wheel-based odometry. |
| **Accelerometer / IMU** | Measures linear acceleration → double-integrate to get position. Useful for aerial robots or robots without wheels. |
| **Compass (flux-gate)** | Provides absolute heading reference (magnetic north). Corrects accumulated heading drift. But sensitive to ferrous interference indoors. |

**In practice:** Most mobile robots combine wheel encoders (primary) with a gyroscope (heading correction) for improved dead reckoning accuracy. This combined approach is called **odometry + gyro dead reckoning**.

---

## QUESTION 4

---

### Q4.a — Two Types of Bug Algorithm with Diagram (7 Marks)

**Answer:**

The **Bug algorithms** are the simplest class of reactive obstacle avoidance methods. They require only:
- Knowledge of the *approximate direction* to the goal
- A contact or proximity sensor to detect obstacles
- No map of the environment

They guarantee reaching any reachable goal by following an intuitive "walk toward goal, go around obstacles" strategy.

---

#### Bug1 Algorithm

**Strategy:** When an obstacle is encountered, the robot completely circumnavigates it to find the closest point to the goal, then departs from that point.

**Detailed Steps:**

1. **Move directly toward the goal** along a straight line (called the M-line in Bug2, but Bug1 just uses a direction vector to goal)
2. **When an obstacle is hit** at a point called the **Hit Point (H):**
   - Record this position and start following the obstacle's boundary (wall-follow)
3. **Completely circle the obstacle** — traverse the entire perimeter
   - While circling, continuously record the *distance to the goal* at every point on the perimeter
4. **Find the Leave Point (L):** The point on the perimeter where the robot was *closest to the goal*
5. **Return to L** (the robot must navigate back to this optimal departure point)
6. **Depart L** and resume moving directly toward the goal
7. **Repeat** for each new obstacle encountered

**Diagram:**

```
  Start
    ●
    |
    |  (direct path to goal)
    |
    ↓
    H← ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
    |  ┌───────────────────┐|
    |  │   OBSTACLE        ││
    └→ │                   ││← Robot circles entire perimeter
       │          L●       ││  (L = closest point to goal)
       │                   ││
       └───────────────────┘|
    ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘
         ↓ (departs from L toward goal)
         ●
        Goal
```

**Properties:**
- **Complete:** Guaranteed to reach any reachable goal
- **Inefficient:** Always traverses the *full* obstacle perimeter even if a shorter departure point exists early
- **Path length upper bound:** Known (finite) — proportional to perimeter of all obstacles

---

#### Bug2 Algorithm

**Strategy:** Instead of full perimeter traversal, the robot departs the obstacle as soon as it can move toward the goal on the M-line (the line from start to goal).

**Definition — M-line:** The straight line connecting the *original start position* to the goal position.

**Detailed Steps:**

1. **Define the M-line** — the straight line from Start to Goal
2. **Move along the M-line toward the goal**
3. **When an obstacle is hit** at Hit Point H (on or near the M-line):
   - Start following the obstacle's boundary (clockwise or counterclockwise)
4. **While wall-following, check two conditions simultaneously:**
   a. Is the robot on the M-line?
   b. Is the distance from the robot's current position to the goal *less than* the distance from H to the goal?
5. **When both conditions are true simultaneously** → this is the **Leave Point L**
   - Stop wall-following
   - Turn and move along the M-line toward the goal
6. **Repeat** for each new obstacle

**Diagram:**

```
  Start●──────────────────────────────────────● Goal
       |  ← M-line →                         |
       |                                      |
       ↓H (hit point on M-line)              |
       ┌──────────────────┐                  |
       │    OBSTACLE      │                  |
       │                  │                  |
       │         L●       │←wall-follow stops at L (back on M-line,
       └──────────┼───────┘  closer to goal than H)
                  |
                  └──────────────────────────→● Goal
                     (resume along M-line)
```

**Comparison — Bug1 vs Bug2:**

| Property | Bug1 | Bug2 |
|---|---|---|
| Perimeter traversal | Full (always) | Partial (until M-line re-crossed) |
| Efficiency | Low | Generally better |
| Path complexity | Longer average | Shorter average |
| Worst case | Both can be similar in adversarial environments |
| Completeness | Yes | Yes |
| When Bug2 is worse | Spiral obstacles where M-line is re-crossed many times before reaching goal |

---

#### State-Machine Implementation of Bug Algorithms

Both algorithms can be implemented as a simple two-state machine:

```
STATE: GOAL-SEEK
  Action: Move directly toward goal
  Transition to WALL-FOLLOW if: obstacle detected
  
STATE: WALL-FOLLOW
  Action: Follow obstacle boundary
  Transition to GOAL-SEEK if:
    Bug1 — completed full circuit AND returned to Leave Point (closest to goal)
    Bug2 — back on M-line AND closer to goal than Hit Point
```

---

> **Deep Explanation:** The elegance of Bug algorithms lies in their minimal requirements. They prove that a robot with only a compass (direction to goal) and a touch sensor can navigate any simply-connected environment. The fundamental tradeoff between Bug1 and Bug2 is certainty vs. efficiency: Bug1 guarantees finding the absolutely optimal departure point by full traversal; Bug2 takes the first acceptable departure point it finds. Bug2 wins on average but can perform worse on obstacles specifically designed to exploit its departure criterion (e.g., a spiral obstacle where the M-line crosses the obstacle boundary many times before the robot can actually escape).

---

### Q4.b — Temporal Decomposition and Navigation Architecture (7 Marks)

**Answer:**

#### Definition of Temporal Decomposition

**Temporal decomposition** is an architectural principle for organizing a robot's software modules into layers based on their *timing requirements* — specifically, how rapidly each module must respond to sensor inputs and produce outputs.

In temporal decomposition, modules are arranged in a vertical stack:
- **Bottom layers:** Fast, real-time reactive modules — respond in milliseconds
- **Top layers:** Slow, deliberative strategic modules — may take seconds or longer

Each adjacent layer typically differs by *orders of magnitude* in response time.

---

#### Generic Navigation Architecture Diagram

```
┌─────────────────────────────────────────────────────┐
│          PATH PLANNER (Strategic Layer)             │  ← Slowest (seconds to minutes)
│  • Generates global optimal trajectory              │    Uses complete map
│  • Non-real-time computation                        │    Output: waypoint sequence
└────────────────────┬────────────────────────────────┘
                     ↕ (bidirectional — feedback + commands)
┌────────────────────┴────────────────────────────────┐
│            EXECUTIVE (Tactical Layer)               │  ← Medium speed (100ms–1s)
│  • Bridges planning and real-time execution         │    Activates behaviors
│  • Handles failures; manages short-term memory      │    Monitors progress
└────────────────────┬────────────────────────────────┘
                     ↕
┌────────────────────┴────────────────────────────────┐
│         REAL-TIME CONTROLLER (Reactive Layer)       │  ← Fast (10–50 ms cycles)
│  [Behavior 1:       ][Behavior 2:    ][Behavior 3:] │
│  [Obstacle Avoidance][Wall Following ][Goal Seeking] │
│  [         PID Motion Control (< 10 ms)            ] │
└────────────────────┬────────────────────────────────┘
                     ↕
┌────────────────────┴────────────────────────────────┐
│             ROBOT HARDWARE                          │  ← Physical layer
│  • Executes motor commands                          │    Wheel encoders
│  • Reads raw low-level sensors (bumpers, encoders)  │    IMU, motor drivers
└─────────────────────────────────────────────────────┘
```

---

#### The Four Interrelated Trends of Temporal Decomposition

As you move from the **bottom** (real-time hardware layer) to the **top** (strategic path planning layer), four properties consistently trend together:

---

**Trend 1 — Sensor Response Time (Increases upward)**

**Definition:** The time elapsed between a sensor event being acquired by hardware and the module producing a changed output.

- **Bottom layers (fast):** Sensor response time = milliseconds. A wheel encoder reading causes an immediate motor speed correction in the PID loop. Limited only by processor clock speed and sensor sampling rate.
- **Top layers (slow):** Sensor response time = seconds or longer. A new laser scan of the environment might take several seconds to update the global path plan, as the path planner recomputes the optimal route.

*Analogy:* A driver's foot reacts to a car in front braking in ~150ms (low-level reflex). But if they receive a traffic report about a closed highway (high-level information), they might take 30 seconds to decide to take a different route.

---

**Trend 2 — Temporal Depth (Increases upward)**

Temporal depth has two components:

**a) Temporal Horizon** — how far *ahead* the module plans:
- Bottom (collision avoidance): Plans the next 0.1 seconds — "don't hit what's directly in front of me right now"
- Top (path planner): Plans the next 30 minutes of navigation — "the optimal route from here to the goal across the entire building"

**b) Temporal Memory** — how far back the module uses historical data:
- Bottom: Uses only the most recent sensor reading; forgets history quickly
- Top: Uses a complete map built from all past sensor readings (temporal memory of the entire operation)

*Analogy:* A defensive driver uses 1-2 seconds of ahead-looking reaction (short horizon). A trip planner uses last week's traffic data and plans the route for a 4-hour journey (long horizon + long memory).

---

**Trend 3 — Spatial Locality (Increases upward)**

**Definition:** The physical geographic scope of the module's decisions.

- **Bottom:** Controls wheel torque → affects the robot's motion over the next few centimeters. Spatially very local.
- **Top:** Plans a route from one side of a building to the other → decisions affect the robot's behavior 100+ meters away. Spatially global.

*Analogy:* Your fingertips regulate exactly how hard to press a button (very local, centimeter-scale control). Your brain's navigation system decides which city to travel to next week (very global, kilometer-scale).

---

**Trend 4 — Context Specificity (Increases upward)**

**Definition:** How much the module's output depends on broader context beyond its immediate inputs.

- **Bottom layers:** Context-insensitive. The collision avoidance module applies the same rule in every situation: "obstacle detected within 0.5 m → slow down, turn." It doesn't care whether the robot is delivering medicine or escaping a fire.
- **Top layers:** Highly context-specific. The same sensor reading ("open corridor") might produce completely different responses depending on:
  - The robot's current mission (delivering medicine vs. exploring)
  - Time of day (busy hospital corridor vs. night when empty)
  - Environmental state (emergency vs. routine)
  - Past history (this corridor is usually blocked at this time)

*Analogy:* Your knee-jerk reflex (hammered tendon → leg kicks) is completely context-free — it happens regardless of whether you're in a doctor's office or a ballet performance. But your decision to speak out loud depends enormously on context: the same thought produces very different behavior in a library vs. at a party.

---

> **Deep Explanation:** The four trends are interrelated because they all emerge from the same underlying principle: higher layers process *more integrated, more abstract, more contextual* information over *longer time scales* and *larger spatial scales*. This hierarchy naturally separates fast reactive behaviors (which must be context-free to be fast) from slow deliberative planning (which must be context-aware to be strategically correct). The architecture works because the slow top layers set the objectives, the medium executive layer coordinates which reactive behaviors to activate, and the fast reactive layers guarantee real-time safety — each level operating at the right speed for its scope of decision-making.

---

### Q4.c — Five Key Points of Particle Filter SLAM (7 Marks)

**Answer:**

**Particle Filter SLAM** (also called **FastSLAM** in its efficient formulation) is a method for solving the SLAM problem by representing the robot's belief about both its own pose and the map as a set of *weighted sample hypotheses* (particles).

---

**1. Particle Representation — Each Particle is One Complete World Model**

Each particle pᵢ represents *one hypothesis* about the complete robot state:

```
Particle p^(i) = {
    x_r^(i):  one possible robot pose (x, y, θ)
    m^(i):    one possible map (all landmark positions associated with this pose)
}
```

The cloud of N particles collectively represents the probability distribution over all possible robot poses and maps. Dense clusters of particles at certain poses indicate high probability at those poses; sparse regions indicate low probability.

*Key advantage:* This sample-based representation can represent *any* distribution shape — including multi-modal distributions (robot might be in one of several possible locations) that Gaussian-only methods (Kalman Filter) cannot handle.

---

**2. Three-Step Iterative Process: Sample → Weight → Resample**

The particle filter operates in three repeating phases:

**Phase 1 — Sample (Propagation):**
Each particle's pose is propagated forward through the motion model with noise:
```
x_t^(i) ~ p(x_t | u_t, x_{t-1}^(i))
```
The robot commanded a 0.5 m forward motion → each particle moves forward by approximately 0.5 m, but with different noise realizations (some 0.48 m, some 0.52 m, some slightly angled). This spreads the particle cloud, reflecting growing pose uncertainty.

**Phase 2 — Weight (Measurement Likelihood):**
Each particle receives a weight proportional to how well its hypothesized pose explains the actual sensor reading:
```
w_t^(i) = p(z_t | x_t^(i))
```
A particle near the robot's true pose → its map predicts sensor readings that closely match actual readings → high weight.
A particle far from the true pose → map predictions don't match actual readings → low weight.

**Phase 3 — Resample (Importance Sampling):**
Draw N new particles from the weighted set with replacement, proportional to weights:
- High-weight particles (consistent with observations) are duplicated multiple times
- Low-weight particles (inconsistent) are discarded
- After resampling, all surviving particles have equal weight

Over many iterations, the cloud converges: particles cluster at the robot's true trajectory and the correct map.

---

**3. FastSLAM Factorization — Decoupling Robot Path from Landmarks**

The computational breakthrough of FastSLAM is recognizing a key mathematical factorization:

```
p(robot_path, map | observations) 
  = p(robot_path | observations) × ∏ p(landmark_k | robot_path, observations)
```

**Meaning:** Given the robot's full path, each landmark's position is **conditionally independent** of all other landmarks. This means:
- The particle filter handles the *robot path* distribution (difficult, non-Gaussian, high-dimensional)
- A *separate small EKF per landmark* handles each landmark's position (simple 2D Gaussian, trivially computed)

Each particle carries N independent 2D Kalman Filters — one for each landmark in the map. These per-landmark EKFs are tiny (2×2 covariance matrix) and update in constant time.

**Computational complexity:** O(M log N) per time step where M = particles, N = landmarks — compared to O(N²) for EKF-SLAM. Dramatically more scalable.

---

**4. Loop Closure Detection and Correction**

A critical capability of Particle Filter SLAM is handling **loop closure** — when the robot returns to a previously visited area.

When the robot re-observes a known landmark:
- Particles whose hypothesized robot path is *consistent* with the re-observed landmark receive high weights
- Particles whose hypothesized path has accumulated too much error (would place the robot far from the observed landmark) receive very low weights
- After resampling, only the consistent particles survive → path uncertainty drops dramatically

The loop closure effect propagates: surviving particles not only have a corrected current pose, but also carry corrected maps (because each particle's map was consistent with its trajectory).

*Challenge:* **Data association** — determining whether a current observation is a known landmark or a new one. Wrong associations (false loop closures) cause catastrophic map corruption. Solutions: conservative matching thresholds, geometric consistency checks, appearance descriptors.

---

**5. Particle Deprivation and Mitigation**

**Particle deprivation** (also called *sample depletion* or *particle collapse*) is a critical failure mode:

**Problem:** After many resampling steps, all N particles may converge to a small region of the state space. If the robot's true state is *outside* this region (e.g., after kidnapping, or a poor initial hypothesis), no particle represents the true state. The filter cannot recover — all particles agree on the wrong pose.

*Example:* N=500 particles all cluster at pose (1.0, 2.0, 45°) after 100 iterations. The robot is suddenly kidnapped to pose (8.0, 3.0, 90°). No particle is anywhere near (8.0, 3.0, 90°). All particles continue to predict observations from near (1.0, 2.0, 45°) — all are wrong. All receive near-zero weights → resampled copies of the same wrong particles → the filter is stuck.

**Mitigation strategies:**
- **Increase N:** More particles → broader coverage of state space → less likely for the true state to fall outside coverage
- **Random particle injection:** Periodically add M random particles uniformly distributed over the state space ("sensor resetting"). If the filter is healthy, random particles get zero weight and die immediately. If the filter has collapsed wrong, a random particle near the true state gets high weight and propagates.
- **Adaptive N (KLD-Sampling):** Dynamically set the number of particles based on the current uncertainty — many particles when uncertain, fewer when confident
- **Multiple restart:** If filter consistency metrics (likelihood of most recent observation under current belief) drop below a threshold, re-initialize with a broader distribution

---

> **Deep Explanation:** Particle Filter SLAM sits at the intersection of three ideas: Monte Carlo methods (represent distributions with samples), importance sampling (reweight based on evidence), and Rao-Blackwellization (the FastSLAM factorization that decomposes the joint distribution). Its power comes from the first idea: samples can represent *anything*. Unlike EKF-SLAM which assumes the world is Gaussian (bell-shaped uncertainty), particles can represent a robot that is simultaneously probably in the kitchen (70%) and possibly in the hallway (30%). This makes it robust to global localization, dynamic environments, and the kidnapping problem. The price is computational: maintaining N complete map hypotheses costs N × (map_size) memory. FastSLAM's log N update complexity makes this tractable for moderate-sized maps.

---

### Q4.d — Advantages of Reinforcement Learning in Legged Robots and Autonomous Vehicles (4 Marks)

**Answer:**

#### Advantages in Legged Robots

**1. Autonomous Gait Discovery and Optimization**
Manually designing gaits for legged robots is extremely complex — optimizing step timing, joint torques, foot placement, and balance across dozens of parameters is beyond what human designers can do analytically. RL allows robots to discover effective gaits through trial-and-error, finding solutions that human designers might never conceive.

*Example:* MIT's Mini Cheetah robot, trained with RL, learned to run at high speeds and recover from pushes — generating movement patterns that were more efficient and robust than hand-designed controllers.

**2. Terrain Adaptability**
Pre-programmed locomotion controllers fail on unexpected surfaces (wet, loose, slopes, stairs, debris). RL-trained policies learn to sense ground conditions through proprioceptive feedback (joint forces, IMU readings) and *adapt* their gait in real-time.

*Example:* A quadruped trained with RL across diverse terrain types (gravel, grass, mud, inclines) in simulation learns a generalizable control policy that adapts on the fly when deployed on a real, unseen terrain.

**3. Fault Tolerance**
RL-trained robots can learn to continue functioning even when a joint fails or a leg is damaged — the policy discovers compensating behaviors (adjusting the other legs' patterns) through experience.

**4. Natural, Efficient Movement**
RL with energy efficiency as part of the reward function produces energetically optimal gaits that match or exceed the efficiency of hand-designed controllers, because the optimizer finds solutions in the continuous joint torque space that are impossible to derive analytically.

---

#### Advantages in Autonomous Vehicles

**1. Handling Complex, Long-Tail Scenarios**
Rule-based autonomous driving systems fail on rare or unexpected situations (a sofa falling off a truck, a construction worker directing traffic with unusual gestures, aggressive merging behavior). RL can train on billions of simulated scenarios — including rare ones — to develop robust policies.

**2. Smooth, Human-Like Driving Behavior**
Classical rule-based drivers can be jerky and overly conservative. RL agents trained with rewards for comfort (minimizing acceleration), efficiency (minimizing travel time), and safety (maintaining safe distances) naturally develop smooth, human-like driving behavior.

**3. Adaptive Traffic Navigation**
RL agents learn to navigate complex intersections, roundabouts, dense urban traffic, and highway merging through experience — adapting to the specific behaviors of other drivers they encounter.

*Example:* Waymo uses RL-trained agents as the "social simulation" for other cars in their simulation environment — creating realistic, adversarial traffic scenarios that test their main autonomous driving policy.

**4. Long-Term Planning Under Uncertainty**
Autonomous vehicles must make decisions with long time horizons (should I take this lane now because I know I need to turn right in 500m?). RL's discounted return formulation naturally optimizes for long-horizon consequences, whereas rule-based systems are typically myopic.

**5. Continuous Improvement from Fleet Data**
RL policies can be continuously updated using real-world data from entire fleets of deployed vehicles. Each vehicle's experiences are aggregated, improving the policy for all vehicles — a capability unique to learned systems.

---

**Summary Table:**

| Domain | RL Advantage | Example |
|---|---|---|
| Legged robots | Gait discovery | Mini Cheetah learned to run |
| Legged robots | Terrain adaptability | Quadruped adapts to mud/gravel/grass |
| Legged robots | Fault tolerance | Compensates for damaged leg |
| Autonomous vehicles | Rare scenario handling | Falling cargo, unusual traffic gestures |
| Autonomous vehicles | Smooth driving | Comfort-optimal acceleration profiles |
| Autonomous vehicles | Long-horizon planning | Lane change 500m in advance |
| Autonomous vehicles | Fleet-wide learning | All vehicles improve from shared data |

---

> **Deep Explanation:** The unifying advantage of RL across both domains is its ability to *discover* good solutions in high-dimensional spaces that are intractable to design analytically. Both legged locomotion and autonomous driving involve complex interactions between dozens of continuous variables, with performance criteria that are easy to evaluate (did the robot fall? did the car reach its destination safely?) but hard to translate into explicit control rules. RL transforms these evaluation criteria directly into a learning signal — the reward function — and then optimizes behavior against that signal through experience. The result is systems that can generalize far beyond what their designers explicitly programmed, which is the essential requirement for robots operating in the open, unpredictable world.

---

*End of Solved Paper*

---

## Quick Marks Distribution

| Question | Topic | Marks |
|---|---|---|
| 1a | Technical, Social, Ethical challenges | 7 |
| 1b | Forward and Inverse Kinematics | 7 |
| 1c | Encoders — role, working, selection factors | 7 |
| 1d | Active vs. Passive sensors | 4 |
| 2a | Locomotion vs. Manipulation; key issues | 7 |
| 2b | Static vs. Dynamic stability; Support polygon | 7 |
| 2c | DOF — 2, 3, 5, 6 DOF robots | 7 |
| 2d | Legged vs. Wheeled differences | 4 |
| 3a | Optical encoders — measurement, working, reaction | 7 |
| 3b | Corner detection; Moravec; Harris | 7 |
| 3c | IMU — definition, DOF, block diagram | 7 |
| 3d | Dead reckoning; sensors | 4 |
| 4a | Bug1 and Bug2 algorithms with diagrams | 7 |
| 4b | Temporal decomposition; architecture | 7 |
| 4c | Particle Filter SLAM — 5 key points | 7 |
| 4d | RL advantages in legged robots and AVs | 4 |
| **Total** | | **100** |
