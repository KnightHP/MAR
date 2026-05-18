# Mobile and Autonomous Robotics — Unit 2: Perception
### Complete Notes | Course Code: UE23CS343BB7 | PES University

---

> **How to use these notes:** Every concept is explained from first principles with definitions, analogies, and mathematical foundations. These notes are slide-independent and self-contained.

---

## Table of Contents

1. [Introduction to Perception](#1-introduction-to-perception)
2. [Sensor Classification Recap](#2-sensor-classification-recap)
3. [Sensor Characteristics and Metrics](#3-sensor-characteristics-and-metrics)
4. [Non-Perception Sensors](#4-non-perception-sensors)
   - 4.1 [Optical Encoders](#41-optical-encoders)
   - 4.2 [Heading Sensors and Dead Reckoning](#42-heading-sensors-and-dead-reckoning)
   - 4.3 [Gyroscope — Mechanical and Optical](#43-gyroscope--mechanical-and-optical)
   - 4.4 [Accelerometer](#44-accelerometer)
   - 4.5 [Inertial Measurement Unit (IMU)](#45-inertial-measurement-unit-imu)
   - 4.6 [GPS — Global Positioning System](#46-gps--global-positioning-system)
5. [Perception Sensors — Range Finders](#5-perception-sensors--range-finders)
   - 5.1 [Time-of-Flight: Ultrasonic Sensors](#51-time-of-flight-ultrasonic-sensors)
   - 5.2 [Time-of-Flight: Laser Rangefinders and LiDAR](#52-time-of-flight-laser-rangefinders-and-lidar)
   - 5.3 [Time-of-Flight Camera](#53-time-of-flight-camera)
   - 5.4 [Triangulation Active Ranging](#54-triangulation-active-ranging)
   - 5.5 [RADAR](#55-radar)
6. [Fundamentals of Computer Vision](#6-fundamentals-of-computer-vision)
   - 6.1 [The Digital Camera](#61-the-digital-camera)
   - 6.2 [CCD vs CMOS Sensors](#62-ccd-vs-cmos-sensors)
   - 6.3 [Color Cameras — Bayer Filter and Demosaicing](#63-color-cameras--bayer-filter-and-demosaicing)
   - 6.4 [Image Formation Optics](#64-image-formation-optics)
7. [Robot Vision in Different Platforms](#7-robot-vision-in-different-platforms)
   - 7.1 [UGVs — Unmanned Ground Vehicles](#71-ugvs--unmanned-ground-vehicles)
   - 7.2 [UAVs — Unmanned Aerial Vehicles](#72-uavs--unmanned-aerial-vehicles)
   - 7.3 [AUVs — Autonomous Underwater Vehicles](#73-auvs--autonomous-underwater-vehicles)
8. [Image Processing Fundamentals](#8-image-processing-fundamentals)
9. [Feature Extraction](#9-feature-extraction)
   - 9.1 [What Are Features?](#91-what-are-features)
   - 9.2 [Feature Definition and Hierarchy](#92-feature-definition-and-hierarchy)
   - 9.3 [Factors Influencing Feature Choice](#93-factors-influencing-feature-choice)
10. [Interest Point Detectors](#10-interest-point-detectors)
    - 10.1 [Properties of an Ideal Feature Detector](#101-properties-of-an-ideal-feature-detector)
    - 10.2 [Moravec Corner Detection (SSD Method)](#102-moravec-corner-detection-ssd-method)
    - 10.3 [Harris Corner Detection](#103-harris-corner-detection)
    - 10.4 [Invariance Properties of Harris](#104-invariance-properties-of-harris)
    - 10.5 [Scale-Invariant Detection](#105-scale-invariant-detection)
    - 10.6 [Affine-Invariant Detection (Harris-Affine)](#106-affine-invariant-detection-harris-affine)
    - 10.7 [Other Corner Detectors](#107-other-corner-detectors)
11. [Feature Extraction from Range Data](#11-feature-extraction-from-range-data)
12. [Stereo Vision and 3D Perception](#12-stereo-vision-and-3d-perception)

---

## 1. Introduction to Perception

### What is Perception?

**Definition:** Perception is the ability of a robot to *sense, interpret, and understand* its environment. It converts raw sensor data into *meaningful information* that the robot can act upon.

Perception is what enables a robot to answer the fundamental question: **"What is happening around me?"**

Without perception, a robot is essentially blind — it can move its actuators, but it has no understanding of the world it's moving through.

*Analogy:* Perception in robots is like your entire sensory system — not just the raw signal from your eyes and ears, but the *interpretation* of those signals. Hearing a loud bang is a raw signal; recognizing it as a car backfiring (and not a gunshot) is perception.

### The Perception Pipeline

Perception is not just one step — it is a sequence of processes:

```
Sensory Inputs → Data Processing → Environment Understanding
→ Situation Assessment → Decision Making
```

**Sensory Inputs:** Raw data from cameras, LiDAR, IMU, GPS, ultrasonic sensors, etc.

**Data Processing:** Filtering noise, calibrating, synchronizing data streams.

**Environment Understanding:** Identifying objects, obstacles, landmarks, terrain — giving structure to raw data.

**Situation Assessment:** Understanding *what the current state means* — "there is a pedestrian 2 meters ahead moving left."

**Decision Making:** Using the understood situation to choose the next action.

### Why Perception is Central to Autonomy

One of the most important tasks of any autonomous system is to **acquire knowledge about its environment**. This is done by:
- Taking measurements using various sensors
- Extracting meaningful information from those measurements (perception)

The robot's **sensory system** handles data acquisition — both internal state (joint angles, battery level) and external environment (obstacles, distances, visual features).

The robot's **actuation system** uses this knowledge to take physical actions (locomotion, manipulation).

### Perception vs. Non-Perception Sensors

A key distinction in this unit:

| Category | Sensors | Role |
|---|---|---|
| **Non-Perception Sensors** | Encoders, Gyroscope, Accelerometer, IMU, GPS | Measure the robot's *own state* — internal navigation and position tracking |
| **Perception Sensors** | LiDAR, Radar, Cameras, Ultrasonic, Infrared | Measure the *external environment* — detect and understand the world |

Non-perception sensors support *dead reckoning and state estimation*. Perception sensors support *environment mapping, obstacle detection, and object recognition*.

---

## 2. Sensor Classification Recap

### Two Classification Axes

**Axis 1: Proprioceptive vs. Exteroceptive**

**Proprioceptive sensors** — measure values *internal* to the robot.
- Motor speed, wheel load, joint angles, battery voltage, internal temperature
- *Analogy:* Your sense of where your own limbs are without looking — proprioception in humans. You know your arm is raised even with your eyes closed.

**Exteroceptive sensors** — acquire information from the *environment*.
- Distance to obstacles, light intensity, sound amplitude
- Includes all forms of external sensation: vision, hearing, touch, smell
- *Analogy:* All five of your outward senses — they sense the world outside your skin.

**Axis 2: Passive vs. Active**

**Passive sensors** — measure *ambient energy entering* the sensor; emit nothing.
- Examples: temperature probes (measure existing heat), microphones (capture existing sound), CCD/CMOS cameras (capture existing light)
- *Analogy:* Sitting quietly in a room, absorbing sounds without making any yourself.

**Active sensors** — *emit energy* into the environment, then measure the reflected/returned energy.
- Examples: wheel quadrature encoders (emit light, detect interruption), ultrasonic sensors (emit sound, measure echo), laser rangefinders (emit laser, measure return)
- Advantage: More controlled interaction → often superior performance
- Risk: Emitted energy may affect what you're measuring; interference between multiple active sensors on the same robot; interference from other robots nearby
- *Analogy:* Using a flashlight in a dark room — you emit light to see, but you might also disturb what you're looking at (startling an animal, for example).

---

## 3. Sensor Characteristics and Metrics

This section defines the technical properties by which any sensor is evaluated. Understanding these is critical for choosing the right sensor for a task and understanding its limitations.

### 3.1 Cost

Every sensor requires a trade-off between cost and performance. A highly accurate, low-noise sensor may be unaffordable for a small robot. Cost must be balanced against:
- Reliability (how often it fails)
- Accuracy (how close to the true value)
- Repeatability (same result for same input)
- Operational life

### 3.2 Weight

In mobile and aerial robots, *weight is a critical resource*. A heavy sensor adds to the robot's total inertia, reduces payload capacity, increases energy consumption, and may compromise maneuverability.

*Example:* A heavy camera mounted on a robotic insect would limit its flying capabilities — the insect's wings weren't designed to carry that load.

### 3.3 Output Type: Analog vs. Digital

- **Analog output** — a continuous voltage or current proportional to the measured quantity. Requires an ADC (Analog-to-Digital Converter) to interface with digital systems.
- **Digital output** — a discrete value (e.g., binary, pulse count). No ADC needed, but resolution is limited to the bit depth.

### 3.4 Resolution

**Definition:** Resolution is the *minimum difference between two values that the sensor can distinguish*. It is the smallest detectable change in input. The lower limit of the dynamic range is equal to the resolution.

*Analogy:* A ruler with millimeter markings has 1 mm resolution — it cannot distinguish objects less than 1 mm apart.

**Example:** A voltage sensor performs 8-bit A/D conversion over the range 0–5V.
- Number of discrete levels = 2⁸ - 1 = 255
- Resolution = 5V / 255 ≈ **20 mV**
- This sensor cannot distinguish two voltages that differ by less than 20 mV.

### 3.5 Dynamic Range

**Definition:** Dynamic range is the ratio of the *maximum* measurable input to the *minimum* measurable input. It represents how wide a spread of values the sensor can handle while still functioning correctly.

> **Dynamic Range (dB) = 10 × log₁₀(max/min)** — for power quantities (current, power)
> **Dynamic Range (dB) = 20 × log₁₀(max/min)** — for amplitude quantities (voltage, distance), because power ∝ amplitude²

**Example 1 — Current sensor (1 mA to 20 A):**
> DR = 10 × log₁₀(20,000 mA / 1 mA) = 10 × log₁₀(20,000) = 10 × 4.3 = **43 dB**

**Example 2 — Voltage sensor (1 mV to 20 V):**
> DR = 20 × log₁₀(20,000 mV / 1 mV) = 20 × log₁₀(20,000) = 20 × 4.3 = **86 dB**

A sensor with high dynamic range can handle both very weak and very strong signals without saturation or being unable to detect the weak ones.

*Analogy:* The human ear has a massive dynamic range (~120 dB) — it can hear a quiet whisper (threshold) and a jet engine (pain threshold) without losing function. A cheap microphone might clip (saturate) on loud sounds and miss quiet ones.

### 3.6 Linearity

**Definition:** A sensor has a *linear response* if a proportional change in input produces a proportional change in output. Mathematically:

> If inputs x and y produce outputs f(x) and f(y), then:
> **f(ax + by) = af(x) + bf(y)**

Linear sensors are much easier to model and calibrate. Non-linear sensors require complex calibration curves.

*Analogy:* A spring scale is linear — double the weight, double the deflection. Many sensors start linear but become non-linear at their extremes (saturation).

### 3.7 Bandwidth (Frequency)

**Definition:** Bandwidth is the *rate* at which a sensor can provide measurements — measured in Hz (measurements per second).

A sensor with 100 Hz bandwidth provides 100 readings per second.

**Why bandwidth matters in robotics:**
- A fast-moving robot needs high-bandwidth obstacle detection sensors to react in time
- If the robot is moving faster than the sensor can sample, it may collide before detecting an obstacle
- Increasing bandwidth of ranging and vision sensors has been a high-priority research goal in mobile robotics

*Analogy:* A security guard who checks a door only once per hour (1/3600 Hz) vs. one who checks every second (1 Hz) — the latter catches intruders much sooner.

### 3.8 Sensitivity

**Definition:** Sensitivity is the *ratio of output change to input change* — how strongly the sensor's output responds to a change in what it's measuring.

> **Sensitivity = Δoutput / Δinput**

High sensitivity: small input changes → large output changes (good for detecting subtle effects)
Low sensitivity: need large input changes to see any output change

**Cross-sensitivity:** Sensitivity to environmental parameters that are *orthogonal* (unrelated) to the target parameter — this is the sensor being confused by something it's not supposed to measure.

**Important Example — Flux-gate Compass:**
- A flux-gate compass has high sensitivity to magnetic north → very useful for robot navigation
- However, it also has high cross-sensitivity to ferrous (iron-containing) building materials
- In some indoor environments, metal beams, reinforcement bars, and elevator shafts create local magnetic distortions
- The cross-sensitivity to these ferrous materials can make the compass reading completely unreliable indoors
- This is why robots often cannot use a compass for indoor navigation

*Analogy:* A very sensitive smoke detector (high sensitivity to smoke) that also triggers on burnt toast (cross-sensitivity to cooking vapors) — the sensitivity is useful in real fires but causes false alarms.

### 3.9 Error

**Definition:** Error = Measured value (m) − True value (v)

> **Error = m − v**

Error tells you how far off the sensor's reading is from reality.

**Types of Error:**

**Systematic Errors (Deterministic)**
- Caused by identifiable, modelable factors — they are *predictable* and *repeatable*
- Can (in theory) be corrected through calibration
- Examples:
  - *Poor calibration* of a laser rangefinder → always reads 5 cm too far
  - *Bent stereo camera head* from a collision → systematic offset in stereo depth estimates
  - *Encoder miscounting* due to physical damage to the disk

*Analogy:* A weighing scale that always reads 2 kg too heavy — systematic, but fixable (subtract 2 kg from every reading).

**Random Errors (Stochastic)**
- Cannot be predicted or modeled — described only *probabilistically*
- Cannot be eliminated by better hardware alone
- Can only be managed statistically (averaging, filtering)
- Examples:
  - *Hue instability* in a color camera → color values fluctuate randomly frame to frame
  - *Spurious range-finding errors* → random distant readings with no physical cause
  - *Black level noise* in a camera → random pixel brightness in dark areas (electronic shot noise)

*Analogy:* Rolling a die — you can't predict the next roll, only its probability distribution.

### 3.10 Precision vs. Accuracy

These two terms are commonly confused. They are *distinct and independent* properties:

**Precision** — reproducibility; how *consistently* does the sensor produce the same output for the same input?

**Accuracy** — correctness; how *close* is the sensor's output to the true value?

| | High Precision | Low Precision |
|---|---|---|
| **High Accuracy** | ✅ Ideal: consistent AND correct | Correct on average but scattered |
| **Low Accuracy** | Consistent but consistently wrong | Scattered AND wrong |

*Analogy:* Think of a dartboard. If you always hit the same spot (tight cluster), that's high precision. If that spot is the bullseye, that's also high accuracy. A sensor can be precise (repeatable) but inaccurate (consistently biased).

**Mathematical characterization:** If sensor random error has mean μ and standard deviation σ:
- High precision → small σ (tight clustering)
- High accuracy → μ ≈ 0 (mean error close to zero, no bias)

---

## 4. Non-Perception Sensors

Non-perception sensors measure the robot's *own internal state* — they support dead reckoning, state estimation, and self-awareness. They answer "where am I and how am I moving?" rather than "what is in my environment?"

---

### 4.1 Optical Encoders

**What they are:** Optical encoders are proprioceptive sensors that measure *angular speed and position* of a rotating shaft. They use light to count discrete rotational increments.

**Odometry:** Encoders enable *odometry* — the process of estimating a robot's position by integrating wheel motion over time. The estimate is done in the *robot's own reference frame*, starting from a known position.

*Analogy:* Like counting your steps while walking. Each step is an increment; multiplying by stride length gives approximate distance traveled from your starting point.

**Physical Construction:**
1. An **illumination source** (LED) on one side of a disk
2. A **rotor disk** with a fine optical grid — alternating opaque and transparent sections — that rotates with the shaft
3. A **fixed grating** that masks the light
4. **Fixed optical detectors** (phototransistors) on the other side

**How it works:**
- As the disk rotates, transparent sections let light through → detector output HIGH
- Opaque sections block light → detector output LOW
- This creates a square wave pulse train
- Each pulse = one increment of rotation
- Counting pulses → measures *angular displacement*
- Measuring pulse rate → measures *angular velocity*

**Resolution:** Depends on how many sections (slots) are on the disk. More slots = finer resolution. A 1000 CPR (counts per revolution) encoder has 1000 pulses per full rotation → 0.36° resolution.

**Quadrature Encoding:** Most practical encoders use *two light beams offset by 90°*, producing two square waves (Channel A and Channel B) that are 90° out of phase. This allows:
- Determining *direction* of rotation (if A leads B → clockwise; B leads A → counterclockwise)
- Counting at *4× the base resolution* by counting all four edges (both rising and falling) of both channels

*Analogy:* Like a turnstile at a subway — clicking forward means you entered, clicking backward means you exited. The quadrature encoder knows whether the shaft is going forward or backward.

---

### 4.2 Heading Sensors and Dead Reckoning

**Heading sensors** determine a robot's *orientation* and *inclination* — the direction it is facing. Types:
- **Proprioceptive:** Gyroscope, inclinometer, accelerometer
- **Exteroceptive:** Compass (measures external magnetic field)

**Dead Reckoning:**

**Definition:** Dead reckoning is the process of calculating the *current position* of a moving object by using a *previously known position* (a "fix") and then continuously integrating estimates of:
- Speed (how fast)
- Heading / Direction (which way)
- Elapsed time (for how long)

> New Position = Old Position + (Speed × Time) in the Direction of Heading

*Analogy:* Navigating a ship across the ocean without GPS. You know your starting port, your speed (from the engine), your compass heading, and elapsed time — you calculate your current position from all of these. If any measurement has error, that error accumulates over time. Early sailors used dead reckoning, and small errors over long voyages led to significant positional errors.

**Limitation:** Dead reckoning *accumulates error over time*. Every small wheel slip, encoder miscalibration, or heading error adds up. The robot's estimated position drifts increasingly far from its true position. This is why dead reckoning must be corrected periodically by absolute position sensors (GPS, landmarks).

---

### 4.3 Gyroscope — Mechanical and Optical

**Definition:** A gyroscope is a heading sensor that *measures, maintains, and preserves the angular velocity and orientation* of an object relative to a fixed (inertial) reference frame.

Unlike a compass (which points north magnetically), a gyroscope provides *absolute heading relative to space* — it resists changes in its orientation due to angular momentum.

*Analogy:* A spinning top resists being tipped over — the faster it spins, the more stable its axis. That's the gyroscopic effect. A gyroscope in a robot uses this resistance to measure rotation.

#### Mechanical Gyroscope

**Principle:** Based on *gyroscopic precession* — the angular momentum of a fast-spinning rotor keeps its axis of rotation inertially stable.

The key property:
> **Reactive torque τ ∝ ω × Ω × I**

Where:
- ω = spinning speed of the rotor
- Ω = precession speed (the rate at which the rotation axis is being changed)
- I = moment of inertia of the rotor

**What this means:**
- If you try to forcibly rotate a fast-spinning wheel around a perpendicular axis, you feel a strong resistance — the wheel "fights back" against the change to its orientation
- This resistance to rotational change is used to maintain a stable reference direction
- The *stronger the spinning speed* and *greater the rotor inertia*, the more stable the gyroscope

**For navigation:**
- The spinning axis must be initially aligned with a reference direction (e.g., north-south meridian)
- If aligned with the north-south meridian: Earth's rotation has no effect on the horizontal axis — the gyro reads only robot rotation
- If aligned east-west: the horizontal axis reads Earth's rotation rate

*Analogy:* A bicycle wheel held by its axle while spinning fast resists being tilted. That resistance is the gyroscopic effect — the wheel's angular momentum makes it want to keep its axis pointing in the same direction.

#### Optical Gyroscope

**Principle:** Uses two laser beams (not mechanical spinning parts) based on the *Sagnac effect* — the principle that the speed of light is constant but geometric path length can change.

**How it works:**
1. Two monochromatic laser beams are emitted from the same source
2. One beam travels *clockwise* around a circular fiber-optic path
3. The other travels *counterclockwise* around the same path
4. If the gyroscope is *rotating*, the beam going in the direction of rotation has a slightly shorter effective path → *higher frequency*
5. The beam going against rotation has a slightly longer path → *lower frequency*
6. **Δf (frequency difference)** ∝ **Ω (angular velocity)**
7. By measuring Δf, you measure the rotation rate

**Advantages over mechanical:**
- No moving parts → no mechanical wear, vibration sensitivity, or warm-up time
- Solid-state versions using microfabrication technology (MEMS gyros)
- Resolution can be < 0.0001°/hr; bandwidth > 100 kHz
- Far exceeds the needs of most mobile robotic applications

*Analogy:* Two runners on a circular track, both starting at the same time. If the track starts spinning (rotating), the runner going in the rotation direction finishes their lap slightly sooner, the other slightly later. The difference in their lap times tells you how fast the track is spinning. Light beams are the "runners."

---

### 4.4 Accelerometer

**Definition:** An accelerometer is a device that measures *all external forces acting upon it*, including gravity. It belongs to the *proprioceptive* sensor class.

*Analogy:* Imagine you're in an elevator with a scale under your feet. When the elevator accelerates upward, the scale reads more than your true weight. When it decelerates, it reads less. The accelerometer is essentially that scale — it measures force, which (via Newton's 2nd law, F = ma) reveals acceleration.

**Physical Model (Spring-Mass-Damper):**

The accelerometer contains a *proof mass* m suspended by a spring (stiffness k) and damper (coefficient c) inside a casing.

When an external force is applied to the casing:

> **F_applied = F_inertial + F_damping + F_spring**
> **F_applied = mẍ + cẋ + kx**

Where:
- x = displacement of proof mass from equilibrium
- ẋ = velocity of proof mass
- ẍ = acceleration of proof mass
- m = proof mass
- c = damping coefficient
- k = spring constant

By measuring x (the displacement of the proof mass), you can solve for the applied acceleration.

**What an accelerometer measures:**
- **Specific force** — linear acceleration minus gravitational acceleration
- At rest on Earth: reads 1g (9.81 m/s²) pointing upward — because the ground is pushing up on it with 1g to counteract gravity
- In free fall: reads 0g — nothing counteracting gravity
- During robot motion: reads the combination of motion acceleration and gravity component

**Importance in Robotics:**

**Motion Detection** — Detect and measure movement and velocity changes. Essential for navigation and motion control.

**Orientation Estimation** — At rest, gravity always points straight down. By measuring which axis of the accelerometer experiences gravitational force, you can estimate tilt angle and orientation relative to vertical. (This only works when the robot is not also accelerating from motion.)

**Impact Detection** — Sudden large acceleration spikes indicate collisions or drops. Can trigger emergency stops or safety measures.

**Gesture Recognition** — In wearable or humanoid robotics, accelerometers detect user body movements for natural interaction.

**Applications in Practice:**

- **IMUs:** Accelerometers + gyroscopes combined form IMUs (see next section)
- **Robot Navigation (Dead Reckoning):** Integrate acceleration once → velocity; integrate velocity → position (double integration). But errors accumulate very quickly.
- **Drone Stability Control:** Accelerometers give feedback for leveling quadcopter attitude. Motor speeds are adjusted based on sensed tilt.
- **Fall Detection:** In assistive robots and healthcare, detect falls (sudden high acceleration followed by sudden stop) and trigger alerts.

---

### 4.5 Inertial Measurement Unit (IMU)

**Definition:** An IMU (also called an Inertial Navigation System, INS) is a device that integrates *gyroscopes and accelerometers* (and often magnetometers) to estimate the complete **6-DOF pose** (position + orientation) of a moving vehicle.

The 6-DOF pose consists of:
- **Position:** x, y, z
- **Orientation:** roll (rotation about x-axis), pitch (rotation about y-axis), yaw (rotation about z-axis)

*Analogy:* The IMU is the combination of your inner ear (gyroscope = sense of rotation) and the feeling in your muscles and skin (accelerometer = sense of linear force/acceleration). Together they give you a complete sense of where your body is in space and how it's moving — without looking.

**How an IMU Works (Block Diagram Overview):**

```
Gyroscope → Angular velocity (ω) → Integrate → Orientation (roll, pitch, yaw)
                                          ↓
Accelerometer → Linear acceleration → Transform to navigation frame (using current orientation)
                                    → Integrate → Velocity
                                    → Integrate again → Position
```

**Step-by-step process:**
1. **Gyroscope** measures angular velocity (how fast the orientation is changing)
2. Integrate gyroscope data → current *orientation* (roll, pitch, yaw)
3. **Accelerometer** measures specific force (acceleration + gravity)
4. Using the current orientation, subtract the gravity vector → net linear acceleration
5. Integrate linear acceleration → *velocity* (needs initial velocity as input — usually zero at rest)
6. Integrate velocity → *position*

**The Critical Problem: Drift**

**IMUs are highly sensitive to measurement errors.** Errors in both gyroscopes and accelerometers cause the estimated position and orientation to drift progressively further from reality.

- **Gyroscope drift:** Small errors in angular velocity measurement accumulate → wrong orientation estimate → incorrect gravity vector subtraction
- **Accelerometer error propagation:** Since position = ∫∫ acceleration dt², *any* residual error in acceleration is integrated twice → position error grows *quadratically* over time

*Analogy:* Imagine estimating your position by closing your eyes and counting your steps. Each step measurement might be off by 1 cm. After 10 steps, you might be 10 cm off. After 1000 steps, you could be meters away from your true position. The IMU's drift is this accumulation of tiny integration errors.

**Drift Mitigation:**
- **GPS:** Provides periodic absolute position corrections outdoors
- **Cameras (visual odometry):** Observe known features in the environment to correct orientation and position estimates
- **Sensor fusion (Kalman filter):** Mathematically combines IMU predictions (fast, continuous) with periodic absolute corrections (slow, accurate)
- **Magnetometer:** Provides absolute heading reference (north) to correct yaw drift from the gyroscope

---

### 4.6 GPS — Global Positioning System

**Background:** Originally developed for military use, GPS is now freely available for civilian navigation. It gives any receiver on (or near) Earth's surface a *global position and time*.

**System Architecture:**
- **24+ GPS satellites** in orbit at ~20,000 km altitude
- Each satellite continuously broadcasts its *current position* and *precise time* (using atomic clocks)
- Satellites orbit every 12 hours; 4 satellites in each of 6 orbital planes inclined at 55° to Earth's equator
- Always at least 4 satellites visible from any point on Earth's surface (in open sky)
- **Control stations** on Earth monitor and update satellite positions and clocks
- **GPS receivers** on robots/devices calculate their own position by receiving satellite signals

**How GPS Works (Triangulation by Time of Arrival):**

1. Satellites synchronize their transmissions — all signals sent at the same time
2. The GPS receiver reads transmissions from 4+ satellites
3. Each signal arrives at a *slightly different time* — signals from closer satellites arrive sooner
4. The *arrival time difference* tells the receiver its *relative distance* to each satellite:
   > **Distance to satellite = (time of arrival) × (speed of light)**
5. With distance measurements to 4 satellites, the receiver can solve for its 3D position (x, y, z) and also correct its own clock error
   - 3 satellites give 3D position (3 unknowns: x, y, z)
   - 4th satellite corrects the receiver's clock error (the receiver clock isn't atomic-grade)

*Analogy:* Imagine three people at known locations shouting "Hello!" at the same time. You hear each voice at a slightly different time because sound travels at finite speed. The *delay differences* tell you how far you are from each person. GPS is the same, but with satellites and light-speed radio signals instead.

**GPS Characteristics:**
- Completely **passive** sensor — the receiver only receives, never transmits → cannot be jammed by "listening" → BUT also exteroceptive (measures from external reference)
- Timing is critical — GPS measures travel times in *nanoseconds*
- Satellites carry *atomic clocks* synchronized to nanosecond precision
- Update rate: typically 200–300 ms latency → maximum ~5 Hz update rate

**Accuracy Levels:**

| Method | Resolution |
|---|---|
| Basic pseudorange | ~15 meters |
| Differential GPS (DGPS) | ~1 meter or better |
| Carrier phase measurement | ~1 cm (sub-centimeter with multiple receivers) |

**DGPS:** Uses a *second stationary receiver* at a precisely known position to compute and broadcast corrections for atmospheric and clock errors. The moving robot's receiver applies these corrections to its own measurements.

**Limitations of GPS:**

**Line-of-sight required:** GPS satellite signals are extremely low power. Requires clear view of the sky. Cannot penetrate buildings, tunnels, or dense foliage.

**Urban canyon effect:** Tall buildings in cities reflect GPS signals, causing *multipath errors* — the receiver gets the same signal from multiple paths, confusing its calculations.

**Indoors:** Almost completely ineffective. Most indoor environments do not provide sufficient sky visibility.

**Accuracy variation by latitude:** At the poles, satellites are near the horizon → poor altitude resolution. Equatorial regions have best coverage.

**Latency:** 200–300 ms update delay. Fast-moving robots need motion integration (dead reckoning) to compensate for this delay during real-time control.

**Best use cases:** Outdoor navigation in open environments — fields, roads, large open spaces, aerial robots (UAVs).

---

## 5. Perception Sensors — Range Finders

Range finders measure the *distance* from the robot to objects in its environment. They are classified by their *measurement principle*:

1. **Time-of-Flight (ToF):** Measure the travel time of an emitted signal (sound or light)
2. **Triangulation:** Use geometry to determine distance from angle of reflection

---

### 5.1 Time-of-Flight: Ultrasonic Sensors

**Principle:** Emit a packet of ultrasonic pressure waves (inaudible to humans, typically 40–250 kHz). Measure the time it takes for the echo to return from a reflecting surface.

**Distance Formula:**

> **d = (c × t) / 2**

Where:
- d = distance to the reflecting object
- c = speed of sound in air
- t = total time of flight (there-and-back)
- Divide by 2 because the signal travels to the object AND back

**Speed of Sound:**

The speed of sound depends on the medium's temperature:

> **c = √(γRT)**

Where:
- γ = ratio of specific heats of air (≈ 1.4)
- R = gas constant for air
- T = temperature in Kelvin

At standard pressure and 20°C: c ≈ **343 m/s**

**This means the speed of sound is temperature-dependent.** A sensor calibrated at 20°C will give slightly wrong distance readings at 30°C or 0°C unless temperature compensation is applied.

**Error Sources in Ultrasonic Sensors:**
- Uncertainty in determining the exact moment the echo arrives (threshold detection vs. true peak)
- Inaccuracies in time-of-flight measurement circuitry
- **Dispersal cone** — ultrasonic waves spread out in a cone (typically 15–30° wide), not a thin beam. The sensor can't pinpoint the exact angle to the reflecting object. You know something is within the cone at distance d, but not exactly where.
- **Surface absorption** — soft surfaces (carpet, foam) absorb sound → weak or no echo
- **Specular reflections** — smooth surfaces at an angle reflect the beam away from the sensor, like a mirror → no return signal → robot thinks nothing is there
- **Variation of propagation speed** — temperature changes affect c
- Multiple sensors on the same robot can interfere with each other (one sensor's emission is detected by another's receiver)

**Range:** Typically effective from a few cm to ~5–10 m.

**Applications:** Proximity detection, parking sensors (in cars), short-range obstacle avoidance, indoor robot navigation where precision is not critical.

---

### 5.2 Time-of-Flight: Laser Rangefinders and LiDAR

**LiDAR (Light Detection and Ranging):**

LiDAR uses *laser light* instead of sound for distance measurement. The same time-of-flight principle applies, but light travels at ~3×10⁸ m/s instead of 343 m/s.

**Components:**
- **Transmitter:** Emits a collimated (focused, parallel) beam of laser light
- **Receiver:** Detects the reflected light component
- **Timing circuitry:** Measures the round-trip travel time with nanosecond precision
- **Scanning mechanism:** A rotating mirror or MEMS mirror sweeps the beam to cover a plane (2D LiDAR) or full 3D volume (3D LiDAR)

**Distance Formula:**

> **d = (c_light × t) / 2**

Where c_light ≈ 3 × 10⁸ m/s

Because light is so fast, measuring the travel time for 1 meter round trip (2 m total travel) requires resolving ~6.7 *nanoseconds* — demanding very precise timing electronics.

**2D vs. 3D LiDAR:**
- **2D LiDAR:** Scans a single horizontal plane. Gives a 2D distance map around the robot. Used for indoor mapping, obstacle detection in the horizontal plane. Common in service robots, autonomous forklifts.
- **3D LiDAR:** Multiple scan planes (rotating multi-beam) or 3D scanning mechanism. Generates a full 3D *point cloud* of the environment. Used in self-driving cars (Velodyne HDL-64E produces 64 simultaneous scan lines at 10-20 rotations/second → millions of points/sec).

**What LiDAR enables:**
- **3D mapping:** Dense, accurate point clouds of surroundings
- **Obstacle detection and avoidance:** Real-time 3D obstacle identification
- **SLAM:** LiDAR scans are matched over time to build maps and localize the robot
- **Object recognition:** Classifying point cloud clusters as cars, pedestrians, buildings
- **Navigation:** Planning collision-free paths through a point cloud world model

**Advantages over ultrasonic:**
- Much more precise (cm to mm accuracy vs. cm range for ultrasonic)
- Narrow beam → pinpoint angle resolution (can pinpoint where an object is)
- Long range (up to 100+ m for automotive LiDAR)
- 2D/3D scanning capability

**Limitations:**
- **Rain and fog:** Water droplets scatter laser light → range and accuracy drop significantly
- **Reflectivity issues:** Black/absorptive surfaces return little light; highly reflective surfaces (mirrors, retroreflectors) can overwhelm the detector
- **Cost:** High-quality 3D LiDAR units have historically been very expensive (though costs are dropping rapidly)
- **Motion distortion:** If the robot moves while scanning, the resulting point cloud is distorted because different points were captured at different times

---

### 5.3 Time-of-Flight Camera

A ToF camera applies the time-of-flight principle to capture a *complete 2D depth image* (one depth value per pixel) simultaneously, without any moving parts.

**How it works:**
1. An **infrared illumination source** (modulated — switched on and off rapidly) illuminates the scene
2. A **Photonic Mixer Device (PMD) sensor array** captures the returned light at each pixel
3. For each pixel, the phase shift between emitted and received light determines the *distance* to the corresponding scene point
4. The result: a full **depth map** — every pixel has a distance value

**To eliminate background ambient light interference:**
The acquisition is performed *twice*:
- Once WITH illumination (signal + background)
- Once WITHOUT illumination (background only)
- Subtraction removes the background contribution → clean depth measurement

**Frame rate:** Up to 100 FPS → real-time depth imaging

**Comparison with LiDAR and Stereo Vision:**

| Property | ToF Camera | 2D/3D LiDAR | Stereo Camera |
|---|---|---|---|
| Moving parts | No | Yes (rotating) | No |
| Range | Short-medium (<10m) | Long (up to 100m+) | Medium |
| Frame rate | Up to 100 FPS | 10-20 Hz (rotating) | Depends on processing |
| Cost | Moderate | High (3D LiDAR) | Low |
| Works in dark | Yes (active) | Yes (active) | No (passive) |
| Use case | Indoor, manipulation | Outdoor navigation | Passive outdoor |

---

### 5.4 Triangulation Active Ranging

**Principle:** Use *geometry* rather than timing to measure distance. A known light pattern is projected onto the environment. The *position* of the reflection on a detector tells you the distance to the reflecting surface.

*Analogy:* Hold a flashlight at arm's length and shine it on a wall. Now move the wall closer — the reflection spot stays on the wall, but its *angle* changes relative to your fixed flashlight position. Measuring that angle change tells you the distance.

#### 5.4.1 Optical Triangulation (1D Sensor)

**Setup:**
- A collimated beam (focused infrared LED or laser) is transmitted toward the target
- The reflected light is collected by a lens
- The lens focuses the reflected light onto a **Position-Sensitive Device (PSD)** or **linear camera** (1D sensor array)

**How distance is calculated:**
- The geometric relationship between the emitter, lens, and PSD is known precisely
- The *position* of the reflected spot on the PSD changes as the target distance changes
- Using trigonometry (the triangle formed by emitter, target, and detector), distance D is calculated

**Key formula (triangulation geometry):**
> **D = (f × b) / x**

Where:
- f = focal length of the lens
- b = baseline (distance between emitter and lens/detector)
- x = position of spot on PSD

**Advantages:** Simple, cheap, works well for short ranges.

**Limitations:** Only measures one point at a time; sensitive to surface reflectivity; baseline limits minimum detectable range (very close objects have large angles that fall off the PSD).

#### 5.4.2 Structured Light (2D Sensor)

Structured light sensors extend optical triangulation to measure *many points simultaneously*, recovering a 2D depth image.

**Changes from 1D:**
- Replace the PSD/linear camera with a **2D camera (CCD or CMOS)**
- Replace the single beam with a **known light pattern** projected onto the scene:
  - Light textures (random speckle patterns — used in Microsoft Kinect v1)
  - Laser stripes (one at a time → faster line-by-line scanning, or multiple stripes → parallel)
  - Grid patterns, coded patterns

**How it works:**
1. The projector emits the known pattern onto the scene
2. The camera captures the scene with the reflected pattern
3. Where the pattern appears on the camera image reveals the 3D geometry of the surface (because the pattern deforms according to depth)
4. The 2D camera image is filtered to identify and locate the pattern's reflection
5. Per-pixel depth is recovered by triangulation

**Advantages over passive stereo:**
- **Works in featureless environments:** Passive stereo requires distinct visual features to match between camera views. Structured light projects its own features (the pattern) — so it works even on blank white walls or surfaces with uniform color.
- **Works in the dark:** Completely active — projects its own illumination.
- Simpler correspondence problem — pattern pixels have known identities.

**Applications:** 3D scanning (industrial inspection), Microsoft Kinect (gaming + robotics), facial recognition (Apple Face ID uses structured IR light), augmented reality.

---

### 5.5 RADAR

**RADAR (Radio Detection And Ranging):** A sensing system that detects, locates, and tracks objects using *electromagnetic radio waves*.

**How Radar Works:**
1. A **transmitter** emits radio waves (typically in GHz frequency range)
2. The waves propagate through the air and hit a **target object**
3. The object **reflects** (echoes) the waves back
4. A **receiver** captures the reflected signal
5. The **time delay** between transmission and reception → distance:

> **Distance = (c × time_delay) / 2**

Where c = speed of light ≈ 3×10⁸ m/s

**What Radar Can Measure:**
- **Distance (range):** From round-trip time delay
- **Speed (velocity):** Via the **Doppler effect** — if the target is moving toward the radar, reflected frequency is higher; if moving away, lower. The frequency difference (Doppler shift) is proportional to the target's radial velocity.
  > Δf = 2 × v × f₀ / c (where v = target velocity, f₀ = transmitted frequency)
- **Direction/Angle:** Using beam steering (phased arrays) or multiple antennas
- **Object presence:** Simply detecting whether something is there

**Key Characteristics:**
- **All-weather operation:** Radio waves at radar frequencies penetrate fog, rain, dust, snow — conditions that blind cameras and severely degrade LiDAR
- **Works in darkness:** Active sensor, completely independent of lighting
- **Long range:** Automotive radar commonly reaches 200+ m
- **Robust:** Doesn't care about surface texture or color (unlike cameras)
- **Complements cameras and LiDAR:** Makes up for their weaknesses in adverse weather

**Applications in Robotics:**

**Autonomous Navigation** — Detects obstacles in poor visibility conditions (fog, heavy rain, smoke). Essential for outdoor autonomous vehicles in unpredictable weather.

**Object Detection and Tracking** — Tracks moving objects, estimates velocity, predicts future position → critical for collision avoidance with moving obstacles (other vehicles, people).

**Human-Robot Interaction** — Radar sensors detect human presence and movement through walls or in darkness. Enables occupancy sensing, people counting, safety zones.

**Automotive Systems** — Adaptive cruise control (maintains following distance), blind-spot monitoring, forward collision warning, automated emergency braking.

**Comparison: LiDAR vs. Radar vs. Cameras**

| Property | Camera | LiDAR | Radar |
|---|---|---|---|
| Resolution | Very high (visual) | High (3D points) | Low |
| Range | Moderate | Long | Very long |
| Adverse weather | Poor | Poor-moderate | Excellent |
| Dark operation | Poor (passive) | Excellent (active) | Excellent (active) |
| Velocity measurement | Via optical flow | Limited | Direct (Doppler) |
| Cost | Low | High | Moderate |
| Object classification | Excellent (AI-assisted) | Good | Poor |

---

## 6. Fundamentals of Computer Vision

**Computer Vision** is the field of enabling machines to understand images. Also called: Image Analysis, Scene Analysis, Image Understanding.

**Goal:** Duplicate human vision capabilities electronically — acquiring, processing, analyzing, and understanding images to extract useful information.

**Why vision matters:** Vision is the most powerful human sense, providing enormous environmental information and enabling rich, intelligent interaction with dynamic environments. Vision data can come from video sequences, depth images, multiple cameras, medical scanners, satellite sensors, and more.

---

### 6.1 The Digital Camera

**How a digital camera creates an image:**

1. **Light** from a scene reflects off surfaces and passes through the camera's **lens system**
2. The focused light reaches the **imaging sensor** (CCD or CMOS chip)
3. **Photons** arriving at each pixel of the sensor are converted into electronic charge (electrons liberated by the photoelectric effect)
4. Charge is accumulated during the **integration period** (exposure time / shutter speed)
5. After integration, accumulated charge is read out and converted to **digital (R, G, B) values** through sense amplifiers and A/D converters
6. The result is a digital image — a 2D matrix of pixel values

**Key parameters:**
- **Iris (aperture):** Controls how much light enters. Larger aperture = more light, shallower depth of field.
- **Shutter speed:** How long the sensor integrates light. Slow shutter = more light but motion blur. Fast shutter = frozen action but needs more light.
- **Camera gain (ISO):** Amplifies the signal from the sensor. High gain = amplifies the signal but also amplifies noise.

---

### 6.2 CCD vs CMOS Sensors

Both CCD and CMOS are semiconductor sensor types used in digital cameras. They capture light the same way but *read out the captured data differently*.

#### CCD (Charge-Coupled Device)

**Construction:** Array of photodiodes. Each pixel is a discharging capacitor that accumulates charge when hit by photons.

**Reading out data:**
- After exposure, charge from each pixel is *physically shifted* (transferred) across the chip, row by row and column by column, to *one corner* of the chip
- There, specialized circuitry reads the charge value
- This serial readout requires *specialized clock drivers and control circuitry*

**Light sensitivity:**
- Photodiodes are sensitive to 400–1000 nm wavelength (visible + near-infrared)
- Decreased sensitivity to UV; increased sensitivity to IR (IR filter often placed in front)

**CCD Advantages:**
- High image quality, low noise (because signal is carefully transferred to one reading circuit)
- High dynamic range
- Historically better resolution

**CCD Disadvantages:**
- Requires specialized fabrication — cannot be made on standard microchip production lines
- High power consumption (complex charge transfer process)
- Slow readout (serial process)
- **Blooming:** At very high light intensity, a pixel overfills with electrons, and charge spills into neighboring pixels → bright spots cause halo artifacts

**Dynamic range limitation:** Each pixel has a "well capacity" (max electrons it can hold) — typically 20,000 to 350,000 electrons. Once full, the pixel saturates.

#### CMOS (Complementary Metal-Oxide-Semiconductor)

**Construction:** Like CCD but each pixel has its own *amplifier and A/D converter transistors* located next to the photodiode.

**Reading out data:**
- Each pixel's signal is measured and amplified *independently and in parallel*
- No need to transfer charge across the chip
- Direct readout — simple row/column addressing like RAM memory

**CMOS Advantages:**
- Can be manufactured on *standard semiconductor production lines* (same factories as microprocessors) → cheaper, higher volume
- Very low power consumption: ~1/100th of CCD
- Faster readout (all pixels can be read simultaneously)
- On-chip integration possible (ADC, processing circuitry on same chip)
- Critical advantage for mobile robots where power is scarce

**CMOS Disadvantages:**
- Transistors next to each pixel *consume chip area* → less light-collecting area per pixel → lower sensitivity than equivalent CCD
- More photons hit transistors instead of photodiodes → lower quantum efficiency
- Historically higher noise levels (fixed-pattern noise from transistor mismatch)
- Younger technology → historically lower peak resolution

**Current State:** CMOS has become dominant in virtually all digital cameras today (phones, professional cameras, robotics cameras). Advances in CMOS technology have largely closed the quality gap with CCD. For mobile robotics (where power consumption matters greatly), CMOS is strongly preferred.

**CCD vs CMOS Summary:**

| Property | CCD | CMOS |
|---|---|---|
| Image quality | Higher (historically) | Comparable now |
| Power consumption | High | ~100× lower |
| Cost | High | Low |
| Readout speed | Slow (serial) | Fast (parallel) |
| Manufacturing | Specialized process | Standard chip process |
| On-chip integration | Difficult | Easy |
| Noise | Lower | Historically higher (improving) |
| Mobile robotics preference | Less suitable | **Preferred** |

---

### 6.3 Color Cameras — Bayer Filter and Demosaicing

The basic photodiode is *colorblind* — it only counts photons regardless of color. To capture color, filters are used.

#### Single-Chip Color (Bayer Filter)

**How it works:**
- Pixels are grouped into 2×2 blocks
- A color filter array (the **Bayer filter**) is placed over the chip:
  - 2 pixels per 2×2 block measure **Green**
  - 1 pixel measures **Red**
  - 1 pixel measures **Blue**
- Each pixel sees only one color of light

**Why 2 green pixels?** The human visual system is *most sensitive to luminance (brightness) detail*, which is mostly carried by the green channel. Adding more green pixels improves the spatial resolution of the most perceptually important channel.

**Demosaicing:** Since each pixel only has one color value, the *missing* color values must be *interpolated* from neighboring pixels. This process is called **demosaicing** (or debayering).

*Analogy:* Imagine a tile floor where only every other tile shows a color and the rest are gray. You'd estimate the color of the gray tiles by looking at their neighbors. Demosaicing does this mathematically.

**Resolution disadvantage:** The effective resolution is reduced by ~4× compared to a monochrome sensor of the same size, since only 1/4 of pixels measure each color. Demosaicing partially recovers perceived sharpness through interpolation.

#### Three-Chip Color Camera

- Incoming light is split by a **beam splitter prism** into three complete copies
- Each copy goes to a *separate full-resolution chip* with a single color filter (R, G, or B)
- All three chips measure their respective color at full resolution simultaneously

**Advantage:** Full resolution preserved in all three channels — significantly better color fidelity and resolution.

**Disadvantage:** Much more expensive (three chips, complex optics). *Rarely used in mobile robotics* due to cost and size.

**White Balance:** Both types suffer from color inconsistency under different light sources (sunlight is different from fluorescent lighting, which is different from incandescent). White balance controls adjust the relative gains of R, G, B channels so that a white object appears white regardless of the light source.

---

### 6.4 Image Formation Optics

Understanding how a camera lens forms an image is fundamental to understanding depth estimation and camera modeling.

#### The Lens Law

The fundamental relationship between object distance, image distance, and focal length:

> **1/u + 1/v = 1/f**

Where:
- u = distance from lens to the object (object distance)
- v = distance from lens to the image plane (image distance)
- f = focal length of the lens (a fixed property of the lens design)

**Depth from Focus:** Rearranging the lens law:
> u = (f × v) / (v - f)

If you *know the focal length* and can measure v (the distance at which the image is in focus on the sensor), you can *calculate the distance to the object*. This technique is called **depth from focus**.

#### Depth of Field and Circle of Confusion

If the image plane is *not exactly at the correct distance* for a given object:
- Light from that object voxel spreads onto the sensor as a **blur circle** (also called **circle of confusion**)
- The radius R of this blur circle:

> **R = (L × |Δv|) / (2 × v)**

Where:
- L = diameter of the lens (aperture)
- Δv = displacement of image plane from the true focal point

**Depth of Field:** The range of object distances that appear acceptably sharp (blur circle smaller than a pixel).

**Aperture-Depth of Field trade-off:**
- **Smaller aperture (L↓):** Smaller blur circles → greater depth of field (more objects in focus simultaneously) → BUT less light → needs longer exposure (motion blur risk) or higher gain (more noise)
- **Larger aperture (L↑):** Larger blur circles → shallower depth of field → but more light

**Pinhole camera (extreme small aperture):**
- R → 0 as L → 0
- All objects at all distances are (approximately) in focus simultaneously
- But very little light → only works in very bright conditions
- The mathematical model of the pinhole camera (**perspective projection model**) is the standard model used in computer vision and robotics

**Zoom lens / long focal length:**
- Large f → high magnification → large blur circles at nearby objects but fine for distant objects → better range resolution at long distances
- Reduced field of view

*Analogy:* Your eye's pupil works like an adjustable aperture. In bright light, your pupil contracts (small aperture → wide depth of field → everything sharp). In dim light, your pupil dilates (large aperture → more light → shallower depth of field → background appears blurry while foreground is sharp, just like a portrait photograph).

---

## 7. Robot Vision in Different Platforms

### 7.1 UGVs — Unmanned Ground Vehicles

**Definition:** A UGV is a vehicle that operates on the ground without an onboard human. It may operate autonomously or be teleoperated from a remote location.

**Key system components:**
- **Vehicle control system** — manages motors, brakes, steering
- **Navigation system** — localization and path following
- **Obstacle detecting system** — sensors for safe navigation
- **Traffic signal monitoring** — perception of road signs and traffic signals

**Sensory system for UGVs:**

**RGB Cameras** — Primary sensor for visual perception: object recognition (detecting other vehicles, pedestrians, road signs), lane detection, terrain classification.

**Thermal Cameras** — Detect heat signatures. Useful for:
- Low-light and nighttime conditions (people and vehicles emit heat even in darkness)
- Detecting people in dense foliage or obscured environments (firefighting robots, search and rescue)

**LiDAR** — Creates detailed 3D maps: obstacle detection and avoidance, lane boundary detection, precise localization against a pre-built map.

**Depth/Stereo Cameras** — 3D scene understanding for manipulation and close-range obstacle avoidance.

**Ultrasonic sensors** — Short-range proximity detection (parking, slow-speed maneuvers).

**Multi-sensor fusion is crucial for UGVs** because:
- Environmental conditions (lighting, weather, surface type) affect individual sensors differently
- No single sensor is reliable in all conditions
- Fusing data from multiple sensors provides a robust, comprehensive environmental model

**Challenges specific to UGVs:**
- Varying lighting conditions (sunlight glare, shadows, night)
- Weather effects (rain on cameras, fog reducing LiDAR range)
- Occlusions (one vehicle hidden behind another)
- Computational and power constraints for real-time processing

**Applications:** Military reconnaissance, agriculture, mining, logistics (warehouse robots, autonomous trucks), inspection.

---

### 7.2 UAVs — Unmanned Aerial Vehicles

**Key UAV System Architecture (block diagram):**

```
User Command
    ↓
Path Planning Module → computes optimal collision-free path
    ↓
Flight Control Module → ensures stability and trajectory tracking
    ↓
Aerial Robot Real Motion → actual physical movement influenced by wind, disturbances
    ↓
Perception & State Estimation Module
    ├── INS (Inertial Navigation System)
    ├── Localization & Mapping
    └── Sensor Fusion
    ↓ (feedback to path planning)
Mission Results → displayed output (maps, images, detections)
```

**Sensor suite for UAVs:**

**RGB Cameras** — Object recognition and scene understanding. Most UAVs carry at least one forward-facing camera and one downward-facing camera.

**Thermal Cameras** — Essential for nighttime operations, search and rescue (detecting body heat), detecting heat signatures in infrastructure inspection (power line hotspots, roof insulation failures).

**Multispectral Cameras** — Capture data beyond the visible spectrum (near-infrared, red-edge, etc.). Applications:
- **Agriculture:** Detecting plant stress, water stress, nutrient deficiency (plants that look healthy in visible light may show stress in near-IR)
- **Environmental monitoring:** Vegetation health, water quality
- **Resource exploration:** Geological mapping

**Challenges for UAV vision:**
- Limited sensor resolution in low-light conditions (weight constraints limit sensor quality)
- Computational constraints — powerful processors are heavy; processing must be efficient
- Real-time processing requirement at high frame rates for fast-moving platforms
- Safety in congested environments (GPS-denied spaces, crowded airspace)
- Vibration from motors affects camera stability → vibration isolation or electronic image stabilization needed

---

### 7.3 AUVs — Autonomous Underwater Vehicles

AUVs face unique perception challenges due to the underwater environment:

**Why cameras are limited underwater:**
- Light attenuates rapidly in water — only the blue-green spectrum penetrates to significant depths
- Turbidity (suspended particles) scatters light → poor visibility
- Bioluminescence and refraction affect image quality

**Key sensors for AUVs:**

**Sonar (Sound Navigation and Ranging):**
- Sound travels much better in water than light
- **Forward-looking sonar:** Obstacle avoidance and navigation
- **Side-scan sonar:** Maps the seafloor by sweeping acoustic beams sideways
- **Multi-beam sonar:** Creates detailed 3D maps of the seafloor
- **USBL (Ultra-Short Baseline):** Acoustic positioning system for AUV localization

**Underwater cameras:** Work at shallow depths for visual inspection, coral reef mapping, ship hull inspection.

**DVL (Doppler Velocity Log):** Acoustic sensor that measures AUV velocity by bouncing sound off the seafloor — used for dead reckoning navigation.

**Pressure sensors:** Measure depth.

**IMU + DVL + USBL:** Standard fusion combination for AUV navigation — IMU provides high-rate motion estimation, DVL corrects velocity, USBL corrects absolute position.

---

## 8. Image Processing Fundamentals

### What is Image Processing?

**Definition:** Image processing involves manipulation and analysis of images using algorithms. It:
- Enhances image quality (noise removal, contrast enhancement)
- Extracts meaningful information from images
- Forms the foundation of computer vision and AI perception systems

### Digital Image Representation

An image is represented as a **matrix (array) of pixels**. Each pixel has:
- **Location:** (row, column) coordinates
- **Intensity value:** a number representing brightness or color

**Types of images:**

| Type | Pixel values | Channels |
|---|---|---|
| Binary image | 0 (black) or 1 (white) | 1 |
| Grayscale image | 0 (black) to 255 (white) — 8-bit | 1 |
| Color image (RGB) | 0–255 per channel | 3 (Red, Green, Blue) |
| HDR image | Floating point values | 3+ |

A color image is stored as three separate 2D arrays (one per channel R, G, B), each of the same pixel dimensions.

### Image Acquisition Pipeline

**Step 1 — Sensing:** The camera sensor (CCD or CMOS) converts incoming photons into electrical signals.

**Step 2 — Sampling (Spatial Discretization):** The continuous 2D light field is divided into a finite grid of pixels. Higher sampling rate = better spatial resolution (more pixels = finer detail).

**Step 3 — Quantization (Intensity Discretization):** Each pixel's continuous analog signal is converted to a discrete integer value. 8-bit quantization gives 256 levels. Higher bit depth = more intensity levels = better dynamic range.

> **Higher sampling** → better spatial resolution (captures finer detail)
> **Higher quantization** → better intensity resolution (smoother gradients, less banding)

### Image Processing Pipeline

```
Image Acquisition
        ↓
Preprocessing (noise removal, filtering, contrast enhancement)
        ↓
Segmentation (divide image into meaningful regions)
        ↓
Feature Extraction (identify key geometric or visual features)
        ↓
Interpretation / Decision Making
```

Each stage reduces data volume while increasing semantic content.

---

## 9. Feature Extraction

### 9.1 What Are Features?

Autonomous mobile robots use sensor measurements to determine their relationship to the environment. Sensors introduce uncertainty and error. Two strategies for handling uncertain sensor input:

1. **Use raw sensor measurements directly** — feed raw data into robot behavior (e.g., raw LiDAR distance triggers an emergency stop when below a threshold)
2. **Extract higher-level perceptual information first** — process raw data into meaningful features, then use features for behavior (e.g., extract wall boundaries from LiDAR data, then use those boundaries for navigation)

**Feature Extraction** is the process of deriving these higher-level percepts from sensor readings.

**When each strategy is used:**
- **Immediate obstacle detection:** Raw sensor readings → emergency stop trigger. Speed and simplicity matter most.
- **Local obstacle avoidance:** Raw readings fed into an *occupancy grid* model → smooth avoidance behavior.
- **Map-building and precise navigation:** Full feature extraction and scene interpretation → minimizes individual sensor uncertainty, enables robust long-term maps.

The more sophisticated the task, the more reliance is placed on feature extraction.

*Analogy:* A self-driving car's camera feeds raw pixels to a collision detector (fast response) AND to a deep learning object detector (semantic understanding). Simple reactions use raw data; complex decision-making uses features.

---

### 9.2 Feature Definition and Hierarchy

**Definition:** Features are *recognizable structures in the environment* that can be *mathematically described* and reliably detected across sensor readings.

#### Feature Hierarchy (from raw to abstract):

**Level 0 — Raw Sensor Data:** Millions of LiDAR points, millions of image pixels. Maximum data volume, minimum distinctiveness — almost every point looks like almost every other.

**Level 1 — Low-Level Features (Geometric Primitives):**
- Lines, points, corners, blobs, circles, polygons
- Directly derived from raw data via local operations
- Intermediate distinctiveness — a corner is more distinctive than a random pixel
- *Analogy:* Identifying individual words in a paragraph rather than working letter-by-letter.

**Level 2 — High-Level Features (Objects):**
- Doors, tables, chairs, people, cars, trash cans
- Derived from combinations of low-level features
- Maximum abstraction from raw data
- Maximum distinctiveness — a specific object is unmistakable
- *Analogy:* Identifying sentences and their meanings rather than individual words.

**Key trade-off:**
> Raw data: high volume, low distinctiveness
> Low-level features: medium volume, medium distinctiveness
> High-level features: low volume, high distinctiveness

**Why spatial locality matters:** Features must be localized in space — they must correspond to a specific location in the environment. Their geometric extent can range from a single point (corner feature) to an extended region (wall segment).

**Role in mobile robotics:**
- Building compact, robust **environmental models** (maps)
- Enabling **localization** by recognizing previously seen features
- Supporting **SLAM** by providing consistent landmarks

---

### 9.3 Factors Influencing Feature Choice

The choice of which features to use is determined by four factors:

#### 1. Target Environment

Features must be detectable in the actual operating environment.

- **Office buildings / indoor structured environments:** Abundant straight walls → *line features* are ideal. Corners of walls, floor-ceiling intersections are reliable line sources.
- **Natural / unstructured environments (Mars, forests):** No straight walls → line features are useless. *Point features* (corners, blobs, texture patches) are more general — found in any textured environment.
- **Mars rovers (Spirit and Opportunity):** Used *corner features for visual odometry*, leveraging the naturally textured (but line-free) Martian surface.

#### 2. Available Sensors

Sensor capabilities determine which features can be reliably extracted.

- **Laser rangefinders:** High angular resolution, accurate depth → ideal for extracting precise *geometric features* (corners, line segments) from range data
- **Sonar/ultrasonic sensors:** Wide dispersion cone, poor angular resolution → corner extraction is unreliable; better suited to simple proximity detection
- **Cameras:** Rich visual texture → point features (Harris corners, SIFT, ORB keypoints) from image data

#### 3. Computational Power

Visual feature extraction is computationally expensive.

- Processing megapixel images in real-time requires significant CPU/GPU resources
- On resource-constrained robots (small drones, embedded systems), simpler features may be necessary
- Trade-off: more distinctive features (better navigation) vs. computational budget
- Hardware accelerators (GPUs, NPUs) help run more sophisticated feature extractors in real-time

#### 4. Environment Representation

The type of features must match the type of map/model being built.

- **Geometric models (metric maps):** Line and corner features are ideal — they directly encode geometric structure
- **Topological models (graph-based maps):** Non-geometric visual features (appearance descriptors of places) are more suitable
- The chosen features must be compatible with the downstream estimation and navigation algorithms

---

## 10. Interest Point Detectors

**Local features** (also called *interest points*, *keypoints*, or *interest regions*) are image patterns that *differ from their immediate surroundings* in intensity, color, or texture. They can be:
- Small image patches
- Edge points
- Corner points (intersections of two or more edges)

### Categories of Local Features

**Category 1 — Semantically meaningful features:** Features with a direct real-world interpretation.
- Examples: Edges corresponding to road lane markings, blobs representing blood cells in medical images, corners of doors in hallways

**Category 2 — Correspondence features:** Features *without* individual semantic interpretation, but whose *reliable localization* across time and viewpoint is valuable.
- Used for: feature tracking (follow a point across video frames), camera calibration, 3D reconstruction, image mosaicing (stitching panoramas)

**Category 3 — Bag-of-words features:** Features that lack individual semantic meaning but *collectively* enable recognition tasks.
- Used for: Scene recognition (is this the same room?), object recognition (is this a car?), texture analysis, image retrieval
- *Visual-word-based place recognition* — matches features between a database of known places and the current camera view to identify the current location

---

### 10.1 Properties of an Ideal Feature Detector

**1. Repeatability**
A good detector finds the same feature in multiple views of the same scene, even under:
- Different viewing angles
- Different illumination conditions
- Some image noise
- *Analogy:* A reliable person you can always find at their desk, no matter what time you come by.

**2. Distinctiveness**
Each detected feature should be *distinguishable from all others* — the patch around a feature point should carry unique information that doesn't appear at thousands of other locations.
- *Analogy:* A distinctive landmark (the Eiffel Tower) vs. a non-distinctive landmark (any lamp post).

**3. Localization Accuracy**
Features must be precisely located in both image position AND scale.
- Critical for: camera calibration (needs sub-pixel accuracy), 3D reconstruction, panorama stitching
- Poor localization → accumulated errors in downstream tasks

**4. Quantity**
The number of detected features must match the application:
- Object recognition / 3D reconstruction: benefit from many features (dense coverage)
- Tracking: fewer high-quality features may be preferable
- Too few features → insufficient constraints for reconstruction
- Too many features → computational burden

**5. Invariance**
Features should be detectable consistently despite image transformations:
- **Rotation invariance:** Detect the same corner regardless of camera rotation
- **Scale invariance:** Detect the same corner whether zoomed in or out
- **Illumination invariance:** Detect the same corner in different lighting
- **Affine invariance:** Detect the same corner under small viewpoint changes
- Achieved through mathematical normalization and transformation-robust descriptors

**6. Computational Efficiency**
Real-time robotics and large-scale image processing require *fast* detection and matching.
- More invariance generally requires more computation
- Trade-off: rich invariance vs. real-time speed

**7. Robustness**
Features should tolerate:
- Image noise
- JPEG compression artifacts
- Motion blur
- Small violations of the mathematical models used for invariance

---

### 10.2 Moravec Corner Detection (SSD Method)

**Introduced by Hans Moravec** as an early, intuitive method for corner detection.

**Core Idea:** A *corner* is a point where the image looks different in *all* directions when you move a small observation window. Contrast this with:
- **Flat region:** Image looks the same in all directions (no change)
- **Edge:** Image changes significantly in one direction (perpendicular to edge), looks similar in the other (parallel to edge)
- **Corner:** Image changes significantly in *every* direction → maximum signal of interest

**Algorithm:**

**Step 1 — Define reference window:**
Choose a target pixel. Place a 3×3 (or larger) neighborhood window around it. This is the **reference matrix**.

**Step 2 — Shift in 8 directions:**
Shift the reference window by 1 pixel in each of 8 directions:
- Cardinal: Up, Down, Left, Right
- Diagonal: Up-Left, Up-Right, Down-Left, Down-Right

**Step 3 — Compute SSD (Sum of Squared Differences):**
For each shift direction, compute how different the shifted window is from the original reference:

> **SSD = Σ (Reference_pixel - Shifted_pixel)²**

Each SSD value measures how much the image content changes in that direction.

**Step 4 — Select minimum SSD:**
The smallest SSD among all 8 shifts represents the direction of *least change* (most similar neighborhood).

**Step 5 — Threshold:**
- If **minimum SSD > threshold** → the pixel is a **corner** (even in the most similar direction, the image changes significantly → it changes in all directions)
- If **minimum SSD ≤ threshold** → the pixel is **not a corner** (at least one direction shows similarity → it's a flat region or edge)

**Example from slides:**

Reference matrix (3×3 around target pixel 14):
```
[ 6   6   6 ]
[15  14   0 ]
[15   0   0 ]
```

SSD values for 8 shifts:
- SSD1 (up): 395
- SSD2 (down): 451
- SSD3 (left): 422
- SSD4 (right): 576
- SSD5 (diagonal up-left): 227 ← **Minimum**
- SSD6 (diagonal up-right): 764
- SSD7 (diagonal down-left): 493
- SSD8 (diagonal down-right): 702

Minimum SSD = 227. With threshold = 220:
> 227 > 220 → **This pixel IS a corner** ✓

**Limitations of Moravec:**
- Not invariant to rotation — only shifts in 4 or 8 directions, misses corners between those angles
- Very sensitive to noise — SSD is sensitive to small intensity fluctuations
- Binary decision (corner or not), no continuous score

---

### 10.3 Harris Corner Detection

**Introduced by Chris Harris and Mike Stephens (1988)** as an improvement over Moravec.

**Key improvement:** Instead of comparing finite shifts (Moravec), Harris uses *partial derivatives* (infinitesimal changes) of image intensity — smoother, more principled, and rotation-invariant.

**Mathematical Foundation:**

Consider a small window W around a pixel (x, y). For a shift (u, v), the change in image content is:

> **E(u,v) = Σ_{(x,y)∈W} [I(x+u, y+v) - I(x,y)]²**

Using a first-order Taylor expansion (for small shifts):

> **E(u,v) ≈ [u, v] × M × [u, v]ᵀ**

Where **M** is the **second moment matrix** (also called the structure tensor):

```
M = Σ_{(x,y)∈W} w(x,y) × [Ix²    IxIy]
                            [IxIy   Iy² ]
```

Where:
- Ix = ∂I/∂x = image gradient in x direction (using kernel [-1, 0, 1])
- Iy = ∂I/∂y = image gradient in y direction (using kernel [-1, 0, 1]ᵀ)
- w(x,y) = window weighting function (often a Gaussian for smooth spatial weighting)
- Σ(Ix²), Σ(Iy²), Σ(IxIy) are sums over the window W

**Computing derivatives (Harris step-by-step):**

**1. Compute X-derivative (Ix):**
Apply horizontal kernel [-1, 0, 1] to each pixel:
> Ix(pixel) = (-1 × left_pixel) + (0 × center) + (1 × right_pixel)

Example: For center pixel = 0, left = 1, right = 5:
> Ix = (-1×1) + (0×0) + (1×5) = 4

**2. Compute Y-derivative (Iy):**
Apply vertical kernel [-1; 0; 1] to each pixel:
> Iy(pixel) = (-1 × top_pixel) + (0 × center) + (1 × bottom_pixel)

Example: For center pixel = 0, top = 0, bottom = 4:
> Iy = (-1×0) + (0×0) + (1×4) = 4

**3. Compute Harris Matrix components:**
> Hxx = Σ(Ix²), Hyy = Σ(Iy²), Hxy = Σ(IxIy)

**4. Build the 2×2 Harris Matrix H:**
```
H = [Hxx  Hxy]
    [Hxy  Hyy]
```

**5. Eigenvalue analysis:**
The eigenvalues λ₁ and λ₂ of M characterize the local image structure:

| λ₁ small, λ₂ small | Both eigenvalues small → flat region (no significant gradient in any direction) |
|---|---|
| λ₁ large, λ₂ small | One large eigenvalue → edge (strong gradient in one direction only) |
| λ₁ large, λ₂ large | Both eigenvalues large → **corner** (strong gradients in multiple directions) |

Geometrically: The second moment matrix defines an ellipse. The axis lengths are determined by eigenvalues; orientation by eigenvectors. A circle (λ₁ ≈ λ₂) = corner; a thin elongated ellipse (λ₁ >> λ₂) = edge.

**6. Harris Corner Response function (avoiding expensive eigenvalue computation):**

Computing eigenvalues is computationally expensive (requires solving a quadratic equation for each pixel). Harris proposed using the determinant and trace instead:

> **C = det(H) - k × (trace(H))²**

Where:
- **det(H) = Hxx × Hyy - Hxy × Hxy = λ₁ × λ₂**
- **trace(H) = Hxx + Hyy = λ₁ + λ₂**
- **k** is an empirical constant, typically **k = 0.04 to 0.15** (must be determined experimentally)

**Interpreting the Harris Corner Score C:**

| C value | Interpretation |
|---|---|
| **C is a large positive value** | **CORNER** — both eigenvalues are large → both det (product) and trace (sum) are large, but det dominates |
| **C is negative** | **EDGE** — one eigenvalue is much larger than the other → det is small (product of unequal values), trace is large → negative result |
| **|C| is small** | **FLAT REGION** — both eigenvalues are small → det ≈ 0, trace ≈ 0 → C ≈ 0 |

*Analogy:* Think of λ₁ and λ₂ as the "edge strength" in two perpendicular directions. For a corner, you have strong edges in both directions. For an edge, you have a strong edge in one direction only. For flat ground, no strong edges in either direction.

**7. Non-maximum suppression:**
Extract *local maxima* of C — pixels where C is highest in their neighborhood. Retain only those maxima above a given threshold.

**Applications of Harris Corner Detection:**
- Feature extraction for object recognition
- Keypoint detection for tracking systems
- Structure from Motion (SfM) — building 3D models from multiple 2D images
- SLAM — detected corners serve as stable landmarks for localization

---

### 10.4 Invariance Properties of Harris

**Invariant to:**
- **2D image rotation:** The eigenvalues of M remain unchanged under rotation (the ellipse rotates but its shape doesn't change) → the same corner is detected at the same location regardless of camera rotation ✓
- **Affine intensity changes (I' = a×I + b):** The position of local maxima of C is unchanged by linear brightness/contrast changes (eigenvalues are rescaled but their relative ordering is preserved) ✓

**NOT invariant to:**
- **Scale changes:** If you zoom in on a corner, it may appear as an edge at the larger scale (the window captures the corner differently). In a study, after rescaling an image by 2×, only 20% of Harris correspondences were correctly re-detected.
- **Geometric affine transformations** (shear, non-uniform scaling): The neighborhood of the feature is distorted, affecting classification.

**Despite these limitations,** Harris corner detector remains extremely widely used — it's in OpenCV, used in nearly all classic computer vision pipelines, and is the foundation for more advanced detectors.

---

### 10.5 Scale-Invariant Detection

**Problem:** The Harris detector misses corners under scale changes. A corner detected at one zoom level may be classified as an edge at a different zoom level.

**Solution 1 — Multiscale Harris:**
Apply the Harris detector at *multiple scales* simultaneously (using different window sizes). If a corner is detected at any scale, it's kept.

**Solution 2 — Scale-Space Pyramid:**
Rather than varying the window size, generate multiple *downsampled or upsampled versions* of the original image (a Gaussian pyramid). Apply the detector at each pyramid level.

> Each pyramid level corresponds to a different effective scale. At some level of the pyramid, the corner will be detected at the scale where its structure is most distinct.

**Harris-Laplacian Detector:**
A principled scale-invariant extension of Harris that:
1. Detects corners with the Harris detector at multiple scales
2. Uses the **Laplacian of Gaussian (LoG)** operator to automatically select the *characteristic scale* of each detected corner — the scale at which the LoG response is maximum

This gives each detected corner a *position + scale*, making matching across zoom levels possible.

---

### 10.6 Affine-Invariant Detection (Harris-Affine)

**Problem:** Affine transformations (rotation + non-uniform scaling + shear) — a good approximation of perspective distortion for locally planar patches — break the standard Harris detector.

**Harris-Affine Detector (Mikolajczyk & Schmid):**

**Procedure:**
1. Detect features using the scale-invariant **Harris-Laplacian** detector (gives position + scale)
2. For each detected feature, use the **second moment matrix M** to find the two directions of *slowest and fastest intensity change* around the feature (the principal axes of the ellipse defined by M)
3. From these two directions, compute an **ellipse** of the same size as the characteristic scale
4. **Normalize the elliptical region to a circular region** — this normalization removes the affine distortion
5. Detect and describe features in the normalized circular region

**Result:** The same feature is detected and normalized regardless of the affine transformation applied — enabling matching across different viewpoints.

---

### 10.7 Other Corner Detectors

**Shi-Tomasi Corner Detector**
- Strongly based on Harris
- Modification: Uses the criterion **min(λ₁, λ₂) > threshold** instead of Harris's det-trace formula
- More stable for feature tracking (used as "goodFeaturesToTrack" in OpenCV)
- Better suited for applications requiring the very best N corner points

**SUSAN Corner Detector (Smallest Univalue Segment Assimilating Nucleus)**
- Defines a circular mask around each pixel
- Computes how many pixels within the mask have similar intensity to the center ("SUSAN area")
- A corner has a small SUSAN area; an edge has a medium SUSAN area; flat region has large SUSAN area
- Also used for edge detection and noise suppression
- Computationally efficient

**FAST Corner Detector (Features from Accelerated Segment Test)**
- Extremely fast — designed for real-time applications on resource-constrained hardware
- Tests whether a circle of 16 pixels around a candidate point has a sufficiently large contiguous arc of pixels all brighter (or all darker) than the center by a threshold
- If yes → corner
- Much faster than Harris (no matrix computation) with comparable quality
- Used extensively in real-time robotics, mobile AR, and embedded systems
- Variants: FAST-9, FAST-12 (minimum arc length of 9 or 12 pixels)

**Summary of Corner Detectors:**

| Detector | Speed | Rotation Invariant | Scale Invariant | Key Property |
|---|---|---|---|---|
| Moravec | Slow | ✗ | ✗ | SSD-based, simple |
| Harris | Moderate | ✓ | ✗ | Eigenvalue/det-trace criterion |
| Shi-Tomasi | Moderate | ✓ | ✗ | min(λ₁,λ₂) criterion |
| Harris-Laplacian | Slow | ✓ | ✓ | Multi-scale Harris |
| Harris-Affine | Very slow | ✓ | ✓ | Affine normalization |
| SUSAN | Fast | ✓ | ✗ | Circular mask USAN area |
| FAST | Very fast | ✓ | ✗ | Segment test on circle |

---

## 11. Feature Extraction from Range Data

### Why Range-Based Features?

Laser rangefinders (LiDAR) and ultrasonic sensors provide *distance measurements* rather than visual images. Feature extraction from range data involves fitting geometric shapes (primarily lines, but also circles and corners) to the measured distance points.

**Most common geometric feature from range data:** **Line segments** — because most man-made environments (offices, warehouses, corridors, rooms) have abundant straight walls, and line features are:
- Simple to represent (2 parameters: angle and offset)
- Detectable with high reliability using LiDAR
- Useful for robot localization and map building

### Line Fitting

**Problem setup:**
- A LiDAR scan gives a set of 2D distance measurements (range + angle → Cartesian x, y points)
- We want to fit line segments to these points
- The system is *overdetermined* — more data points than line parameters → use optimization

**Least-Squares Line Fitting:**
Find the line parameters that minimize the sum of squared distances from all points to the line:

> **Minimize Σᵢ dᵢ²**

Where dᵢ is the perpendicular distance from point i to the fitted line.

This is well-understood mathematically and can be solved in closed form for simple cases.

**Real-world complication — Segmentation:**
Range measurements from a real scan may correspond to *multiple different surfaces*. The challenge is:
1. How many lines are present in the scan?
2. Which points belong to which line?
3. How do we estimate each line's parameters?

This process of partitioning points into groups before fitting is called **segmentation**.

### Six Line-Extraction Algorithms

The following algorithms address both segmentation and line fitting for 2D range scans:

#### 1. Iterative End-Point Fit (IEPF / Split-and-Merge)

**Concept:** Recursively split sets of points at their farthest outlier.

**Procedure:**
1. Start with all scan points assumed to belong to one line
2. Fit a line through the first and last point of the set
3. Find the point with the *maximum perpendicular distance* to the fitted line
4. If max distance > threshold → **split** the set at that point; recurse on each subset
5. If max distance ≤ threshold → all points in this set belong to this line → **stop**

After splitting, adjacent line segments may be **merged** if they are nearly collinear.

*Analogy:* Drawing a straight line through a crowd of people standing in a rough line. Find the person who deviates most from your straight line. If they deviate too much, split the group at them and redraw each sub-line.

**Advantage:** Simple, fast, well-suited for environments with well-defined straight walls.

#### 2. Split-and-Merge (Extended)

A formal version of IEPF. More careful about the merge step — checks that merged segments are truly collinear before combining.

#### 3. Incremental Algorithm

**Concept:** Process scan points one at a time, growing line segments incrementally.

**Procedure:**
1. Start with the first two points — initialize a candidate line
2. Add the next point
3. Re-fit the line to all points in the current segment
4. If the fit is still good (all points within distance threshold of the new line) → continue
5. If the fit degrades → the new point starts a new segment

**Advantage:** Natural for sequential processing of scan data; can be run in real-time as the scan is received.

#### 4. RANSAC (Random Sample Consensus)

**Concept:** A robust estimation method that explicitly handles *outliers* (range measurements that don't belong to any clean geometric structure — reflections, noise, glass surfaces).

**Procedure:**
1. Randomly select the *minimum number of points* to define a line (2 points)
2. Fit a line to these points
3. Count how many *other* scan points are within a distance threshold of this line (the **inliers**)
4. Repeat steps 1–3 many times
5. Keep the line hypothesis with the *most inliers*
6. Re-fit the final line using all inliers

**Advantage:** Extremely robust to outliers — even if 50%+ of measurements are garbage, RANSAC can find the correct line from the minority of clean measurements.

**Disadvantage:** Probabilistic — may need many iterations to find the true consensus; non-deterministic.

*Analogy:* In a noisy bar, you're trying to identify the loudest clear voice (the signal) amidst many different conversations (noise). You randomly sample two people, test if they seem to be having the same conversation (fit the line), count how many others are in that same conversation (inliers), and repeat until you find the group everyone agrees on.

#### 5. Hough Transform

**Concept:** A voting-based approach that detects *all* lines in an image simultaneously.

**Key insight:** Each point in (x, y) space corresponds to a *family of lines* that pass through it. In the dual space (ρ, θ) — where ρ is perpendicular distance from origin and θ is angle of the normal — each point maps to a sinusoidal curve. Lines in (x, y) correspond to *intersection points* in (ρ, θ).

**Procedure:**
1. Initialize an *accumulator array* (ρ, θ histogram)
2. For each scan point (x, y), compute all possible (ρ, θ) pairs that correspond to lines passing through it → increment the accumulator at each (ρ, θ)
3. Find *peaks* in the accumulator → these correspond to lines that many points agree on
4. Threshold peaks above a minimum vote count → detected lines

**Advantage:** Detects multiple lines simultaneously, robust to noise and missing data.

**Disadvantage:** Computationally expensive for continuous parameter spaces; accuracy limited by accumulator resolution.

#### 6. Expected Model Algorithm

A model-based approach that uses *prior knowledge of the environment* (e.g., expected wall orientations in a known building type) to guide line extraction. Lines are fitted to match expected geometric constraints.

**Most suitable when:** Operating in a well-known environment type (office buildings, warehouses) where the structure is predictable.

---

## 12. Stereo Vision and 3D Perception

### What is Stereo Vision?

**Stereopsis** is the biological mechanism by which the brain perceives depth from the slightly different images seen by the two eyes. Each eye sees the world from a slightly different position → objects appear at slightly different locations in each eye's image → the brain fuses these images using the positional difference (disparity) to perceive depth and 3D structure.

**Test:** Close one eye, hold your finger in front of you. Close that eye and open the other — your finger appears to "jump" sideways. The distance between the two apparent positions is the **disparity**. Near objects have high disparity; distant objects have low disparity.

**Computational Stereo Vision (Binocular Stereo):** Using two cameras at different positions to recover depth information from their image pair, mimicking biological stereopsis.

### Basic Stereo Setup

Consider the simplified *canonical stereo* configuration:
- Two cameras with *parallel optical axes*
- Separated by a horizontal distance **b** called the **baseline**
- Same focal length **f**, same orientation

For a scene point P at distance Z from the cameras:
- Point P appears at pixel position **xₗ** in the left image
- Same point appears at position **xᵣ** in the right image

**Disparity d = xₗ - xᵣ** (the horizontal offset between the two image positions)

**Depth equation:**

> **Z = (f × b) / d**

Where:
- Z = depth (distance to the scene point)
- f = focal length of the cameras
- b = baseline (separation between cameras)
- d = disparity (difference in pixel positions between left and right images)

### Key Observations from the Depth Equation

**1. Inverse relationship between depth and disparity:**
> Z ∝ 1/d

- **Nearby objects** have **large disparity** (they shift a lot between the two camera views) → depth can be measured *accurately* for close objects
- **Distant objects** have **small disparity** (they barely shift) → depth estimates for distant objects are increasingly imprecise
- This is appropriate for robotics — nearby obstacles matter most for collision avoidance

*Analogy:* Move a finger close to your face and alternate closing each eye — the finger jumps far (large disparity, easy to judge close distance). Hold the finger at arm's length — smaller jump (smaller disparity, harder to judge distance). Look at a mountain — almost no apparent shift between eyes (tiny disparity, very hard to judge exact distance).

**2. Disparity proportional to baseline:**
> d ∝ b

For the same disparity error, accuracy of depth estimate *increases with larger baseline* b.

- Larger baseline → larger disparity for the same scene point → easier to measure accurately

**Trade-off:** As baseline increases, some objects appear in *one camera but not the other* (field of view mismatch) — these objects cannot be ranged (no corresponding point in both images).

**3. Unknown baseline (Structure from Motion):**
If the baseline b is unknown (e.g., when a single moving camera takes two frames at different positions without knowing the exact displacement), the scene can only be reconstructed *up to scale* — you can determine the shape but not the absolute size or distance.

### The Correspondence Problem

**Core Challenge:** The depth equation requires knowing which pixel in the left image corresponds to the same 3D scene point as which pixel in the right image. Finding these **conjugate pairs** is the **correspondence problem**.

> **Why it's hard:**
> - Two views see the same 3D point with different lighting (photometric distortion)
> - Perspective distortion — the same surface appears differently shaped from different angles
> - Occlusions — some parts visible in one view are hidden in the other

**False correspondences:** A pixel in the left image is mistakenly matched to a wrong pixel in the right image → incorrect depth estimate.

### Constraints for Solving the Correspondence Problem

**Epipolar Constraint (Most Important):**
Given a point in the left image, its corresponding point in the right image must lie on a specific line called the **epipolar line**. This reduces the 2D search problem to a 1D search along this line → greatly reduces computation and false matches.

*Derivation:* The two camera centers and the 3D point define a plane (the *epipolar plane*). This plane intersects both image planes as lines (the epipolar lines). The correspondence of a point in one image must lie on the corresponding epipolar line in the other image.

**Similarity Constraint:** Corresponding points should have similar local appearance (color, texture, intensity pattern). Match points by maximizing appearance similarity.

**Continuity Constraint:** Depth varies smoothly (nearby pixels in 3D space usually have similar depth) → matched disparities should be smooth (no abrupt jumps, except at depth boundaries like object edges).

**Unicity Constraint:** Each point in one image can correspond to at most one point in the other image (one-to-one matching).

**Monotonic Order Constraint:** If point A is to the left of point B in the left image, A's correspondence in the right image should be to the left of B's correspondence — preserving the order of points along the epipolar line (except at occlusions).

### Matching Methods

**Area-based methods:** Match corresponding points by comparing small image patches (windows of pixels). Use metrics like Sum of Squared Differences (SSD), Normalized Cross-Correlation (NCC), or Sum of Absolute Differences (SAD).

**Feature-based methods:** Detect distinctive features (Harris corners, SIFT keypoints) in both images, then match features by their descriptors. More robust to lighting changes than area-based, but sparser (only at feature locations).

---

## Quick Reference Summary

| Topic | Key Term | One-Line Definition |
|---|---|---|
| Perception | Sensing + Interpreting | Converting raw sensor data into meaningful understanding of environment |
| Proprioceptive | Internal sensing | Measures the robot's own state (joints, speed, battery) |
| Exteroceptive | External sensing | Measures the environment (distance, light, objects) |
| Active sensor | Emits + receives | Emits energy, measures environment reaction (LiDAR, sonar) |
| Passive sensor | Receives only | Measures ambient energy (camera, microphone, thermometer) |
| Resolution | Smallest detectable change | Minimum input difference that produces a detectable output change |
| Dynamic range | Max/min ratio | Ratio of largest to smallest measurable value, in dB |
| Sensitivity | Output/Input ratio | How strongly output responds to input changes |
| Cross-sensitivity | Unwanted sensitivity | Sensitivity to parameters you don't want to measure |
| Systematic error | Predictable error | Repeatable, modelable, correctable by calibration |
| Random error | Unpredictable error | Stochastic, described by probability distributions |
| Precision | Repeatability | Consistency of repeated measurements (small σ) |
| Accuracy | Correctness | Closeness to true value (small bias μ) |
| Optical encoder | Shaft rotation sensor | Counts light interruptions from rotating slotted disk |
| Odometry | Wheel-based positioning | Estimating position by integrating wheel encoder data |
| Dead reckoning | Position from motion history | Estimating current position from previous position + motion integration |
| Gyroscope | Angular velocity sensor | Measures rotation rate; maintains orientation reference |
| Mechanical gyroscope | Spinning rotor | Uses angular momentum of spinning mass to resist rotation |
| Optical gyroscope | Sagnac effect laser | Uses frequency difference of CW/CCW laser beams |
| Accelerometer | Force/acceleration sensor | Measures linear acceleration including gravity via proof mass |
| IMU | 6-DOF motion sensor | Combines gyros + accelerometers for full pose estimation |
| IMU drift | Accumulated error | Quadratic error growth from double integration of accelerometer noise |
| GPS | Satellite positioning | Trilateration from ≥4 satellite signals → 3D position |
| DGPS | Corrected GPS | Uses reference receiver at known position to improve GPS accuracy |
| Triangulation ranging | Geometry-based distance | Uses angle of reflection to compute distance |
| ToF ranging | Time-based distance | Measures signal round-trip time → distance = (c×t)/2 |
| Ultrasonic sensor | Sound ToF | Emits sound pulses, measures echo time; d = c×t/2 |
| LiDAR | Laser ToF + scanning | Laser pulses scanned in 2D/3D; generates point clouds |
| Structured light | Pattern projection depth | Projects known pattern; camera measures distortion → depth |
| RADAR | Radio ToF + Doppler | Radio waves; works in adverse weather; measures distance and speed |
| CCD | Charge-coupled sensor | Serial charge transfer readout; high quality, high power |
| CMOS | MOS transistor sensor | Parallel pixel readout; low power, cheap, dominant today |
| Bayer filter | Color filter array | Single-chip color via R/G/B pixel-level filters (2G:1R:1B) |
| Demosaicing | Color interpolation | Estimating missing color values between Bayer filter pixels |
| Feature extraction | High-level perception | Deriving recognizable structures (corners, lines) from raw sensor data |
| Low-level feature | Geometric primitive | Lines, corners, blobs, circles derived from raw data |
| High-level feature | Object-level | Doors, tables, cars — recognized from combinations of primitives |
| Repeatability | Feature consistency | Ability to re-detect same features under varying conditions |
| Invariance | Transformation robustness | Feature detected consistently despite rotation, scale, lighting changes |
| SSD | Similarity metric | Sum of Squared Differences; measures patch difference |
| Moravec detector | SSD-based corners | Finds corners by maximum-in-all-directions SSD response |
| Harris detector | Gradient-based corners | Uses second moment matrix; det(M) - k×trace²(M) > threshold → corner |
| Harris matrix M | Gradient structure tensor | 2×2 matrix of squared/cross image gradients over a window |
| Cornerness function C | Harris score | C = det(H) - k×trace²(H); large positive → corner, negative → edge, ≈0 → flat |
| Eigenvalues λ₁,λ₂ | Corner classification | Both large → corner; one large → edge; both small → flat |
| Scale-space pyramid | Multi-scale detection | Image at multiple scales; detects features scale-invariantly |
| Harris-Laplacian | Scale-invariant Harris | Harris + LoG for automatic scale selection |
| Harris-Affine | Affine-invariant Harris | Normalizes elliptical regions to circles for affine invariance |
| FAST detector | Fast corner detection | Segment test on 16-pixel circle; very fast for real-time robotics |
| Line extraction | Range data feature | Fitting line segments to LiDAR/sonar point clouds |
| RANSAC | Robust estimator | Random sampling to find consensus model despite outliers |
| Split-and-merge | Recursive line fitting | Recursively splits point sets at maximum-distance outliers |
| Stereo vision | Binocular depth | Two cameras → disparity between views → depth Z = f×b/d |
| Disparity d | Stereo depth cue | Pixel offset of same 3D point between left and right images |
| Baseline b | Camera separation | Distance between two stereo camera centers |
| Correspondence problem | Stereo matching | Finding which pixel in left image matches which pixel in right image |
| Epipolar constraint | 1D search constraint | Corresponding point must lie on epipolar line → reduces 2D to 1D search |

---

*Notes compiled from PES University MAR Unit 2 slides (UE23CS343BB7) — Course by Dr. Ashok Kumar Patil.*
*All slide content covered; supplemented with full explanations, mathematical derivations, analogies, and extended context.*
