# Unit 3 – Localization
### Mobile and Autonomous Robots | UE23CS343BB7
### PES University | Dr. Ashok Kumar Patil

---

## Table of Contents
1. [Introduction to Localization](#1-introduction-to-localization)
2. [Challenges of Localization](#2-challenges-of-localization)
   - [Sensor Noise](#21-sensor-noise)
   - [Sensor Aliasing](#22-sensor-aliasing)
   - [Effector Noise](#23-effector-noise)
3. [Localization-Based Navigation vs Programmed Solutions](#3-localization-based-navigation-vs-programmed-solutions)
4. [Belief Representations](#4-belief-representations)
   - [Single-Hypothesis Belief](#41-single-hypothesis-belief)
   - [Multiple-Hypothesis Belief](#42-multiple-hypothesis-belief)
5. [The Robot Localization Problem](#5-the-robot-localization-problem)
   - [Prediction Phase](#51-prediction-phase)
   - [Perception Phase](#52-perception-phase)
   - [Markov vs Kalman Filter](#53-markov-localization-vs-kalman-filter-localization)
6. [Classification of Localization Problems](#6-classification-of-localization-problems)
7. [Probabilistic Map-Based Localization](#7-probabilistic-map-based-localization)
   - [Basic Probability Concepts](#71-basic-concepts-in-probability)
8. [Markov Localization](#8-markov-localization)
9. [Kalman Filter Localization](#9-kalman-filter-localization)
10. [SLAM – Simultaneous Localization and Mapping](#10-slam--simultaneous-localization-and-mapping)
    - [EKF-SLAM](#101-ekf-slam)
    - [Particle Filter SLAM](#102-particle-filter-slam)
    - [Open Challenges in SLAM](#103-open-challenges-in-slam)

---

## 1. Introduction to Localization

**Localization** is the ability of a robot to determine its own **position and orientation** within an environment so it can navigate and perform tasks effectively.

Localization methods involve:
- **Sensors** – Cameras, LIDAR, odometry
- **Algorithms** – SLAM (Simultaneous Localization and Mapping), which creates maps of surroundings while simultaneously locating the robot within that map

### General Localization Loop (Figure 5.2 concept)

```
Encoder → Prediction of Position (e.g., odometry)
              ↓
         Predicted Position ──→ Matching ←── Raw Sensor Data / Extracted Features
                                    ↓ (YES – match found)
                              Matched Observations
                                    ↓
                            Position Update (Estimation)
                                    ↓
                               New Position ──→ (loop back)
```

The robot uses encoder data to predict position, then compares sensor observations against a map database. If observations match expected features, the position is updated.

---

## 2. Challenges of Localization

Three main sources of noise/inaccuracy challenge localization:

1. **Sensor Noise**
2. **Sensor Aliasing**
3. **Effector Noise**

> **Important insight**: Localization means more than simply determining an absolute pose in space. It means **building a map**, then **identifying the robot's position relative to that map**.

The three domains — **Localization**, **Mapping**, and **Motion Control** — overlap and interact (visualized as a Venn diagram), and SLAM sits at the intersection of all three.

---

### 2.1 Sensor Noise

**Definition**: Random variations in sensor readings even when the environment remains unchanged.

**Impact**:
- Reduces accuracy of readings
- Reduces information reliability
- Makes perception difficult

**Sources**:
- Environmental changes (e.g., lighting)
- Sensor hardware limitations
- Electronic interference

#### Example: CCD Camera for Indoor Navigation
- The robot uses color information to detect objects
- **Problem**: Lighting changes (sunlight through clouds, indoor lamp flicker) alter color values
- **Result**: Camera output appears noisy even though the environment didn't physically change

**Common causes of noise in vision sensors**:
| Cause | Effect |
|-------|--------|
| Picture jitter | Unstable frames |
| Signal gain variations | Inconsistent brightness |
| Blooming | Overflow of bright pixels |
| Blurring | Loss of fine features |

These effects reduce image clarity and feature detection reliability → **Lower information content from visual sensors**.

#### Handling Sensor Noise

Two primary solutions:

1. **Temporal Fusion** – Combine readings over time to average out noise
2. **Multi-Sensor Fusion** – Combine data from multiple sensors
   - Camera + LiDAR
   - Sonar + IMU
   - GPS + Vision
   
**Result**: Higher reliability and better localization

---

### 2.2 Sensor Aliasing

**Definition**: Occurs when **different environments produce identical sensor readings**, creating ambiguity in localization.

In robotics: Many distinct environmental states → Same sensor output → Robot cannot distinguish locations.

#### Human Analogy
Humans rarely experience aliasing due to powerful vision systems, but consider:
- **Walking in complete darkness** – everything looks the same → hard to localize
- **Maze of tall hedges** – many locations look identical → position is ambiguous

#### Robot Example: Range Sensors
A robot using range sensors (e.g., sonar/LiDAR) only measures **distance to obstacle**. It cannot detect:
- Color
- Texture
- Material

So the robot might detect an obstacle but cannot tell if it is a **human** or a **cardboard box** — both produce **identical sensor readings**.

#### Solving Sensor Aliasing

To overcome aliasing, robots must use:
1. Multiple sensor readings over time
2. Motion-based observations (move slightly and re-observe)
3. Environmental mapping

Example techniques:
- **SLAM** – Simultaneous Localization and Mapping
- **Particle Filters** – Multiple hypotheses sampled
- **Kalman Filters** – Probabilistic position tracking

> Localization accuracy **improves as more data is collected**.

---

### 2.3 Effector Noise

**Definition**: Errors arising from imperfect robot motion — the robot's actual motion differs from the intended motion.

**Causes**:
- Wheel slip
- Uneven terrain
- External disturbances (bumps, pushes)

#### Odometry and Dead Reckoning

Robot position estimation commonly uses:
- **Odometry** – Uses wheel encoder sensors to estimate displacement
- **Dead Reckoning** – Combines wheel encoders + heading sensors (gyro, compass) to track position from a known starting point

**Problem**: Errors **accumulate over time** → Periodic correction with exteroceptive sensors is necessary.

#### Sources of Odometry Errors

| # | Source |
|---|--------|
| 1 | Limited sensor resolution |
| 2 | Wheel misalignment |
| 3 | Unequal wheel diameters |
| 4 | Changing wheel contact points |
| 5 | Floor irregularities |
| 6 | Wheel slipping |

Errors are classified as:
- **Deterministic (Systematic)** – Predictable, can be calibrated
- **Random (Nondeterministic)** – Unpredictable, harder to correct

#### Types of Odometry Errors

1. **Range Error** – Error in total distance traveled
2. **Turn Error** – Error in the degree of robot rotation
3. **Drift Error** – Orientation error caused by differences in wheel sizes/characteristics

Over time, **turn and drift errors dominate** localization error. Even a small angular error produces **large position errors** after long movement (the error grows non-linearly with distance).

> **Conclusion**: Localization must be continuously corrected using external (exteroceptive) sensors.

---

## 3. Localization-Based Navigation vs Programmed Solutions

### Option 1: Behavior-Based (Programmed) Navigation

A navigation system requires sensors and motion control for obstacle avoidance. Importantly, **explicit localization with a map may not be necessary** for successful navigation.

**Behavior-based approaches** use sets of reactive behaviors that directly map sensor input to actuator output — no explicit localization or path planning.

**Example**: A robot navigating from Room A to Room B using:
- Left-wall following behavior
- A Room B detector triggered by a unique cue (e.g., a distinctive marker)

This architecture (Figure 5.7) routes sensor inputs through behaviors like:
- Communicate data
- Discover new area
- Detect goal position
- Avoid obstacles
- Follow right/left wall

All behaviors are fused (e.g., via vector summation) → actuators

**Pros**:
- Quick to implement for specific environments

**Cons**:
- Lacks scalability (environment-specific)
- Requires careful design of underlying procedures
- Can suffer from **instability** when multiple behaviors are active simultaneously

---

### Option 2: Map-Based (Localization-Based) Navigation

**Architecture** (Figure 5.8):
```
Sensors → Perception → Localization/Map-Building → Cognition/Planning → Motion Control → Actuators
```

In **map-based navigation**, the robot explicitly localizes by collecting sensor data and updating its belief about position with respect to a map.

**Key Advantages**:
1. The explicit map-based position concept makes the system's belief **transparently available** to human operators
2. The map is a **communication medium** between human and robot — just give the robot a new map for a new environment
3. A robot-built map can also be **used by humans** (dual use)

**Disadvantages**:
1. Requires **more upfront development effort**
2. **Key risk**: Reliance on an internal representation that may **diverge from reality**, leading to undesirable robot behavior

---

## 4. Belief Representations

### Analogy
Finding your apartment in a large city requires:
1. A **map** of the city (environment representation)
2. Knowing your **current address** (belief about position)

For a robot:
- **Map** = representation of the environment
- **Belief** = estimate of the robot's current position

The core challenge in map-based localization is **representation** along two axes:
- **Map Design** – Content and fidelity of the environment model
- **Belief Design** – Whether position is a single unique point or a set of possibilities

These design decisions affect architectural complexity, computational complexity, and localization accuracy.

---

### 4.1 Single-Hypothesis Belief

**Definition**: The robot's belief about position is expressed as a **single unique point** on the map.

**Variants** (shown in four sub-figures):
- **(a)** Actual environment layout
- **(b)** Single point `(x, y, θ)` on a continuous 2D geometric map
- **(c)** Single cell on a discrete, tessellated grid map
- **(d)** Single node `i` on a topological graph

**Advantages**:
- Eliminates positional ambiguity
- Simplifies cognitive-level decision-making (e.g., path planning)
- Easy to update — single position → new single position

**Disadvantage**:
- Motion-induced uncertainty makes forcing a single-hypothesis update challenging
- Does not gracefully handle sensor ambiguity

---

### 4.2 Multiple-Hypothesis Belief

**Definition**: The system tracks **several possible positions simultaneously**, representing the robot's degree of positional uncertainty.

**Belief state**: A set of possible coordinates on a 2D or 3D map (or a probability distribution over poses).

**Key representations**:
- **Geometric polygons** — regions of possible positions
- **Gaussian distributions** — probabilistic spread around a mean position

The system explicitly manages uncertainty from noisy or partial sensor data. As the robot moves and collects more data, the **spread of belief narrows**.

**The four cases of belief representation (1D example)**:
- **(a)** Continuous map, single Gaussian — single-hypothesis
- **(b)** Continuous map, multiple Gaussians — multiple-hypothesis
- **(c)** Discretized grid map with probability over all cells — Markov approach
- **(d)** Discretized topological map with probability over nodes — Markov approach

> In the multiple-hypothesis figure: The robot starts with a spread-out belief (position 2), and as it moves and observes (positions 3 and 4), the belief concentrates. Darker coloring = higher probability.

**Drawbacks**:
- Decision-making is complicated (which hypothesis to act on?)
- Full implementation leads to **high computational complexity** and state space explosion

**Hybrid approaches** track clusters of high probability, combining geometric maps with simplified Metric-Topological approaches.

---

## 5. The Robot Localization Problem

Mobile robot navigation relies on the **constant management of position uncertainty**.

When a robot starts from a known location and moves using odometry:
- Odometry is subject to cumulative errors (wheel slip, mechanical tolerances)
- Uncertainty **grows unbounded** over time

To prevent this, the robot must periodically **localize relative to a map** using **exteroceptive sensors** (ultrasonic, laser, camera).

The position update process is split into two logical steps:

---

### 5.1 Prediction Phase

**Also called**: Action Update

The robot uses its **proprioceptive sensors** (wheel encoders) to estimate how it has moved.

- Uncertainty about robot configuration **increases** in this phase
- Encoder errors accumulate → motion is nondeterministic

**Graphically**: Starting from a known position `x₀` (Dirac delta function — perfect certainty), as the robot moves to `x₁` and `x₂`, the probability density function (PDF) **broadens**.

---

### 5.2 Perception Phase

**Also called**: Measurement Update / Correction Update

The robot uses **exteroceptive sensors** (rangefinder, camera) to correct the position estimated in the prediction phase.

- Compares sensor readings to the known map
- **Uncertainty shrinks** — the PDF narrows

**Graphically**: After measuring distance `d` from a wall, the robot computes position `x'₂` which conflicts with `x₂` from odometry. The perception update reconciles them, correcting to `x₂''` with reduced uncertainty.

---

### 5.3 Markov Localization vs Kalman Filter Localization

| Property | Markov Localization | Kalman Filter Localization |
|---|---|---|
| Belief representation | Any arbitrary PDF | Single Gaussian (μ, σ) |
| Initial position | Can start from unknown position | Needs known initial position |
| Handles multiple hypotheses | Yes | No (unimodal only) |
| Computational cost | High (updates all states) | Low (updates just μ and σ) |
| Map type | Discrete (grid / topological) | Continuous |
| Recovery from large error | Yes | May fail — becomes "irrecoverably lost" |
| Precision | Limited by grid resolution | High (continuous) |

---

## 6. Classification of Localization Problems

### 6.1 Local Localization (Position Tracking)
- Robot **knows its initial position approximately**
- Task: Continuously track pose as it moves
- Small errors accumulate over time (drift)
- Example: Robot starts at a known map point, uses odometry + LiDAR to update position
- Typically handled with **Kalman Filter** (unimodal, normal distribution)

### 6.2 Global Localization
- Robot's initial location is **completely unknown**
- Robot must determine its pose considering all possible map locations
- Initial belief = **uniform distribution** (all positions equally likely)
- Example: Warehouse robot placed anywhere without knowing its start position

### 6.3 Kidnapped Robot Problem
- Similar to global localization, but harder: the robot **doesn't know it has been moved**
- Robot is physically relocated while it still believes it knows its position
- Must detect the inconsistency between expected and observed sensor readings
- Recovering from kidnapping is crucial for autonomous and commercial robots

### 6.4 SLAM (Simultaneous Localization and Mapping)
- Robot has **no prior map** and **no known position**
- Must build the map and localize itself simultaneously
- Variants:
  - **Online SLAM** – real-time processing
  - **Offline SLAM** – post-processing after data collection

### 6.5 Relative vs Absolute Localization

| Type | Basis | Method | Issue |
|------|-------|--------|-------|
| **Relative** | Internal sensors (wheel encoders, IMU) | Dead reckoning | Cumulative drift error |
| **Absolute** | External references (GPS, landmarks) | Direct measurement | Environment-dependent |

### 6.6 Metric vs Topological Localization

| Type | Description |
|------|-------------|
| **Metric** | Uses precise coordinates (x, y, θ); requires detailed geometric maps |
| **Topological** | Represents environment as nodes and connections; focuses on "which place" not "exact coordinates" |

---

## 7. Probabilistic Map-Based Localization

### Core Philosophy
Probabilistic robotics acknowledges that **uncertainty is unavoidable** in robot perception and action. Instead of relying on a single "best guess," probabilistic algorithms represent information as **probability distributions over a whole space of possible hypotheses**.

This allows robots to:
- Represent ambiguity mathematically
- Model degrees of belief
- Accommodate all sources of uncertainty (sensor noise, aliasing, effector noise)

### Why It's Needed
- Probabilistic methods make robots robust to sensor noise, imperfect data, and dynamic environments
- They enable solutions to the kidnapped robot problem and mapping in GPS-denied environments
- **Caveat**: These methods are computationally intensive and often require approximations because exact probability distributions in continuous environments are infeasible

---

### 7.1 Basic Concepts in Probability

#### Random Variables
In probabilistic robotics, sensor measurements, controls, and robot/environment states are all modeled as **random variables** — they take multiple values according to specific probabilistic laws.

#### Probability Density Functions (PDFs)
In continuous spaces, random variables take on a continuum of values → PDFs.

Both discrete and continuous probabilities must sum/integrate to 1:
- Discrete: `Σ p(x) = 1`
- Continuous: `∫ p(x) dx = 1`

#### Gaussian (Normal) Distribution

$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$$

Abbreviated as: `p(x) = N(μ, σ²)`

For a **k-dimensional vector** x (multivariate):

$$p(x) = \frac{1}{(2\pi)^{k/2} \det(\Sigma)^{1/2}} \exp\left(-\frac{1}{2}(x-\mu)^T \Sigma^{-1} (x-\mu)\right)$$

where:
- `μ` = mean vector
- `Σ` = covariance matrix (positive semidefinite, symmetric)

#### Conditional Probability

The probability that X = x given that Y = y (for p(y) > 0):

$$p(x|y) = \frac{p(x,y)}{p(y)}$$

#### Bayes' Rule

$$p(x|y) = \frac{p(y|x) \cdot p(x)}{p(y)}$$

| Term | Name | Meaning |
|------|------|---------|
| `p(x\|y)` | **Posterior** | Probability of x *after* observing y (what we want) |
| `p(y\|x)` | **Likelihood** | How likely is observation y if x is true? |
| `p(x)` | **Prior** | Probability of x before observing anything |
| `p(y)` | **Evidence** | Total probability of observing y |

Evidence: `p(y) = Σ p(y|xᵢ) · p(xᵢ)` (sum over all hypotheses)

Both Markov and Kalman Filter localization algorithms use Bayes' Rule during the **measurement (Perception) update**.

#### Bayes Rule Example: Robot Locating Itself

**Scenario**: Robot has a map and sees a RED WALL. Where is it?

Locations: Kitchen (K), Hallway (H), Office (O)

**Prior** (uniform): P(K) = P(H) = P(O) = 0.33

**Likelihood** (sensor model):
- P(Red Wall | Kitchen) = 0.95
- P(Red Wall | Hallway) = 0.10
- P(Red Wall | Office) = 0.05

**Evidence**:
```
P(Red) = (0.95)(0.33) + (0.10)(0.33) + (0.05)(0.33)
       = 0.3135 + 0.033 + 0.0165 = 0.363
```

**Posterior** for Kitchen:
```
P(Kitchen | Red Wall) = (0.95 × 0.33) / 0.363 = 0.3135 / 0.363 = 0.863
```

→ Robot is **86.3% sure** it is in the kitchen.

#### Belief Distributions

A robot cannot directly measure its true pose (even GPS has error). The best estimate of robot state based on sensor data is called the **belief**:

$$bel(x_t) = \sum p(x_t | z_{1 \to t}, u_{1 \to t})$$

where:
- `x_t` = robot state at time t
- `z_{1→t}` = all past observations (exteroceptive sensor readings)
- `u_{1→t}` = all past control inputs (odometry / wheel encoders)

---

## 8. Markov Localization

### Overview
Markov localization tracks the robot's belief state using an **arbitrary probability density function**. In practice, it tessellates the robot configuration space `(x, y, θ)` into a **finite, discrete number** of possible robot poses.

The number of possible poses ranges from several hundred to **millions** of positions and orientations.

Markov localization addresses **all three** localization problems:
- Global localization
- Position tracking
- Kidnapped robot problem

### Algorithm

```
for all x_t do:
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}) · bel(x_{t-1}) dx_{t-1}    ← prediction update
    bel(x_t) = η · p(z_t | x_t, M) · bel̄(x_t)                       ← measurement update
endfor
return bel(x_t)
```

- `p(x_t | u_t, x_{t-1})` = motion model (probability of being at x_t given action u_t from x_{t-1})
- `p(z_t | x_t, M)` = sensor model (probability of observation z_t given pose x_t and map M)
- `η` = normalization constant

### Three Key Sensor Model Assumptions

1. **Gaussian Error Distribution**
   - Range sensor error follows a Gaussian distribution centered at the correct reading
   - Captures variability and noise statistically

2. **Nonzero Probability Across All Readings**
   - All possible sensor readings (even outliers) have nonzero probability
   - Peak is at the correct reading; tails extend across full range

3. **Specific Failure Mode for Range Sensors**
   - Sensors sometimes report maximum range when signal is absorbed or coherently reflected
   - The PDF has a small probability **spike at maximum range** to account for this failure

### Concrete Example: LIDAR Sensor Model

**Setup**: LIDAR with max range = 50m, actual object distance = 10m

**Assumption 1 (Gaussian Error)**:
- Normal readings: `N(10, 0.5²)` — most readings cluster around 10m ± 0.5m

**Assumption 2 (Nonzero probability)**:
- All readings from 0m–50m have nonzero probability

**Assumption 3 (Failure mode)**:
- `P(reading = 50m | actual = 10m) = 0.05` — 5% chance of coherent reflection failure

**Robot interpretation**:
- Reads 10m → "95% sure object is at 10m" ✓
- Reads 50m → "95% likely a sensor failure; act cautiously" ⚠

---

## 9. Kalman Filter Localization

### Motivation

Markov localization is highly general but **computationally inefficient** for many real-world applications. Instead, a simpler, more computationally tractable approach focuses on the **sensor fusion problem**.

### What is the Kalman Filter?

The Kalman Filter is an **optimal recursive data-processing algorithm** that:
- Incorporates all available information regardless of precision
- Estimates the current state (position) from noisy measurements
- Fuses system knowledge (expected motion) with sensor signals (actual observations)
- Minimizes the impact of error sources from both environment and hardware

### Architecture (Figure concept)

```
Control Signal ──→ [System] ←── System Error Source
                      ↓
                 System State (unknown)
                      ↓
              [Measuring Devices] ←── Measurement Error Sources
                      ↓
              Observed Measurement
                      ↓
              [Kalman Filter]
                      ↓
          Optimal Estimate of System State
```

### Mathematical Assumptions

The Kalman Filter is mathematically **optimal** only when:
1. The system is **linear**
2. The noise is **white Gaussian noise**

In mobile robotics, these are rarely met — most systems are **inherently nonlinear**.

### Extended Kalman Filter (EKF)

To handle nonlinearity, engineers use the **Extended Kalman Filter (EKF)**:
- Linearizes the system using **Jacobians** at each time step
- Maintains functionality in nonlinear systems
- Optimality is no longer strictly guaranteed, but results are extremely useful in practice

### Kalman Filter — Concrete Example

**Scenario**: Robot tracking position using wheel encoder and noisy GPS (1D for simplicity)

**Setup**:
- State: Robot position x
- Motion model (linear): `x_t = x_{t-1} + u_t` (position + movement)
- Measurement noise: `N(0, 0.5²)` (GPS noise)
- Process noise: `N(0, 0.1²)` (odometry noise)

**Initial State**: `x̂₀ = 0m`, `P₀ = 1m²` (high uncertainty)

**Time t=1 (moved 1m, GPS reads 1.1m)**:
- Predict: `x̄₁ = 0 + 1 = 1m`, `P̄₁ = 1 + 0.01 = 1.01m²`
- Kalman Gain: `K = 1.01 / (1.01 + 0.25) = 0.80`
- Update: `x̂₁ = 1 + 0.80(1.1 - 1) = 1.08m`
- Updated uncertainty: `P₁ = (1 - 0.80) × 1.01 = 0.20m²`

**Time t=2 (moved 1m again, GPS reads 2.3m)**:
- Predict: `x̄₂ = 1.08 + 1 = 2.08m`, `P̄₂ = 0.20 + 0.01 = 0.21m²`
- Kalman Gain: `K = 0.21 / (0.21 + 0.25) = 0.46`
- Update: `x̂₂ = 2.08 + 0.46(2.3 - 2.08) = 2.18m`
- Updated uncertainty: `P₂ = 0.11m²`

**Summary Table**:

| Step | Estimate | Uncertainty |
|------|----------|-------------|
| Start | 0 m | 1.00 m² |
| t=1 | 1.08 m | 0.20 m² ↓ |
| t=2 | 2.18 m | 0.11 m² ↓ |
| t=3+ | Converges | → 0 |

**Key Insight**:
- Wheel encoder is predictable but drifts
- GPS is noisy but corrects drift
- Kalman Filter **blends both optimally**
- Uncertainty decreases over time → Robot becomes more confident

---

## 10. SLAM – Simultaneous Localization and Mapping

### Why SLAM?

Traditional mapping is **manual, costly, and time-consuming**. Static maps cannot adapt to dynamic environment changes. True robot autonomy requires **automatic map building**.

However:
- A robot's sensors have **limited range**
- The robot must physically **explore** its environment to build a map
- While exploring, it must **simultaneously localize** itself

This interdependency is the **SLAM problem**.

### The Chicken-and-Egg Problem

- For **localization**, the robot needs to know where the map features are
- For **map building**, the robot needs to know where it is on the map

SLAM solves both simultaneously.

### SLAM Defined

- **Localization** = estimating robot position/path given a **known map**
- **Mapping** = constructing a map given the **true robot path**
- **SLAM** = recovering both the robot path AND the environment map using only proprioceptive and exteroceptive sensor data

**Data used**:
- Robot displacement from odometry
- Features extracted from laser, ultrasonic, or camera images (corners, lines, planes)

**Difficulty**: Both estimated path and extracted features are corrupted by noise.

### SLAM Process Illustrated (5 stages)

**(a) Initial observation**:
Robot starts at known position (zero uncertainty). It observes feature m₀ and maps it with uncertainty from the sensor model.

**(b) Robot moves**:
Pose uncertainty grows due to odometry errors. At new position, robot observes two new features (m₁, m₂) and maps them — their uncertainty combines sensor error + robot pose uncertainty.

**(c) Map-robot correlation**:
The map becomes correlated with the robot position estimate. Updating position from a poorly-known feature propagates error — the estimates are no longer independent.

**(d) Uncertainty grows**:
As robot moves further, robot pose uncertainty and feature uncertainties grow. The robot needs to observe well-known features to reduce its uncertainty.

**(e) Loop Closure**:
The robot returns to feature m₀ (previously observed). This is called **loop closure detection**. When a loop closure is detected:
- Robot pose uncertainty **shrinks significantly**
- The map is updated
- Uncertainty of all previously observed features also reduces
- Map covariance converges toward zero

---

### 10.1 EKF-SLAM

**EKF-SLAM** (Extended Kalman Filter SLAM) simultaneously builds a map and localizes the robot in real-time.

#### How it works

Maintains a **joint state vector**:
```
State = [robot_pose (x, y, θ), landmark_1 (x₁, y₁), landmark_2 (x₂, y₂), ..., landmark_N (xN, yN)]
```

Also maintains a **joint covariance matrix** capturing:
- Uncertainty in robot pose
- Uncertainty in each landmark position
- **Correlations** between robot pose and all landmark estimates

#### Steps

1. **Prediction (Motion Update)**: Use motion model to propagate robot pose and grow uncertainty
2. **Update (Measurement Update)**: Use LIDAR/camera observations to correct predicted state via Kalman gain equations
3. **Loop Closure**: When a previously seen landmark is re-observed, uncertainty shrinks across the entire state

#### Advantages
- Real-time localization
- Continuous map updates
- Handles uncertainty explicitly

#### Challenges

1. **Quadratic Complexity**: Computation grows as O(N²) with number of features → very expensive for large environments
   - Mitigated by: dividing map into submaps and updating their covariances separately

2. **Linearization Errors**: Using Jacobians introduces approximation errors in highly nonlinear scenarios → can cause inconsistencies or divergence

3. **Data Association / Loop Closure**: Matching current observations to previously mapped features is error-prone, especially with 2D laser rangefinders
   - Cameras + SIFT-like detectors significantly improve this

4. **Growing Correlations**: As features are observed more, they become more correlated → necessary for accuracy but computationally heavy

#### EKF-SLAM with 3D Laser Scanner (Real Example)
- Robot makes 3 rounds of an environment
- **(a) Odometry only** → inconsistent, misaligned map (error accumulates)
- **(b) Scan matching** → drastically reduced odometry error, but small residual offset remains
- **(c) EKF-SLAM** → accurate map, superimposable on building blueprint

---

### 10.2 Particle Filter SLAM

**Particle Filter SLAM** is a Bayesian filtering approach that represents the robot's belief distribution not as a parametric form (like a Gaussian) but as a **set of samples (particles)** drawn randomly from the distribution.

#### Why Particle Filters?

The power of particle representation is its ability to model:
- **Any sort of distribution** (not just Gaussian)
- **Nonlinear transformations**

#### EKF vs Particle Filter (Figure comparison)

- **(a) EKF SLAM** → probability distribution shown as a 2D Gaussian ellipse
- **(b) Particle Filter SLAM** → same distribution shown as a cloud of discrete particles (denser near center = higher probability)

#### Key Properties

**Bayesian Filter**: Maintains a probability distribution over possible poses and maps based on sensor measurements and motion updates.

**Particle Representation**: Each particle = one hypothesis of `(robot_pose, map)`. Updated over time via sensor measurements and motion commands.

**Resampling (Importance Sampling)**:
- After robot moves and observes, particles are reweighted by their likelihood
- **High-likelihood particles** are kept and duplicated
- **Low-likelihood particles** are discarded
- This focuses computation on the most probable regions of state space

**Data Association**: Each particle maintains its own map and performs data association independently, allowing different particles to explore different correspondence hypotheses.

**FastSLAM**: A practical variant of particle filter SLAM that runs efficiently in real-time by factoring the joint distribution into the robot path and per-landmark maps.

#### Challenges

1. **Curse of Dimensionality**: Number of particles needed grows exponentially with state space dimension
2. **Particle Deprivation**: In high-dimensional spaces, particles can degenerate (all cluster in wrong region)
3. **Memory**: Each particle carries an entire map — for large environments this is expensive

#### Applications
- Mobile robotics
- Autonomous vehicles
- Augmented and virtual reality
- Any system requiring navigation + mapping in unknown environments

---

### 10.3 Open Challenges in SLAM

| Challenge | Description |
|-----------|-------------|
| **Dynamic Environments** | Moving objects (vehicles, humans, animals) violate the static map assumption. Algorithms must treat them as outliers or predict their motion. |
| **Multi-Robot Mapping** | Combining individual readings from multiple robots into a single global map is difficult — requires consistent coordinate frames and communication. |
| **Data Association & Loop Closure** | High sensitivity to matching errors when returning to a previously visited location. A single bad match can cause map failure. |
| **Sensor Feature Limitations** | 2D laser rangefinders struggle to identify unique, distinctive features compared to cameras. Perceptual aliasing is common in symmetric environments. |
| **Monocular Scale Ambiguity** | Single-camera systems provide only bearing information. Recovering absolute scale requires external data (GPS, IMU, or known object sizes). |

---

## Quick Summary Table

| Concept | Key Idea |
|---------|----------|
| Localization | Robot determines its position and orientation in a known/unknown environment |
| Sensor Noise | Random variations in sensor readings from environmental/hardware sources |
| Sensor Aliasing | Different locations produce identical sensor readings → ambiguity |
| Effector Noise | Imperfect robot motion → actual ≠ intended displacement |
| Odometry | Position tracking via wheel encoders; errors accumulate over time |
| Behavior-based Nav | Reactive, no explicit map; fast but not scalable |
| Map-based Nav | Explicit localization against a map; scalable, transparent, harder to build |
| Single-hypothesis belief | Robot has one best-guess position |
| Multiple-hypothesis belief | Robot tracks several possible positions simultaneously |
| Prediction update | Use motion data (proprioceptive) → uncertainty grows |
| Perception update | Use sensor data (exteroceptive) → uncertainty shrinks |
| Markov Localization | Arbitrary PDF; handles global localization; high compute cost |
| Kalman Filter | Single Gaussian belief; efficient; needs known initial position; EKF for nonlinear |
| SLAM | Build map and localize simultaneously — the chicken-and-egg problem |
| Loop Closure | Re-observing a known landmark to dramatically reduce accumulated error |
| EKF-SLAM | KF-based SLAM; quadratic complexity; accurate but expensive |
| Particle Filter SLAM | Sample-based SLAM; handles non-Gaussian distributions; FastSLAM variant |

---

*Notes compiled from Unit 3 slides of UE23CS343BB7 — Mobile and Autonomous Robots*
*Dr. Ashok Kumar Patil | Department of Computer Science and Engineering | PES University*
