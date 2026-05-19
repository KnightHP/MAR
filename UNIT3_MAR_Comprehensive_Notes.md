# Mobile and Autonomous Robotics — Unit 3: Localization
### Complete Notes | Course Code: UE23CS343BB7 | PES University

---

> **How to use these notes:** Every concept is explained from first principles with full definitions, analogies, mathematical derivations, pseudocode, and worked examples. These notes are slide-independent and completely self-contained.

---

## Table of Contents

1. [Introduction to Localization](#1-introduction-to-localization)
2. [Challenges of Localization](#2-challenges-of-localization)
   - 2.1 [Sensor Noise](#21-sensor-noise)
   - 2.2 [Sensor Aliasing](#22-sensor-aliasing)
   - 2.3 [Effector Noise and Odometry Error](#23-effector-noise-and-odometry-error)
3. [Localization-Based vs. Behavior-Based Navigation](#3-localization-based-vs-behavior-based-navigation)
4. [Belief Representations](#4-belief-representations)
   - 4.1 [Single-Hypothesis Belief](#41-single-hypothesis-belief)
   - 4.2 [Multiple-Hypothesis Belief](#42-multiple-hypothesis-belief)
5. [The Robot Localization Problem — Two Phases](#5-the-robot-localization-problem--two-phases)
   - 5.1 [Prediction Phase](#51-prediction-phase)
   - 5.2 [Perception Phase](#52-perception-phase)
6. [Classification of Localization Problems](#6-classification-of-localization-problems)
7. [Probabilistic Foundations](#7-probabilistic-foundations)
   - 7.1 [Why Probabilistic Robotics?](#71-why-probabilistic-robotics)
   - 7.2 [Random Variables and PDFs](#72-random-variables-and-pdfs)
   - 7.3 [Gaussian Distribution](#73-gaussian-distribution)
   - 7.4 [Conditional Probability](#74-conditional-probability)
   - 7.5 [Bayes' Rule — The Engine of Localization](#75-bayes-rule--the-engine-of-localization)
   - 7.6 [The Markov Assumption](#76-the-markov-assumption)
   - 7.7 [The Bayes Filter — Recursive Belief Update](#77-the-bayes-filter--recursive-belief-update)
   - 7.8 [Robot Motion Models](#78-robot-motion-models)
   - 7.9 [Robot Sensor Models](#79-robot-sensor-models)
8. [Markov Localization](#8-markov-localization)
9. [Kalman Filter Localization](#9-kalman-filter-localization)
   - 9.1 [The Five Kalman Filter Equations](#91-the-five-kalman-filter-equations)
   - 9.2 [Worked Example](#92-worked-example)
   - 9.3 [Extended Kalman Filter (EKF)](#93-extended-kalman-filter-ekf)
   - 9.4 [Markov vs. Kalman — Full Comparison](#94-markov-vs-kalman--full-comparison)
10. [SLAM — Simultaneous Localization and Mapping](#10-slam--simultaneous-localization-and-mapping)
    - 10.1 [The Chicken-and-Egg Problem](#101-the-chicken-and-egg-problem)
    - 10.2 [SLAM Process — Five Stages](#102-slam-process--five-stages)
    - 10.3 [EKF-SLAM](#103-ekf-slam)
    - 10.4 [Particle Filter SLAM and FastSLAM](#104-particle-filter-slam-and-fastslam)
    - 10.5 [Open Challenges in SLAM](#105-open-challenges-in-slam)
11. [Quick Reference Summary](#11-quick-reference-summary)

---

## 1. Introduction to Localization

### What is Localization?

**Localization** is the ability of a robot to determine its own **position and orientation** (collectively called its *pose*) within an environment. Without knowing where it is, a robot cannot navigate, plan a path, manipulate objects at known locations, or interact meaningfully with the world.

A robot's pose is typically expressed as:
> **(x, y, θ)** — x and y are position coordinates in 2D space, θ is the heading angle (orientation)

In 3D environments (UAVs, underwater robots), pose extends to 6 DOF: (x, y, z, roll, pitch, yaw).

*Analogy:* Localization is the robot equivalent of knowing your exact address in a city. You can have a perfect map of the entire city, but if you don't know which street you're currently standing on, that map is useless for navigation. Localization answers the question: **"Where am I on the map right now?"**

### Localization vs. Mapping vs. Motion Control

These three domains form an interconnected triad (visualized as a Venn diagram):

- **Localization** — "Where am I?"
- **Mapping** — "What does the world around me look like?"
- **Motion Control** — "How do I physically move?"

**SLAM (Simultaneous Localization and Mapping)** sits at the intersection of all three. It requires all three domains to function together — you need motion control to explore, localization to know where sensor readings come from, and mapping to record what you see.

### The General Localization Loop

```
┌──────────────────────────────────────────────────────────────┐
│  Encoder Data (proprioceptive)                               │
│         ↓                                                    │
│  Prediction of Position (dead reckoning / odometry)          │
│         ↓                                                    │
│  Predicted Position ──────────→ MATCHING ←── Raw Sensor Data │
│                                     │       (exteroceptive)  │
│                                     ↓                        │
│                              Matched Observations            │
│                                     ↓                        │
│                          Position Update / Estimation        │
│                                     ↓                        │
│                              New Best Estimate ──→ (loop)    │
└──────────────────────────────────────────────────────────────┘
```

**How it works:**
1. Robot uses wheel encoders to *predict* where it has moved (prediction phase)
2. Robot takes sensor readings (camera, LiDAR, sonar) and compares them against a stored map or previously seen features (matching)
3. If observations match expected features at a map location → the robot's position is confirmed or corrected (perception phase)
4. The updated position feeds the next prediction → continuous closed-loop estimation

This loop runs continuously throughout the robot's operation. The prediction phase grows uncertainty; the perception phase shrinks it. The interplay between these two is the heartbeat of localization.

---

## 2. Challenges of Localization

Three fundamental sources of uncertainty challenge any localization system:

### 2.1 Sensor Noise

**Definition:** Sensor noise refers to random variations in sensor readings that occur even when the environment has not actually changed. The sensor's output fluctuates around the true value due to hardware limitations, environmental interference, and electronics.

*Analogy:* Imagine trying to read a thermometer in a room where the temperature truly hasn't changed, but the thermometer needle jiggles slightly every time you look. The environment is stable; the instrument is not. That jiggle is noise.

**Why it matters for localization:**
If sensor readings fluctuate randomly, the robot cannot be sure whether a change in its readings reflects:
- A genuine change in the environment (something moved)
- A genuine change in the robot's position (it has moved)
- Just noise (nothing has changed)

This ambiguity directly degrades localization accuracy.

**Common sources of sensor noise:**

| Source | Effect on Sensor Output |
|---|---|
| Lighting changes (indoor flicker, cloud shadows) | Camera color values fluctuate even for static scene |
| Electronic interference / EMI | Random spikes in sensor signals |
| Thermal drift in electronics | Baseline of sensor output drifts slowly |
| Picture jitter in cameras | Frames shift slightly between captures |
| Signal gain variations | Inconsistent brightness across frames |
| Blooming (CCD cameras) | Bright objects cause charge overflow into neighboring pixels |
| Motion blur | Fast-moving camera blurs features, reducing detectability |

**Concrete example — CCD camera indoors:**
A robot uses camera color information to detect its position relative to colored markers. Sunlight through a window shifts color values between frames. Even with the robot stationary and the markers stationary, the camera reads slightly different RGB values every frame → localization algorithm receives a fluctuating signal → position estimate drifts.

**Solutions to sensor noise:**

**Temporal Fusion:** Average sensor readings over multiple time steps. Random noise tends to cancel out when averaged. Systematic offsets remain (noise ≠ bias).

**Multi-Sensor Fusion:** Combine fundamentally different sensor types.
- Camera + LiDAR: camera fails in darkness, LiDAR doesn't; LiDAR fails on glass, camera can detect glass
- Sonar + IMU: sonar gives distance, IMU gives orientation; combined → stable position estimate
- GPS + Vision: GPS corrects long-term drift, vision provides short-term accuracy

The combination produces *complementary coverage* — each sensor's weaknesses are covered by another sensor's strengths.

---

### 2.2 Sensor Aliasing

**Definition:** Sensor aliasing occurs when **different physical locations in the environment produce identical (or nearly identical) sensor readings**, creating positional ambiguity. The robot cannot distinguish between two or more possible locations because they look the same to its sensors.

*Analogy 1:* Imagine being blindfolded and placed in a hotel corridor. Every room door looks the same; every stretch of corridor feels the same. You can't tell if you're on the 3rd or 5th floor, or which end of the corridor you're on. Your "senses" (touch, corridor smell) produce identical readings regardless of your actual position. That's aliasing.

*Analogy 2:* A long symmetric building — if you can only see a section of straight corridor with identical tiles, cameras produce identical readings whether you're 10m or 100m from one end. You are aliased.

**Why it's fundamentally hard:**
Aliasing is not a hardware problem — it's a *geometric problem*. No matter how good your sensor is, if two places genuinely look the same, no sensor can distinguish them without additional information.

**Robot example with range sensors:**
A robot equipped with only an ultrasonic rangefinder measures distance to the nearest obstacle. It detects "1.5 m ahead." This could mean:
- A human standing 1.5 m away
- A cardboard box at 1.5 m
- A wall segment at 1.5 m

All produce **identical sensor readings** — the rangefinder only measures distance, not identity, material, or texture.

Similarly, the sensor might read "2 m, 2 m, 2 m" in all three forward directions — which could be:
- Corridor 1 (all walls at 2 m)
- Corridor 2 (different location, same geometry)

The robot cannot tell them apart from a single reading.

**Solutions to sensor aliasing:**

1. **Temporal sequences:** Take multiple readings from different positions. If two locations look the same from position A, they may look different from position B (after moving slightly). Using a sequence of observations breaks the aliasing.

2. **Richer sensors:** Cameras provide color, texture, and fine-grained detail that range sensors miss — much harder to alias (though still possible in very symmetric environments).

3. **Multiple hypotheses:** Rather than committing to one location guess, maintain multiple simultaneous position hypotheses and let evidence accumulate to resolve the ambiguity.

4. **Probabilistic methods (Markov/Particle Filter):** Represent the belief about position as a probability distribution over all possible locations. Aliased locations all receive probability mass. As the robot moves and observes more, some hypotheses become inconsistent and their probability drops to near zero.

> **Key insight:** Aliasing improves with more data. A single observation may be aliased; a hundred observations from different positions almost never are.

---

### 2.3 Effector Noise and Odometry Error

**Definition:** Effector noise refers to errors in the robot's physical motion — the robot's actual trajectory differs from the commanded trajectory. Even if the robot commands "move forward 1 metre," it may actually move 0.94 m, slightly angled.

*Analogy:* You're driving a car and steer slightly to the right without realizing it. After one mile, you're off-course by 50 metres. Over ten miles, you're in a completely different town. Small, consistent errors in motion produce large positional errors over time. This is exactly what happens to robots via odometry error.

**Odometry** — position estimation using wheel encoders — is the most common proprioceptive localization method. The robot tracks how much each wheel has rotated and integrates this to estimate displacement.

**Dead Reckoning** extends odometry by combining:
- Wheel encoders → linear displacement
- Heading sensors (gyroscope, compass) → angular displacement
- Time → velocity × time = distance

Dead reckoning estimates position from a known starting point purely from the robot's own motion history. No external references needed. But errors accumulate continuously.

**Sources of Odometry Error:**

| # | Source | Type |
|---|---|---|
| 1 | Limited encoder resolution | Systematic |
| 2 | Wheel misalignment (axis not perfectly perpendicular to body) | Systematic |
| 3 | Unequal effective wheel diameters | Systematic |
| 4 | Changing contact point on wheel (flat spot, wear) | Systematic |
| 5 | Floor irregularities (bumps, slopes, joints) | Random |
| 6 | Wheel slip (especially on turns or slippery surfaces) | Random |

Systematic errors are *predictable and repeatable* → can be calibrated out. Random errors are *unpredictable* → can only be managed statistically.

**Three types of odometry error:**

**1. Range Error:** Error in the total distance traveled. Example: robot commands 2 m, actually travels 1.95 m due to wheel diameter miscalibration. Grows linearly with distance traveled.

**2. Turn Error:** Error in the actual rotation angle. Example: robot commands 90° turn, actually turns 88°. Causes the robot to travel in a direction offset from intended.

**3. Drift Error:** Orientation error caused by systematic differences between the two wheels (unequal diameters, unequal encoder resolutions). Even while traveling "straight," the robot slowly curves. Drift error is particularly insidious because it *compounds* — angular error causes position error to grow quadratically (a small heading error produces growing lateral displacement).

**Why turn and drift errors dominate:**
Consider a 1° angular error per meter of travel. After 10 m, the robot is pointed 10° off. After 100 m, the robot has curved significantly off its intended straight path. The *lateral* displacement grows as: Δy ≈ d × sin(θ) ≈ d × θ (radians). For large θ, this is massive.

> **Conclusion:** Dead reckoning alone is unsuitable for long-distance navigation. Periodic correction using exteroceptive sensors (cameras, LiDAR, GPS, landmarks) is essential.

---

## 3. Localization-Based vs. Behavior-Based Navigation

There are two fundamentally different approaches to robot navigation. Understanding both and their trade-offs is important.

### Option 1: Behavior-Based (Programmed) Navigation

**Concept:** The robot uses *reactive behaviors* — fixed rules that map sensor inputs directly to actuator outputs — without any explicit map or position estimate. The robot never asks "where am I?" — it just reacts to what it senses right now.

**Architecture:**
```
Sensors → [Behavior 1: Obstacle Avoidance]  ┐
         → [Behavior 2: Wall Following]      ├→ Behavior Fusion → Actuators
         → [Behavior 3: Goal Seeking]        ┘
```

Each behavior produces a control output (e.g., a steering direction and speed). These are combined (e.g., via weighted vector sum) into the robot's actual motion.

**Example:**
Robot must navigate from Room A to Room B:
- *Wall-following behavior:* Follow the left wall
- *Goal detector:* When a unique colored marker for Room B is visible, stop

No map needed. No position estimate needed. Just sensor-to-action rules.

**Advantages:**
- Fast to implement for specific, known environments
- No computational overhead of position estimation
- Robust to sensor noise in simple scenarios (reactive, not predictive)

**Disadvantages:**
- **Not scalable:** Each new environment requires redesigning the behaviors
- **Fragile:** Slight environmental changes (moved furniture, new obstacle) can break carefully designed rule sets
- **Instability risk:** When multiple behaviors are simultaneously active and conflicting, the combined output can oscillate or cause erratic motion
- **Opaque:** There is no human-readable understanding of "what the robot thinks" — no map to inspect, no position to read

---

### Option 2: Map-Based (Localization-Based) Navigation

**Concept:** The robot maintains an explicit internal model (map) of its environment and continuously estimates its position relative to that map. Navigation decisions are made based on "I am at position X on the map; I need to reach position Y."

**Architecture:**
```
Sensors → Perception → Localization / Map-Building → Cognition / Planning → Motion Control → Actuators
```

**Key Advantages:**

**1. Transparency:** The robot's belief (position + map) is explicitly represented and inspectable. A human operator can see exactly what the robot "thinks" about its environment — critical for debugging, verification, and trust.

**2. Transferability:** Providing the robot with a new map immediately makes it work in a new environment. No behavior reprogramming needed — just upload a new map.

**3. Dual use:** If the robot builds its own map (SLAM), that map can also be used by humans (floor plans, emergency services, facility management).

**4. Scalability:** Map-based systems handle arbitrarily complex environments — the complexity lives in the map, not in hand-coded rules.

**Disadvantages:**

**1. Development effort:** Building reliable localization and mapping systems requires significant engineering.

**2. Map-reality divergence:** The internal map may become inconsistent with the real environment (furniture moved, temporary obstacles, construction). A robot trusting a stale map may behave dangerously. *This is the central risk of map-based navigation.*

**3. Computational cost:** Maintaining probabilistic belief over a large map is expensive.

---

## 4. Belief Representations

### The Core Problem

A robot can never know its true position with certainty. Sensors are noisy; motion is imperfect. Therefore, the robot must maintain a *belief* — a mathematical representation of how likely it is to be at each possible position.

The key design decisions are:
- **What type of map?** (geometric, grid, topological)
- **What type of belief?** (single point, multiple hypotheses, full probability distribution)

*Analogy:* It's like trying to find your apartment in an unfamiliar city at night. You might know you're "probably on this block" (high probability region) but not the exact building. A single-hypothesis system picks one address and commits. A multiple-hypothesis system says "I could be at address A (60%) or address B (30%) or address C (10%)" and keeps all options open until more evidence arrives.

---

### 4.1 Single-Hypothesis Belief

**Definition:** The robot's entire belief about its position is expressed as a **single unique pose** — one point on the map.

Represented as: **(x, y, θ)** on a continuous map, or a single cell on a grid, or a single node on a topological graph.

**How it works:** At each time step, the robot commits to one best estimate of its position and updates it as it moves and senses.

**Advantages:**
- Simple to compute and reason with
- No ambiguity — decision-making (path planning) is straightforward
- Low computational cost

**Disadvantages:**
- If the initial position estimate is wrong, the system may never recover (the single hypothesis is simply wrong, and all subsequent updates build on that wrong foundation)
- Cannot represent situations where the robot genuinely doesn't know which of several locations it's in (aliasing scenarios)
- Forced commitment before enough evidence is gathered

---

### 4.2 Multiple-Hypothesis Belief

**Definition:** The robot simultaneously tracks **several possible positions**, each with an associated probability. The belief is a probability distribution over the entire pose space.

Representations:
- **Multiple Gaussians:** A set of Gaussian distributions, each centered on a candidate position. The robot might be at peak A (60%) or peak B (35%) or peak C (5%).
- **Probability grid:** A discrete grid where each cell has a probability value. All cells sum to 1. High-probability cells = likely robot locations.
- **Particle cloud:** A set of sample poses (particles) where denser clusters indicate higher probability.

**How belief evolves (1D example):**

1. **Initial (wide spread):** Robot is placed somewhere on a long corridor. All positions equally likely → flat, uniform distribution
2. **After first sensor reading:** Some positions become more likely (match the sensor reading); others less → distribution develops peaks
3. **After moving and sensing again:** Peaks sharpen further as new data rules out more positions
4. **Convergence:** Eventually, one peak dominates → robot has localized

*Analogy:* Imagine you're blindfolded and placed somewhere in a building. Initially, you could be anywhere (uniform belief). You take one step and feel the floor — carpet. You've narrowed down: you're in a carpeted area. You turn and touch a wall — textured. Now you know: carpeted + textured wall. After a few more observations, there's only one room in the building with that combination. You've localized.

**Advantages:**
- Can handle sensor aliasing — ambiguous readings create multiple peaks; more data resolves them
- Can recover from large errors (the true position just needs to have nonzero probability)
- Mathematically principled representation of uncertainty

**Disadvantages:**
- **Decision complexity:** When the belief has multiple peaks, which one should the robot use for path planning? Standard approach: use the mode (highest peak) or take a weighted action
- **Computational cost:** Full probability distributions over continuous pose spaces are intractable — approximations (grids, particles) are always used
- **State space explosion:** For 3D environments, the number of states is enormous

---

## 5. The Robot Localization Problem — Two Phases

Mobile robot localization is a **continuous, cyclical process** of alternating between two complementary phases:

### 5.1 Prediction Phase

**Also called:** Action Update, Motion Update, Propagation

**Input:** Previous belief + control command (motion) issued to the robot

**Output:** New predicted belief — *less certain* than before

**What happens:**
The robot uses its *motion model* to predict where it has moved. Given that it was at position x₀ and commanded to move Δ, the robot estimates it is now at position x₁. But because motion is imperfect (effector noise), the predicted position is uncertain.

**Effect on the probability distribution:**
- A sharp, certain belief (narrow peak) becomes *spread out* (wider, lower peak) after the prediction phase
- The amount of spreading depends on the uncertainty in the motion model (wheel slip, encoder resolution, etc.)

*Analogy:* You close your eyes and take 10 steps forward. You know you were at position A. But each step varies slightly — you might have taken steps of 0.9 m, 1.0 m, 1.1 m each. After 10 steps, your best estimate of position shifts forward by 10 m, but your uncertainty is now ±0.5 m or more.

**Graphically:**
Starting from a known position x₀ (a very sharp spike — Dirac delta, meaning perfect certainty), after one motion command the distribution becomes a broader Gaussian centered near x₁. After another motion command, it broadens further into an even wider Gaussian at x₂. Uncertainty accumulates with each motion.

---

### 5.2 Perception Phase

**Also called:** Measurement Update, Correction Update

**Input:** Current (uncertain) belief + new sensor observation

**Output:** Updated belief — *more certain* than before

**What happens:**
The robot takes a sensor measurement (e.g., LiDAR reads "wall is 2.1 m to the left"). The robot compares this measurement against what it *would expect* to see from each possible position on the map. Positions where the expected reading matches the actual reading gain probability; positions where they disagree lose probability.

**Effect on the probability distribution:**
- The spread-out post-prediction distribution is *multiplied* by the sensor likelihood function
- Regions where the sensor reading is consistent with the map spike up in probability
- Regions where the reading is inconsistent collapse toward zero
- The resulting distribution is *narrower* (more certain) than before measurement

*Analogy:* You've been walking blindfolded for 10 steps (uncertain position). Now you remove the blindfold and look at a landmark. You see the coffee shop sign directly ahead at about 5 m. You look at your map: the coffee shop is on Main Street, 15 m from the park. Since you know where the coffee shop is, you now know *exactly* where you are — or very close to it. The observation has drastically reduced your uncertainty.

**Graphically:**
The broad Gaussian at x₂ (post-prediction) gets multiplied by a sensor model function (itself a Gaussian peaked where the observation is consistent). The product is a much narrower Gaussian at x₂'' — the corrected position estimate.

---

## 6. Classification of Localization Problems

Not all localization problems are equally hard. They are classified by how much is initially known:

### 6.1 Local Localization (Position Tracking)

**Setting:** The robot *knows its initial position* approximately and must track how that position changes as it moves.

**Difficulty:** Low — the robot just needs to keep up with its own motion.

**Challenge:** Small errors accumulate over time (drift). Without periodic correction, the estimate diverges from reality.

**Algorithm of choice:** Kalman Filter — works well for small, unimodal uncertainty that grows slowly.

*Analogy:* You know you're at the entrance to a supermarket. You keep mental track of where you walk (turning left, going 20 m, turning right, going 10 m). After a while, you're not sure exactly which aisle you're in — position tracking error.

---

### 6.2 Global Localization

**Setting:** The robot's initial position is **completely unknown**. It could be anywhere in the map.

**Initial belief:** Uniform distribution — every position is equally likely.

**Difficulty:** Medium — the robot must identify which part of the map it's in from scratch using sensor observations.

**Algorithm of choice:** Markov Localization or Particle Filter — both can handle multi-modal distributions needed for global localization.

*Analogy:* You wake up in an unfamiliar hotel room with no phone and a map of the hotel. You don't know which room you're in. You look out the window: you see a parking garage to the north. You open the door and see a staircase. You use these observations to narrow down which room you could be in.

---

### 6.3 The Kidnapped Robot Problem

**Setting:** The robot *believes it knows its position* (it has been running normally) but has been **physically moved to a different location without knowing it**. The robot doesn't know it has been kidnapped.

**Difficulty:** High — the robot's internal belief is confidently wrong. It must detect that its confident belief has become inconsistent with sensor readings, then recover.

**Why it matters:** This scenario tests the robustness and recovery capability of localization algorithms. Commercial robots may be picked up by users and placed elsewhere (robotic vacuum cleaners, warehouse robots). The system must detect and recover gracefully.

**How it manifests:** The robot is confidently at position X on its internal map. It now consistently reads sensors that are wildly inconsistent with what position X should produce. A robust system detects this inconsistency and triggers re-localization (reverts to global localization).

**Kalman Filter failure mode:** The Kalman Filter maintains only a single Gaussian belief. When kidnapped, the true position falls completely outside the filter's belief distribution. The filter tries to "stretch" the Gaussian to accommodate inconsistent readings — but cannot represent a genuinely multi-modal solution. It becomes *irrecoverably lost*.

**Markov / Particle Filter advantage:** These maintain probability mass across many positions simultaneously. After kidnapping, while the main peak of probability is at the wrong location, small amounts of probability mass are spread everywhere including the true location. Over time, the true location consistently matches sensor readings → it gains probability mass → the filter recovers.

---

### 6.4 SLAM

**Setting:** The robot has **neither a map nor a known position**. Must build the map and find its position simultaneously.

**Difficulty:** Very high — the two unknowns are interdependent: you need the map to localize, and you need the position to build the map.

See Section 10 for full treatment.

---

### 6.5 Relative vs. Absolute Localization

| Type | Basis | Method | Key Problem |
|---|---|---|---|
| **Relative** | Internal sensors only (encoders, IMU) | Dead reckoning; integrates motion commands | Cumulative drift — error grows unboundedly over time |
| **Absolute** | External references (GPS, landmarks, beacons) | Direct measurement against known references | Environment-dependent; GPS fails indoors; landmarks must be known |

Most practical systems use *both*: relative localization for high-rate, smooth estimation between sensor updates; absolute localization periodically to correct accumulated drift.

---

### 6.6 Metric vs. Topological Localization

| Type | Description | Example |
|---|---|---|
| **Metric** | Uses precise continuous coordinates (x, y, θ). Full geometric map. Precise but computationally intensive. | Grid map + Kalman Filter |
| **Topological** | Represents environment as a graph of nodes (places) and edges (connections). "I am at node Kitchen, connected to Hallway and Dining Room." | Graph + place recognition |

**Hybrid metric-topological** approaches are common — coarse topological structure for global navigation; local metric precision for fine positioning.

---

## 7. Probabilistic Foundations

### 7.1 Why Probabilistic Robotics?

A deterministic robot model says: "Given sensor reading s, I am exactly at position x." This is almost always wrong because:
- Sensors are noisy
- Motion is imperfect
- The world is partially observable

**Probabilistic robotics** replaces single-value estimates with **probability distributions** over hypotheses. Instead of "I am at position x," the system says "I have a 70% probability of being near position x, 20% near position x', and 10% somewhere else."

This approach:
- Naturally represents and propagates uncertainty
- Gracefully handles ambiguous situations (sensor aliasing)
- Provides a principled way to fuse information from multiple sensors
- Enables recovery from large errors (kidnapping)

The main cost is computational — probability distributions are more expensive to represent and update than single values.

---

### 7.2 Random Variables and PDFs

In probabilistic robotics, sensor measurements, control inputs, and robot states are all modeled as **random variables** — quantities that take values according to probability distributions.

**Discrete random variables:** Take a finite set of values. Probability assigned to each.
> Σ p(xᵢ) = 1 over all possible values xᵢ

**Continuous random variables:** Take values in a continuum (e.g., any real number for position x).

Described by a **Probability Density Function (PDF)** p(x):
> ∫₋∞^∞ p(x) dx = 1

The PDF gives the *density* of probability at each point — you integrate over a range to get the probability of being in that range:
> P(a ≤ x ≤ b) = ∫ₐᵇ p(x) dx

*Analogy:* Think of a PDF like a weather forecast probability map. High density (tall bar) means the weather is likely in that region; low density (flat bar) means unlikely. The total area under the "likelihood map" always equals 100%.

---

### 7.3 Gaussian Distribution

The **Gaussian (Normal) distribution** is the most important distribution in localization because:
- Sensor noise is often approximately Gaussian
- Motion errors are often approximately Gaussian
- The product and convolution of Gaussians are also Gaussian → mathematically tractable

**Univariate Gaussian:**

$$p(x) = \frac{1}{\sigma\sqrt{2\pi}} \exp\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)$$

Abbreviated as: **p(x) = N(μ, σ²)**

Where:
- **μ** (mu) = mean — the center of the distribution (the "best guess")
- **σ²** (sigma squared) = variance — how spread out the distribution is
- **σ** = standard deviation — the "width" of the distribution

*Analogy:* If you ask 1000 people to estimate the height of a building from 100m away, their answers form a bell curve (Gaussian) centered on the true height. Most people guess close to the true value (peak at μ); a few guess far off (tails). The spread of their guesses is σ.

**Multivariate Gaussian (for 2D/3D pose estimation):**

For a k-dimensional vector **x** (e.g., pose = (x, y, θ)):

$$p(\mathbf{x}) = \frac{1}{(2\pi)^{k/2} |\Sigma|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x} - \boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x} - \boldsymbol{\mu})\right)$$

Where:
- **μ** = mean vector — best estimate of each dimension
- **Σ** (capital sigma) = **covariance matrix** — a k×k symmetric, positive-semidefinite matrix

The covariance matrix encodes:
- **Diagonal entries Σᵢᵢ:** Variance (uncertainty) in each dimension independently
- **Off-diagonal entries Σᵢⱼ:** Correlation between dimensions — e.g., if uncertainty in x-position correlates with uncertainty in θ (heading), the off-diagonal element captures this

**Concrete example for a robot at (1.0 m, 2.0 m, 45°):**

$$\boldsymbol{\mu} = \begin{bmatrix} 1.0 \\ 2.0 \\ 45° \end{bmatrix}, \quad \Sigma = \begin{bmatrix} 0.04 & 0 & 0 \\ 0 & 0.04 & 0 \\ 0 & 0 & 0.09 \end{bmatrix}$$

This says:
- x-position uncertainty: σ_x = √0.04 = 0.2 m
- y-position uncertainty: σ_y = 0.2 m
- Heading uncertainty: σ_θ = √0.09 = 0.3 rad ≈ 17°
- No correlation between position and heading (off-diagonals = 0)

---

### 7.4 Conditional Probability

**Definition:** The probability that random variable X takes value x, *given that* Y is known to equal y:

$$p(x \mid y) = \frac{p(x, y)}{p(y)} \quad \text{for } p(y) > 0$$

Where p(x, y) is the **joint probability** of both X = x and Y = y occurring simultaneously.

**In robotics context:**
- p(x | z) = probability the robot is at position x, given observation z
- p(z | x) = probability of observing z, given the robot is at x (the **sensor model**)
- p(xₜ | xₜ₋₁, u) = probability of being at position xₜ, given previous position xₜ₋₁ and control command u (the **motion model**)

---

### 7.5 Bayes' Rule — The Engine of Localization

**Bayes' Rule** is the mathematical foundation underlying *all* probabilistic localization algorithms. It tells us how to update our belief about a hypothesis (robot position) when we receive new evidence (sensor reading).

$$\boxed{p(x \mid z) = \frac{p(z \mid x) \cdot p(x)}{p(z)}}$$

| Term | Name | Meaning |
|---|---|---|
| **p(x \| z)** | **Posterior** | Probability of being at x *after* observing z — what we want to compute |
| **p(z \| x)** | **Likelihood** | How probable is observation z if the robot truly is at x? (from sensor model) |
| **p(x)** | **Prior** | Probability of being at x *before* the observation — our previous belief |
| **p(z)** | **Evidence (normalizer)** | Total probability of observing z across all possible positions |

The denominator p(z) is just a normalizing constant:
$$p(z) = \sum_i p(z \mid x_i) \cdot p(x_i) \quad \text{(discrete)}$$
$$p(z) = \int p(z \mid x) \cdot p(x) \, dx \quad \text{(continuous)}$$

It ensures the posterior sums/integrates to 1.

*Analogy:* You are a detective trying to determine who committed a crime. Before any evidence, each of 3 suspects is equally likely (prior = 1/3 each). The crime scene has red paint. You know Suspect A often uses red paint (high likelihood), Suspect B rarely does (low likelihood), Suspect C never does (zero likelihood). Bayes' rule says: update the probability of each suspect being guilty based on how consistent the evidence (red paint) is with each suspect. After applying the rule, Suspect A's probability rises dramatically, Suspect B's drops slightly, and Suspect C's drops to zero.

**Worked Example — Robot Room Identification:**

**Scenario:** The robot has a map with three rooms: Kitchen (K), Hallway (H), Office (O). It sees a RED WALL. Where is it?

**Prior** (uniform — robot could be anywhere initially):
> P(K) = P(H) = P(O) = 1/3 ≈ 0.333

**Sensor Model (Likelihood):**
> P(Red Wall | Kitchen) = 0.95 ← kitchen has a red wall
> P(Red Wall | Hallway) = 0.10 ← hallway rarely has red
> P(Red Wall | Office)  = 0.05 ← office almost never has red

**Evidence (normalizing constant):**
```
P(Red Wall) = P(Red|K)·P(K) + P(Red|H)·P(H) + P(Red|O)·P(O)
            = (0.95)(0.333) + (0.10)(0.333) + (0.05)(0.333)
            = 0.3164 + 0.0333 + 0.0167
            = 0.3664
```

**Posterior (applying Bayes' Rule):**
```
P(Kitchen | Red Wall) = (0.95 × 0.333) / 0.3664 = 0.3164 / 0.3664 ≈ 0.864
P(Hallway | Red Wall) = (0.10 × 0.333) / 0.3664 = 0.0333 / 0.3664 ≈ 0.091
P(Office  | Red Wall) = (0.05 × 0.333) / 0.3664 = 0.0167 / 0.3664 ≈ 0.046
```

**Result:** Robot is **86.4% confident it is in the Kitchen**. One sensor reading with a good sensor model dramatically resolves the ambiguity from 33% to 86%.

---

### 7.6 The Markov Assumption

The **Markov Assumption** is the key simplifying assumption that makes probabilistic localization computationally tractable. Without it, the robot would need to remember its entire history of all past observations and actions to estimate its current position — completely infeasible.

**Formal statement:**

> The current state xₜ is a **complete summary** of the past — given xₜ, the future is independent of the history of past states and observations.

Mathematically:
$$p(x_t \mid x_{t-1}, x_{t-2}, \ldots, x_0, z_t, z_{t-1}, \ldots, z_0, u_t, u_{t-1}, \ldots) = p(x_t \mid x_{t-1}, u_t)$$

And for observations:
$$p(z_t \mid x_t, x_{t-1}, \ldots, z_{t-1}, \ldots) = p(z_t \mid x_t)$$

**What this means in plain English:**

1. **For motion:** The probability of being at position xₜ depends *only* on where I was at the previous step (xₜ₋₁) and the motion command I just executed (uₜ). It does *not* depend on where I was two steps ago, or what I observed yesterday.

2. **For sensing:** The probability of getting observation zₜ depends *only* on my current position xₜ. It does *not* depend on my previous positions or previous observations.

*Analogy:* When you're playing chess, you only need to know the current board position to decide your next move — not the entire history of how the pieces got there. The board state is a complete summary of the history. This is the Markov property.

**Why it enables computation:**
With the Markov assumption, the robot needs to maintain only the *current* belief distribution — not a growing history of everything it has ever seen. The belief at time t is fully determined by the belief at time t-1 plus the new motion and observation. This is the recursive structure that makes the Bayes Filter work.

**When it can fail:**
The Markov assumption is violated when the environment is *dynamic* (things move) or when sensor noise is *correlated* over time (both common in practice). The algorithms still work reasonably well in mild violations, but extreme violations (e.g., people constantly moving in the environment) degrade performance.

---

### 7.7 The Bayes Filter — Recursive Belief Update

The **Bayes Filter** is the general recursive algorithm that underlies *all* probabilistic localization methods (Markov Localization, Kalman Filter, Particle Filter are all specific instances of the Bayes Filter).

**The belief at time t:**
$$bel(x_t) = p(x_t \mid z_{1:t}, u_{1:t})$$

= probability of being at pose xₜ, given all past observations z₁ through zₜ and all past control inputs u₁ through uₜ.

**The Bayes Filter Algorithm:**

```
Algorithm Bayes_Filter(bel(x_{t-1}), u_t, z_t):

  for all x_t:
    
    // --- PREDICTION STEP (Action Update) ---
    // Integrate over all possible previous positions
    bel̄(x_t) = ∫ p(x_t | u_t, x_{t-1}) · bel(x_{t-1}) dx_{t-1}
    
    // --- UPDATE STEP (Measurement Update) ---
    // Multiply by likelihood of observation given position
    bel(x_t) = η · p(z_t | x_t) · bel̄(x_t)
  
  return bel(x_t)
```

Where:
- **p(xₜ | uₜ, xₜ₋₁)** = motion model — how likely is the robot to end up at xₜ given it was at xₜ₋₁ and executed action uₜ?
- **p(zₜ | xₜ)** = sensor model — how likely is observation zₜ given the robot is at xₜ?
- **η** = normalization constant — ensures bel(xₜ) integrates to 1
- **bel̄(xₜ)** = prior belief after motion update but before measurement correction (the "predicted belief")

**Intuition:**

*Prediction step:* "Where might I have ended up after my last motion?" — integrates over all possible previous positions, weighted by how likely it was to be there, and how likely the motion took us to xₜ.

*Update step:* "Given where I might be, how consistent is my sensor reading?" — multiplies the predicted belief by how likely the sensor would produce this reading from each position. Positions where the sensor reading makes sense get amplified; positions where it doesn't make sense get suppressed.

**The Bayes Filter is the skeleton.** Every practical localization algorithm is an implementation of this skeleton with specific choices of:
- How to represent bel(xₜ) — grid, Gaussian, particles
- How to compute the motion model p(xₜ | uₜ, xₜ₋₁)
- How to compute the sensor model p(zₜ | xₜ)

---

### 7.8 Robot Motion Models

The motion model **p(xₜ | uₜ, xₜ₋₁)** describes how the robot's state evolves given a control command. It captures odometry and effector noise.

**Odometry Motion Model:**
Uses wheel encoder readings as the control input. Given that the robot traveled (Δx, Δy, Δθ) according to its encoders, the true displacement is modeled as:

$$x_t = x_{t-1} + \Delta x + \epsilon_x, \quad y_t = y_{t-1} + \Delta y + \epsilon_y, \quad \theta_t = \theta_{t-1} + \Delta\theta + \epsilon_\theta$$

Where ε_x, ε_y, ε_θ are zero-mean Gaussian noise terms:
- ε_x ~ N(0, α₁|Δx| + α₂|Δθ|)  ← x-error grows with translation AND rotation
- ε_y ~ N(0, α₃|Δy| + α₄|Δθ|)
- ε_θ ~ N(0, α₅|Δx| + α₆|Δθ|)

The α parameters are calibrated constants that capture how much error the specific robot's drivetrain introduces per unit of translation and rotation. Larger α → noisier robot.

**Key insight:** Motion error grows with the *amount of motion*, not just with time. A robot that has moved 100 m has more odometry error than one that has moved 1 m — even if the same amount of time has passed.

**Velocity Motion Model:**
Uses commanded velocities (v, ω) instead of encoder readouts. Similar Gaussian noise structure, but parameterized by velocity rather than displacement.

---

### 7.9 Robot Sensor Models

The sensor model **p(zₜ | xₜ)** describes the probability of receiving observation zₜ given that the robot is at pose xₜ. This requires a model of how each sensor generates measurements.

**Beam Model for Range Sensors (LiDAR / Sonar):**

For each beam in a scan, the expected range zₑₓₚₑctₑd is calculated by ray-casting through the known map from position xₜ. The actual reading z is modeled as a mixture of four components:

**Component 1 — Gaussian measurement noise:**
$$p_\text{hit}(z \mid x_t) \propto N(z_\text{expected}, \sigma^2)$$
Most readings cluster around the true distance. Standard deviations of 1–5 cm are typical for LiDAR.

**Component 2 — Unexpected obstacles (dynamic objects):**
$$p_\text{short}(z \mid x_t) \propto \lambda e^{-\lambda z} \text{ for } z < z_\text{expected}$$
Moving objects (people, vehicles) not in the map cause shorter-than-expected readings. Modeled as an exponential distribution weighted toward short ranges.

**Component 3 — Sensor failure (max range reading):**
$$p_\text{max}(z \mid x_t) = \mathbf{1}[z = z_\text{max}]$$
When a beam hits an absorbing surface (black material), coherently reflects away from the sensor (glass, mirrors), or hits nothing — the sensor reports maximum range. Modeled as a point mass at z_max.

**Component 4 — Random noise / interference:**
$$p_\text{rand}(z \mid x_t) = \frac{1}{z_\text{max}}$$
Uniform distribution over all possible readings — accounts for cross-talk between sensors, random electronic interference, or unmodeled effects.

**Full model:**
$$p(z \mid x_t) = \alpha_1 p_\text{hit} + \alpha_2 p_\text{short} + \alpha_3 p_\text{max} + \alpha_4 p_\text{rand}$$

Where α₁ + α₂ + α₃ + α₄ = 1 and the weights are calibrated from data.

**Concrete example for LiDAR (max range 50 m, expected 10 m):**
- Gaussian hit: 90% of readings cluster around 10 m (σ = 0.3 m)
- Short (unexpected obstacle): 3% of readings in range 0–10 m (a person walked past)
- Max range failure: 5% chance of reading 50 m (coherent reflection)
- Random: 2% uniform over 0–50 m

Robot interpretation:
- Reads 10.2 m → almost certainly the mapped wall is there ✓
- Reads 7.3 m → probably a person walked in the beam path
- Reads 50 m → likely a coherent reflection failure; do not trust this beam
- Reads 32 m → random noise; ignore

---

## 8. Markov Localization

### Overview

**Markov Localization** is a direct implementation of the Bayes Filter for robot localization. It represents the belief as an arbitrary discrete probability distribution over a *tessellated* (gridded) version of the pose space.

The pose space (x, y, θ) is divided into a finite number of discrete cells. Each cell represents a candidate robot pose and holds a probability value. All cells together sum to 1.

**Number of states:**
In a typical indoor environment:
- x: 10 m range at 0.1 m resolution → 100 cells
- y: 10 m range at 0.1 m resolution → 100 cells
- θ: 360° at 5° resolution → 72 cells
- Total: 100 × 100 × 72 = **720,000 cells**

For larger environments or finer resolution → millions of cells. Computationally expensive but tractable with modern hardware.

### Full Algorithm

```
Algorithm Markov_Localization(bel(x_{t-1}), u_t, z_t, Map M):

  for all x_t:
    
    // --- PREDICTION (Action Update) ---
    bel̄(x_t) = Σ_{x_{t-1}} p(x_t | u_t, x_{t-1}) · bel(x_{t-1})
    //         Sum over all previous states; weight by motion model
    
    // --- UPDATE (Measurement Update) ---
    bel(x_t) = η · p(z_t | x_t, M) · bel̄(x_t)
    //         Scale by likelihood of sensor reading given this pose and map
  
  normalize: bel(x_t) ← bel(x_t) / Σ bel(x_t)   // ensure total = 1
  return bel(x_t)
```

### Initialization

**Known initial position:** bel(x₀) is a Dirac delta (spike) at the known starting pose. All probability mass concentrated at one cell.

**Unknown initial position (global localization):** bel(x₀) = uniform distribution — 1/N probability at each of N cells. The robot could be anywhere.

### Sensor Model for Markov Localization

For each cell (candidate pose xₜ), the sensor model p(zₜ | xₜ, M) assigns a probability to the observed sensor reading given that pose and the map. This probability is used to weight (scale) that cell's probability in the update step.

**Three key properties of the beam sensor model in Markov:**

**1. Gaussian error distribution:**
The range sensor error follows a Gaussian centered at the correct expected reading. Most readings are close to the true distance; errors decrease sharply away from the true value.

**2. Nonzero probability across all readings:**
All possible readings (even outliers far from the expected value) have a nonzero probability. The Gaussian has infinite tails — rare readings are possible, just unlikely. This prevents the algorithm from completely zeroing out a cell based on one unlikely reading.

**3. Spike at maximum range:**
A small but nonzero probability is assigned to the maximum range reading. This accounts for the coherent reflection / absorption failure mode. Without this, the max-range readings would cause numerical problems (probabilities going to 0 for all cells).

### Handling Localization Problems

**Global Localization:** Initialize with uniform distribution. Let the update step concentrate probability on consistent poses over multiple observations. Works naturally.

**Position Tracking:** Initialize with a sharp spike at the known starting pose. Run the algorithm. The spike spreads slightly with each motion (prediction) and sharpens with each observation (update).

**Kidnapped Robot:** The main probability mass is at the wrong location (where the robot believed it was before kidnapping). After kidnapping, sensor readings are inconsistent with the main peak. However, small amounts of probability mass exist at the true location. Over several observations, the true location consistently matches → its probability grows → the filter recovers. (This assumes some probability mass never fully collapses to zero — maintained by the noise model.)

### Computational Considerations

**Cost per update:** O(N × M) where N = number of state cells, M = number of sensor beams per scan.

For 720,000 states × 360 beams = 259 million operations per update → expensive but feasible at modern CPU speeds (milliseconds per update).

**Optimization:** Only update cells within the reachable range of the motion model (cells too far from the previous belief have zero motion probability and can be skipped).

---

## 9. Kalman Filter Localization

### Motivation

Markov Localization is powerful but computationally expensive for large, fine-grained environments. When the robot's belief can be well-approximated as a **single Gaussian distribution** (unimodal, symmetric uncertainty), the Kalman Filter provides an *exact, optimal, and computationally efficient* alternative.

The Kalman Filter (KF) is the *Bayes Filter specialized to Gaussian distributions and linear systems.*

*Analogy:* Markov localization is like tracking every possible hiding spot for a rabbit in a field (discrete grid, all possibilities). Kalman filter is like tracking the rabbit's position with a GPS transponder (one estimate + confidence ellipse). If you know the rabbit is somewhere specific (unimodal belief), the second approach is far more efficient.

### Key Assumptions for Kalman Filter Optimality

1. **Linear system:** The motion model must be a *linear* function of the state. A robot moving in a straight line is linear. Turning is nonlinear — handled by EKF.

2. **Linear observation model:** The sensor measurement must be a linear function of the state.

3. **Gaussian noise:** All noise (motion noise, sensor noise) must be zero-mean Gaussian.

Under these assumptions, the Kalman Filter is mathematically **optimal** — no other algorithm can extract more information from the available data.

### State Representation

The robot's belief is parameterized by just two quantities:
- **μₜ** (mu_t): The mean pose vector — the best estimate of the robot's current position/orientation
- **Σₜ** (Sigma_t): The covariance matrix — capturing uncertainty and correlations between state dimensions

For a 2D mobile robot: μ = (x, y, θ)ᵀ and Σ is a 3×3 positive semidefinite matrix.

### 9.1 The Five Kalman Filter Equations

The Kalman Filter alternates between two steps (prediction and update), implemented via five equations:

**Variables:**
- μₜ₋₁, Σₜ₋₁ — previous belief (mean and covariance)
- uₜ — control input (motion command)
- zₜ — sensor measurement
- **A** — state transition matrix (how state evolves with time, without control)
- **B** — control input matrix (how control affects state)
- **C** — observation matrix (how state maps to sensor reading)
- **R** — process noise covariance (motion uncertainty)
- **Q** — measurement noise covariance (sensor uncertainty)

---

#### PREDICTION STEP (Equations 1 & 2)

**Equation 1 — Predicted mean:**
$$\bar{\mu}_t = A \mu_{t-1} + B u_t$$

The predicted mean is the linear forward projection of the previous mean using the motion model. A transforms the state (e.g., applies time dynamics), B applies the effect of the control input.

**Equation 2 — Predicted covariance:**
$$\bar{\Sigma}_t = A \Sigma_{t-1} A^T + R$$

The predicted covariance grows — the previous uncertainty (AΣA^T, uncertainty after state transition) plus added process noise R (motion model noise). This is the mathematical representation of "uncertainty grows with motion."

*Why AΣA^T?* The covariance transforms under linear transformation A as AΣA^T. Think of it as rotating and stretching the uncertainty ellipse according to how the motion model rotates and stretches the state space.

---

#### UPDATE STEP (Equations 3, 4 & 5)

**Equation 3 — Kalman Gain:**
$$K_t = \bar{\Sigma}_t C^T (C \bar{\Sigma}_t C^T + Q)^{-1}$$

The Kalman Gain **K** is the most important quantity — it determines *how much to trust the sensor reading* vs. *the prediction*.

- **Numerator** (C̄Σ_t C^T): Uncertainty in the *predicted measurement* (what we expected to see, expressed in measurement space)
- **Denominator** adds Q: measurement noise

**Interpretation of K:**
- If sensor noise Q is **very small** (highly accurate sensor): K → C⁻¹ → trust the sensor almost completely
- If predicted uncertainty Σ̄ is **very small** (motion model very accurate): K → 0 → trust the motion model, ignore the sensor
- In practice: K is somewhere in between, optimally blending both

*Analogy:* K is like deciding how much to trust a GPS reading vs. your odometer. If the GPS is highly accurate (Q small), trust the GPS. If you've been moving very precisely (Σ̄ small), trust the odometer. K computes the optimal blend.

**Equation 4 — Updated mean:**
$$\mu_t = \bar{\mu}_t + K_t(z_t - C\bar{\mu}_t)$$

The updated mean is the predicted mean *plus a correction*. The correction is K times the *innovation* (z_t - Cμ̄_t):
- **z_t** = actual sensor measurement received
- **Cμ̄_t** = predicted sensor measurement (what we expected to see from position μ̄_t)
- **Innovation** = z_t - Cμ̄_t = the "surprise" — how much the actual reading differs from the prediction

If the sensor reading matches the prediction exactly (no surprise), the innovation = 0 → no correction. If the sensor reading deviates significantly, K amplifies the correction accordingly.

**Equation 5 — Updated covariance:**
$$\Sigma_t = (I - K_t C) \bar{\Sigma}_t$$

The updated covariance is always *smaller* than the predicted covariance (since (I - KC) shrinks it). Each sensor observation reduces uncertainty. The more informative the sensor (larger K), the more the covariance shrinks.

Over many updates: Σₜ → 0 (in the limit, perfect certainty — never quite reached in practice).

### 9.2 Worked Example

**Scenario:** 1D robot tracking position x using wheel encoder and a noisy range sensor.

**Parameters:**
- A = 1 (position stays, no dynamics without control)
- B = 1 (move exactly as commanded — before noise)
- C = 1 (sensor directly measures position)
- R = 0.01 m² (process noise — odometry uncertainty)
- Q = 0.25 m² (measurement noise — range sensor uncertainty, σ = 0.5 m)

**Initial state:** μ₀ = 0 m, Σ₀ = 1.0 m² (uncertain starting position)

---

**Time t = 1: Robot commanded to move u₁ = 1.0 m. Sensor reads z₁ = 1.1 m.**

*Prediction:*
- μ̄₁ = 1 × 0 + 1 × 1.0 = **1.0 m**
- Σ̄₁ = 1 × 1.0 × 1 + 0.01 = **1.01 m²**

*Kalman Gain:*
- K₁ = 1.01 × 1 × (1 × 1.01 × 1 + 0.25)⁻¹ = 1.01 / 1.26 = **0.802**

*Update:*
- μ₁ = 1.0 + 0.802 × (1.1 − 1.0) = 1.0 + 0.0802 = **1.080 m**
- Σ₁ = (1 − 0.802 × 1) × 1.01 = 0.198 × 1.01 = **0.200 m²**

Uncertainty dropped from 1.01 m² → 0.200 m² after just one sensor reading.

---

**Time t = 2: Robot commanded to move u₂ = 1.0 m. Sensor reads z₂ = 2.3 m.**

*Prediction:*
- μ̄₂ = 1.080 + 1.0 = **2.080 m**
- Σ̄₂ = 0.200 + 0.01 = **0.210 m²**

*Kalman Gain:*
- K₂ = 0.210 / (0.210 + 0.25) = 0.210 / 0.460 = **0.457**

*Update:*
- Innovation = 2.3 − 2.080 = 0.220 m (sensor reads 22 cm more than predicted)
- μ₂ = 2.080 + 0.457 × 0.220 = 2.080 + 0.100 = **2.180 m**
- Σ₂ = (1 − 0.457) × 0.210 = 0.543 × 0.210 = **0.114 m²**

---

**Summary:**

| Step | Mean Estimate | Uncertainty (Σ) | Interpretation |
|---|---|---|---|
| t = 0 | 0.000 m | 1.000 m² | Unknown start |
| t = 1 | 1.080 m | 0.200 m² ↓↓ | Large drop after first measurement |
| t = 2 | 2.180 m | 0.114 m² ↓ | Continuing to narrow |
| t → ∞ | Converges to true position | → 0 m² | Near-perfect certainty |

**Key insight:** The Kalman gain K₁ = 0.80 means the filter trusted the sensor heavily (sensor reading pulled the estimate 80% of the way from prediction to measurement). K₂ = 0.46 is lower because the uncertainty had already shrunk — the prior from the previous step was already quite good, so the new sensor reading has less leverage.

---

### 9.3 Extended Kalman Filter (EKF)

**Problem:** Real robots are *nonlinear*. A robot that rotates and then translates has a nonlinear motion model. A camera that measures bearing angles has a nonlinear observation model. The standard Kalman Filter requires linear functions — so it cannot be applied directly.

**Solution:** The **Extended Kalman Filter (EKF)** linearizes the nonlinear functions at each time step using *first-order Taylor expansion* — specifically, using **Jacobian matrices**.

#### What is a Jacobian?

A **Jacobian matrix** is a matrix of all first-order partial derivatives of a vector-valued function. It is the multidimensional generalization of a derivative.

For a nonlinear vector function **g(x)** mapping an n-dimensional input to an m-dimensional output:

$$J_g = \frac{\partial g}{\partial x} = \begin{bmatrix} \frac{\partial g_1}{\partial x_1} & \cdots & \frac{\partial g_1}{\partial x_n} \\ \vdots & \ddots & \vdots \\ \frac{\partial g_m}{\partial x_1} & \cdots & \frac{\partial g_m}{\partial x_n} \end{bmatrix}$$

*Analogy:* The derivative of a 1D function gives the slope (rate of change) at a point — a linear approximation of the curve at that point. The Jacobian does the same thing for multidimensional functions — it gives the best linear approximation of how the function changes near the current operating point.

**Why linearization works:**
If we know the current state estimate μₜ (our best guess), the true state is *near* μₜ (it differs only by small errors). In a small region around μₜ, a nonlinear function can be approximated as linear. This linear approximation is the Jacobian.

The linearization is *only valid locally* (near the operating point). This is why EKF is approximate and can fail in highly nonlinear scenarios — the linear approximation breaks down far from the true state.

#### EKF Equations

Let the nonlinear motion model be **g(uₜ, xₜ₋₁)** and the nonlinear observation model be **h(xₜ)**.

**EKF Jacobians:**
- **Gₜ** = ∂g/∂x evaluated at μₜ₋₁ — Jacobian of the motion model (replaces A in standard KF)
- **Hₜ** = ∂h/∂x evaluated at μ̄ₜ — Jacobian of the observation model (replaces C in standard KF)

**EKF Prediction:**
$$\bar{\mu}_t = g(u_t, \mu_{t-1})$$
$$\bar{\Sigma}_t = G_t \Sigma_{t-1} G_t^T + R$$

**EKF Update:**
$$K_t = \bar{\Sigma}_t H_t^T (H_t \bar{\Sigma}_t H_t^T + Q)^{-1}$$
$$\mu_t = \bar{\mu}_t + K_t (z_t - h(\bar{\mu}_t))$$
$$\Sigma_t = (I - K_t H_t) \bar{\Sigma}_t$$

The structure is identical to the standard KF, but:
- A → Gₜ (Jacobian of motion model, changes at every step)
- C → Hₜ (Jacobian of observation model, changes at every step)
- The mean predictions use the actual nonlinear functions g and h

**When EKF works well:** Mildly nonlinear systems where the Gaussian approximation of the belief remains valid (uncertainty is small enough that the linearization error is negligible).

**When EKF fails:** Highly nonlinear systems, or when uncertainty is large (the Gaussian assumption breaks down, multiple modes exist).

---

### 9.4 Markov vs. Kalman — Full Comparison

| Property | Markov Localization | Kalman Filter (EKF) |
|---|---|---|
| **Belief representation** | Any arbitrary PDF over discrete grid | Single Gaussian (μ, Σ) |
| **Initial position required?** | No — uniform distribution works | Yes — needs approximate start |
| **Multiple hypotheses** | ✓ Yes — can represent multi-modal belief | ✗ No — unimodal only |
| **Computational cost** | High — O(N²) per update for N states | Low — O(k²) for k-dim state |
| **Map type** | Discrete (grid or topological) | Continuous geometric |
| **Handles global localization?** | ✓ Yes | ✗ No (no uniform prior) |
| **Handles kidnapped robot?** | ✓ Yes — probability mass at true location | ✗ No — irrecoverably lost |
| **Precision** | Limited by grid resolution | High — continuous space |
| **Nonlinear systems** | Naturally handles (no linearity required) | EKF needed; approximate |
| **Convergence** | Guaranteed for unimodal distributions | Fast convergence when initialized well |
| **Memory** | O(N) cells for N states | O(k²) for k-dim state |

---

## 10. SLAM — Simultaneous Localization and Mapping

### 10.1 The Chicken-and-Egg Problem

**Localization** assumes a known map: "Given this map, where am I?"

**Mapping** assumes a known position: "Given that I'm at this position, what does the world look like?"

**SLAM** must do both simultaneously, with neither given: neither the map nor the robot's position is known. This creates a circular dependency:

```
To localize → Need a map
To build the map → Need to know position
```

This is the fundamental challenge SLAM addresses. Both the robot's trajectory and the environmental map are estimated simultaneously from only:
- Proprioceptive sensor data (odometry — wheel encoders, IMU)
- Exteroceptive sensor data (range, vision — feature observations)

Both are corrupted by noise. This makes SLAM inherently uncertain and probabilistic.

*Analogy:* You're an explorer mapping an unknown island. You sketch the coastline as you walk. But your own position on the sketch depends on how accurately you've recorded your past movements. Any error in your movement recording causes your sketch to be distorted. Yet your movement recording depends on what you've already mapped (using landmarks). Neither the map nor your position can be established without the other.

**Why SLAM matters:**
- Manual map creation is expensive, slow, and becomes outdated
- Many environments (disaster zones, foreign planets, undersea) cannot be manually mapped
- True autonomy requires the ability to explore and operate in unmapped environments

---

### 10.2 SLAM Process — Five Stages

**(a) Initial state:**
Robot starts at a known position (zero uncertainty). It observes feature m₀ (e.g., a corner) and records it on the map. The feature's position uncertainty reflects only sensor noise (the robot's position is certain, so feature uncertainty = sensor uncertainty).

**(b) Robot moves (odometry errors accumulate):**
The robot moves toward new areas. Due to odometry noise, robot pose uncertainty *grows*. The robot now observes new features m₁ and m₂. Their mapped positions have uncertainty combining:
- Robot pose uncertainty (where was I when I observed this?)
- Sensor measurement noise

**(c) Map-robot correlation develops:**
A crucial effect: the robot's position uncertainty and the feature position uncertainties are **correlated**. They share a common source of error — the odometry noise that affects the robot's path. This means updating the robot's position from a feature observation also updates the feature's estimated position (and vice versa). The estimates are no longer independent.

**Important implication:** You cannot treat the robot and map features as independent random variables. Their joint distribution must be maintained, including the cross-correlations. This is what makes SLAM hard — the correlations grow with the number of features.

**(d) Uncertainty continues to grow:**
As the robot explores further, pose uncertainty and feature uncertainties grow. The map becomes increasingly uncertain. The robot needs to observe previously mapped features to "anchor" its position.

**(e) Loop Closure — the key event:**
The robot returns to a previously visited area and re-observes feature m₀ (or any previously mapped feature). This is **loop closure detection**.

**What loop closure does:**
- Comparing the new observation of m₀ to its stored position reveals the accumulated error in the robot's estimated trajectory
- The error between expected and observed position of m₀ is propagated *back through the entire trajectory*
- Robot pose uncertainty **drops dramatically**
- *All* feature uncertainties drop simultaneously — even features not re-observed benefit from the loop closure
- The map covariance matrix converges toward a consistent state

*Analogy:* You've been sketching the island coastline as you walk, accumulating position errors. You return to your starting camp. You know exactly where your camp is — it's the anchor. By comparing where you drew the camp on your sketch to where it actually is, you can calculate how much error has accumulated and correct your entire map accordingly. This is loop closure.

> **Fundamental SLAM result:** Given sufficient loop closures, the errors in the map of feature landmarks become **fully correlated** and the uncertainty of the entire map converges to zero in the limit. The map covariance matrix approaches a constant.

---

### 10.3 EKF-SLAM

**EKF-SLAM** applies the Extended Kalman Filter to the full SLAM problem by maintaining a *joint state vector* containing both the robot's pose and all landmark positions simultaneously.

#### Joint State Vector

$$\mathbf{x} = \begin{bmatrix} x_r \\ y_r \\ \theta_r \\ m_{1x} \\ m_{1y} \\ m_{2x} \\ m_{2y} \\ \vdots \\ m_{Nx} \\ m_{Ny} \end{bmatrix}$$

For a robot pose (3 values) + N landmarks (2 values each):
**Dimension of state vector = 3 + 2N**

#### Joint Covariance Matrix

The covariance matrix Σ has dimension (3+2N) × (3+2N):

$$\Sigma = \begin{bmatrix} \Sigma_{rr} & \Sigma_{rm} \\ \Sigma_{mr} & \Sigma_{mm} \end{bmatrix}$$

Where:
- **Σ_rr** (3×3): Uncertainty in robot pose — how uncertain are we about x_r, y_r, θ_r?
- **Σ_mm** (2N×2N): Uncertainty in all landmark positions — how uncertain are we about each landmark?
- **Σ_rm = Σ_mr^T** (3×2N): **Cross-correlations** between robot pose and landmark positions — the critical coupling term. This is what makes SLAM hard and what enables loop closure.

*Analogy for the covariance matrix:* Think of it as a trust matrix. The diagonal tells you how much you trust each individual estimate. The off-diagonal terms tell you how the estimates are linked — if you learn something about the robot's true position, these off-diagonal terms tell you how much to adjust each landmark estimate accordingly.

#### EKF-SLAM Algorithm

**Step 1 — Prediction (motion update):**
- Apply the motion model: robot pose mean μ_r is updated using odometry
- Robot pose covariance Σ_rr grows (odometry noise added via Jacobian Gₜ)
- Cross-correlations Σ_rm also updated (robot moved → its correlation with landmarks changes)
- **Landmark estimates do not change** during prediction (the robot moved; landmarks didn't)

**Step 2 — Update (measurement update):**
When the robot observes a landmark at position (r, φ) in polar coordinates (range r, bearing φ):
- Compute expected observation from current state estimate (h(μ̄))
- Compute Kalman gain K using the observation Jacobian Hₜ
- Update robot pose μ_r (improves position estimate)
- Update landmark position μ_m (improves landmark estimate)
- Update cross-correlations Σ_rm (all elements of the joint covariance matrix change)

**Step 3 — New landmark initialization:**
When the robot sees a feature it hasn't seen before → add it to the state vector and covariance matrix with initial uncertainty reflecting sensor noise + current robot pose uncertainty.

**Step 4 — Loop closure:**
When the robot re-observes a known landmark, the update propagates across the entire covariance matrix, reducing uncertainty for all correlated states.

#### Computational Complexity

**O(N²) per update** where N is the number of landmarks in the map.

This is the fundamental bottleneck of EKF-SLAM. With N = 1000 landmarks → 1,000,000 covariance matrix elements to update per step. With N = 10,000 → 100,000,000 elements. This makes EKF-SLAM impractical for large-scale environments without approximation.

**Mitigation strategies:**
- **Submapping:** Divide the environment into local submaps. Maintain a precise local EKF within each submap; use a coarser global EKF to connect submaps. Reduces complexity to O(M²) per local map with M << N total landmarks.
- **Sparse extended information filters:** Exploit sparsity in the information matrix (inverse of covariance).
- **iSAM (Incremental Smoothing and Mapping):** Incrementally updates a sparse factor graph representation.

#### EKF-SLAM Challenges

**1. Quadratic complexity:** O(N²) per step → intractable for large maps. Workaround: submapping.

**2. Linearization errors:** The EKF linearizes nonlinear sensor models (bearing-only, range-bearing). In regions of high curvature (large uncertainty, highly nonlinear sensor geometry), linearization errors cause the filter to become *inconsistent* — it underestimates uncertainty, making the estimates overconfident and eventually divergent.

**3. Data Association:** Every time the robot observes a feature, it must determine: "Is this a new landmark, or a previously seen one?" This is the **data association problem**.
- Wrong association → catastrophic failure (map is merged incorrectly)
- In featureless environments (symmetric corridors, open fields), data association is very hard
- Cameras with SIFT/ORB descriptors greatly improve data association over bare LiDAR geometry
- **Perceptual aliasing** (two different places looking the same) is the key failure mode

**4. Growing correlations:** As the number of features grows and the robot travels longer, more elements of the covariance matrix become nonzero (correlated). This is necessary for correctness but computationally heavy.

#### Real-World Validation

**3-round building experiment:**
- (a) Odometry only: map is badly misaligned, walls don't close into rooms, severe cumulative error
- (b) Scan matching alone: dramatically better, but small residual offset remains where loop should close
- (c) EKF-SLAM: accurate map superimposable on the actual building blueprint — loop closure fully corrected accumulated error

---

### 10.4 Particle Filter SLAM and FastSLAM

#### Why Particle Filters for SLAM?

EKF-SLAM's Gaussian assumption can fail when:
- The robot's pose is globally uncertain (multiple peaks — Gaussian is wrong)
- The motion model is highly nonlinear
- The sensor model is non-Gaussian

Particle filters represent the belief as a *set of samples* (particles), each being one possible world state. They can represent *any distribution* — not just Gaussians.

#### Particle Representation

Each **particle** pᵢ represents one hypothesis about the complete robot state:

$$p^{(i)} = \left\langle x_r^{(i)}, m^{(i)} \right\rangle$$

Where:
- **x_r^(i)** = one possible robot pose (position + heading)
- **m^(i)** = one possible map associated with this robot pose

A cloud of N particles collectively represents the probability distribution:
- Dense cluster of particles at some location → high probability there
- Sparse region → low probability

*Analogy:* Think of each particle as a "parallel universe" — one theory about where the robot is and what the map looks like. After each observation, theories that are inconsistent with reality become less numerous; theories consistent with reality become more numerous. Eventually, all surviving theories agree on the true state.

#### The Particle Filter Algorithm

```
Algorithm Particle_Filter(X_{t-1}, u_t, z_t):

  X̄_t = {}  // empty temporary particle set
  
  for i = 1 to N:
    // SAMPLE: propagate particle through motion model
    x_t^(i) ~ p(x_t | u_t, x_{t-1}^(i))
    
    // WEIGHT: evaluate particle against sensor reading
    w_t^(i) = p(z_t | x_t^(i))
    
    X̄_t += {x_t^(i), w_t^(i)}
  
  // RESAMPLE: draw N particles from X̄_t with replacement
  // proportional to weights w_t^(i)
  X_t = Resample(X̄_t, N)
  
  return X_t
```

**Step-by-step:**

**1. Sample (Propagation):** Each particle's pose is propagated forward using the motion model, with random noise added. If the robot commanded a 1 m forward move, each particle moves forward approximately 1 m but with slightly different noise realizations (some particles move 0.95 m, others 1.05 m, some slightly off-angle).

**2. Weight (Measurement likelihood):** Each particle's weight w^(i) is set to p(z_t | x_t^(i)) — the probability that the sensor would produce the observed reading z_t if the robot were truly at the particle's pose x_t^(i). Particles near the true position (where sensor readings are consistent with the map) receive high weight. Particles far from the true position receive low weight.

**3. Resample (Importance Sampling):** Draw N particles from the weighted set, with replacement, proportional to weights:
- High-weight particles are likely to be selected multiple times (they're "good" hypotheses)
- Low-weight particles are likely to be discarded (inconsistent with observations)
- After resampling, all surviving particles have equal weight

*Analogy for resampling:* It's like a lottery where high-weight particles buy more lottery tickets. The winners are duplicated; the losers disappear. Over many generations, only the most consistent hypotheses survive.

**Convergence:** Over successive iterations, particles concentrate in regions consistent with all past observations. For unimodal distributions (local localization), the cloud converges to a tight cluster around the true pose. For multi-modal distributions (global localization), multiple clusters may persist until enough evidence resolves the ambiguity.

#### FastSLAM

**FastSLAM** (Montemerlo et al., 2002) is the landmark efficient variant of particle filter SLAM. It exploits a crucial insight:

**Key factorization:**
$$p(x_{0:t}, m | z_{1:t}, u_{1:t}) = p(x_{0:t} | z_{1:t}, u_{1:t}) \cdot \prod_{k=1}^{N} p(m_k | x_{0:t}, z_{1:t})$$

= (distribution over robot paths) × (product of per-landmark distributions)

**Why this helps:** Given the robot's full path, each landmark position is **conditionally independent** of all other landmarks. Therefore, the per-landmark maps can be estimated independently using N separate, low-dimensional Kalman Filters (one per landmark).

**Structure of each FastSLAM particle:**
```
Particle i = {
  robot_path^(i),        // one sampled trajectory
  KF_1^(i): (μ₁, Σ₁),  // EKF for landmark 1
  KF_2^(i): (μ₂, Σ₂),  // EKF for landmark 2
  ...
  KF_N^(i): (μ_N, Σ_N) // EKF for landmark N
}
```

Each particle carries its own full map, but each per-landmark EKF is only 2D (or 3D) — trivially fast to update.

**Computational complexity:** O(M log N) per time step where M = number of particles and N = number of landmarks (the log N comes from efficient landmark lookup using a tree structure).

Compare to EKF-SLAM's O(N²) per step — FastSLAM scales logarithmically rather than quadratically. Massive advantage for large maps.

**FastSLAM 2.0:** An improved version that also uses the current observation to sample better robot poses (proposals), dramatically improving particle efficiency.

#### EKF-SLAM vs. Particle Filter SLAM

| Property | EKF-SLAM | Particle Filter (FastSLAM) |
|---|---|---|
| **Belief type** | Single Gaussian joint distribution | Set of N sample hypotheses |
| **Distribution shape** | Unimodal (one peak) | Any shape (multi-modal possible) |
| **Complexity** | O(N²) — quadratic in landmarks | O(M log N) — near-linear |
| **Scalability** | Poor for large maps | Good |
| **Consistency** | Can become inconsistent (linearization errors) | More consistent (no linearization) |
| **Data association** | One global hypothesis | Per-particle (can explore multiple) |
| **Memory** | O(N²) for covariance matrix | O(M×N) for M particles × N landmarks |
| **Handles non-Gaussian** | No | Yes |

---

### 10.5 Open Challenges in SLAM

| Challenge | Description | Current Research Directions |
|---|---|---|
| **Dynamic Environments** | Moving objects (people, vehicles) violate the static world assumption. The map is built for a static world, but real environments constantly change. | Treating dynamic objects as outliers (RANSAC-based filtering); semantic SLAM that separately models static and dynamic elements; predicting motion of dynamic objects |
| **Multi-Robot SLAM** | Multiple robots building a shared global map. Requires consistent coordinate frames, handling robots with different starting positions, communication overhead, and merging maps from different perspectives. | Distributed map merging; place recognition for relative pose initialization; decentralized communication protocols |
| **Data Association and Loop Closure** | Correctly recognizing previously visited places is critical — a single wrong association corrupts the entire map. Perceptual aliasing (same-looking places) makes this particularly hard. | Deep learning-based place recognition (NetVLAD, BoW-visual); geometric consistency checks; switchable constraints in pose graphs |
| **Sensor Limitations (2D LiDAR)** | 2D laser scans only capture a horizontal slice. In environments with similar geometry in that slice (symmetric corridors), data association becomes very difficult. | 3D LiDAR; camera-based SLAM (ORB-SLAM3); multi-modal fusion |
| **Monocular Scale Ambiguity** | Single cameras provide *bearing-only* information — they tell you direction but not distance. Without a known baseline or reference object, absolute scale cannot be recovered from monocular images alone. | Scale initialization from known objects; fusion with IMU (Visual-Inertial Odometry); using depth cameras or stereo |
| **Long-term Autonomy** | Maps become stale as environments change over months/years. Seasonal changes, furniture moves, structural modifications. | Lifelong mapping with change detection; probabilistic occupancy with decay over time; experience-based mapping |

---

## 11. Quick Reference Summary

| Concept | Key Term | One-Line Definition |
|---|---|---|
| Localization | Pose estimation | Robot determines its (x, y, θ) in a known or unknown environment |
| Sensor noise | Random fluctuation | Sensor output varies even when environment is unchanged |
| Sensor aliasing | Ambiguous reading | Different locations produce identical sensor readings |
| Effector noise | Motion error | Actual robot motion differs from commanded motion |
| Odometry | Wheel-based positioning | Estimates displacement by integrating wheel encoder counts |
| Dead reckoning | History-based positioning | Position from known start + accumulated motion estimates |
| Range error | Distance odometry error | Cumulative error in total distance traveled |
| Turn error | Angular odometry error | Cumulative error in rotation angle |
| Drift error | Heading bias error | Systematic orientation error from unequal wheels; grows quadratically |
| Behavior-based nav | Reactive navigation | Sensor → action rules; no explicit map or position estimate |
| Map-based nav | Deliberative navigation | Explicit localization against a stored map |
| Belief | Probability over poses | Robot's probability distribution over all possible current positions |
| Single-hypothesis | Committed estimate | One best-guess pose; simple but fragile |
| Multiple-hypothesis | Distributed belief | Probability distribution over many candidate poses |
| Prediction phase | Motion update | Proprioceptive data used; uncertainty grows |
| Perception phase | Measurement update | Exteroceptive data used; uncertainty shrinks |
| Local localization | Position tracking | Known approximate start; tracks drift; Kalman-suited |
| Global localization | Pose recovery | Unknown start; uniform prior; Markov/Particle-suited |
| Kidnapped robot | Covert displacement | Robot moved without knowing; must detect and recover |
| SLAM | Build map + localize | Simultaneous estimation of robot path and environment map |
| Relative localization | Internal only | Dead reckoning; drifts over time |
| Absolute localization | External reference | GPS, landmarks; corrects drift |
| Metric | Coordinate-based | Precise (x,y,θ); requires geometric map |
| Topological | Graph-based | Nodes and edges; less precise but computationally lighter |
| Gaussian distribution | N(μ, σ²) | Bell curve; defined by mean (best guess) and variance (uncertainty) |
| Covariance matrix Σ | Uncertainty structure | Diagonal = per-dimension variance; off-diagonal = correlations |
| PDF | Probability density function | Continuous distribution; integrates to 1 |
| Conditional probability | p(x\|y) | Probability of x given y is known |
| Bayes' Rule | Belief update rule | Posterior = Likelihood × Prior / Evidence |
| Prior p(x) | Pre-observation belief | What we believed before the sensor reading |
| Likelihood p(z\|x) | Sensor model | How probable is reading z if truly at pose x |
| Posterior p(x\|z) | Post-observation belief | Updated belief after incorporating sensor reading |
| Markov Assumption | Conditional independence | Current state is a complete summary of history; future ⊥ past given present |
| Bayes Filter | Recursive belief update | Alternates prediction (motion model) and update (sensor model) |
| Motion model | p(xₜ\|uₜ, xₜ₋₁) | Probability of reaching pose xₜ given action uₜ from xₜ₋₁ |
| Sensor model | p(zₜ\|xₜ) | Probability of observation zₜ given robot is at pose xₜ |
| Odometry motion model | Encoder-based motion | Adds Gaussian noise to encoder-measured displacement |
| Beam sensor model | Range sensor model | Gaussian hit + short obstacle + max-range failure + random components |
| Markov Localization | Grid Bayes Filter | Arbitrary PDF over discrete pose grid; handles all localization problems |
| Kalman Filter | Gaussian Bayes Filter | Optimal for linear-Gaussian systems; five equations (predict + update) |
| Predicted mean μ̄ | KF predict step 1 | μ̄ₜ = Aμₜ₋₁ + Buₜ |
| Predicted covariance Σ̄ | KF predict step 2 | Σ̄ₜ = AΣₜ₋₁Aᵀ + R |
| Kalman Gain K | KF trust weight | K = Σ̄Cᵀ(CΣ̄Cᵀ + Q)⁻¹; balances motion model vs. sensor |
| Updated mean μ | KF update step 1 | μₜ = μ̄ₜ + Kₜ(zₜ − Cμ̄ₜ) |
| Updated covariance Σ | KF update step 2 | Σₜ = (I − KₜC)Σ̄ₜ |
| Innovation | Sensor surprise | zₜ − Cμ̄ₜ; difference between actual and predicted observation |
| Process noise R | Motion uncertainty | Covariance of odometry/motion errors |
| Measurement noise Q | Sensor uncertainty | Covariance of sensor reading errors |
| Extended Kalman Filter | Nonlinear KF | Linearizes via Jacobians; enables KF on nonlinear robot systems |
| Jacobian G | Motion Jacobian | ∂g/∂x — linearization of nonlinear motion model |
| Jacobian H | Observation Jacobian | ∂h/∂x — linearization of nonlinear sensor model |
| Linearization | Taylor approximation | First-order approximation of nonlinear function at current estimate |
| SLAM chicken-and-egg | Interdependency | Map needed for localization; position needed for mapping |
| Loop closure | Re-observation event | Re-seeing a known landmark; dramatically reduces accumulated error |
| EKF-SLAM | Gaussian SLAM | Joint state (pose + landmarks); Σ has cross-correlations; O(N²) per step |
| Joint state vector | Combined state | [robot_pose, landmark_1, ..., landmark_N]ᵀ |
| Cross-correlations Σ_rm | Pose-map coupling | Off-diagonal covariance between robot pose and landmark estimates |
| Data association | Feature matching | Determining if current observation matches a previously mapped landmark |
| Particle filter | Sample-based filter | Represents belief as N weighted samples; any distribution; resampling |
| Particle | One hypothesis | One complete state guess (pose + map for SLAM) |
| Weight w^(i) | Particle likelihood | p(z\|x^(i)) — how consistent is this particle with the sensor reading |
| Resampling | Importance sampling | Duplicate high-weight particles; discard low-weight ones |
| FastSLAM | Factored particle SLAM | Per-particle EKF per landmark; O(M log N) complexity |
| FastSLAM factorization | Conditional independence | Given robot path, landmarks are independent → separate per-landmark KFs |
| Particle deprivation | Particle collapse | All particles in wrong region; true state has zero particles |
| Perceptual aliasing | SLAM data association | Two different places produce identical sensor readings → wrong loop closure |
| Monocular scale ambiguity | Bearing-only problem | Single camera cannot recover absolute scale without additional info |

---

*Notes compiled and upgraded from Unit 3 slides of UE23CS343BB7 — Mobile and Autonomous Robots.*
*Dr. Ashok Kumar Patil | Department of Computer Science and Engineering | PES University.*
*Enhanced with full Bayes Filter derivation, all five Kalman Filter equations, EKF Jacobian explanation, FastSLAM algorithm, Particle Filter pseudocode, Markov Assumption formal statement, sensor/motion model depth, and comprehensive analogies throughout.*
