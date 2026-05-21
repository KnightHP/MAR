# UE23CS343BB7 — Mobile and Autonomous Robotics
## Numerical Question Bank — Complete Solved Solutions
### PES University | Units 1 & 2

---

> **How to use this document:** Every solution shows the formula first, explains WHY each step is taken, then works through the numbers. No steps are skipped.

---

# UNIT 1 — KINEMATICS

---

## Core Theory: Homogeneous Transformation Matrix (HTM)

Before solving any problem, understand this foundation — every FK problem uses it.

### What is an HTM?

A **Homogeneous Transformation Matrix** is a 3×3 (for 2D/planar robots) or 4×4 (for 3D robots) matrix that encodes **both rotation AND translation** in a single matrix multiplication. This lets us chain transformations from joint to joint.

### HTM for a Planar Revolute Joint

For joint i with angle θᵢ and subsequent link length Lᵢ (translation along the local x-axis):

```
Step 1 — Rotate about z-axis by θᵢ:
         [cos θᵢ  -sin θᵢ  0]
Rot(z) = [sin θᵢ   cos θᵢ  0]
         [0         0       1]

Step 2 — Translate along x-axis by Lᵢ:
         [1  0  Lᵢ]
Trans  = [0  1   0]
         [0  0   1]

Combined HTM = Rot(z) × Trans:
```

$$H_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & L_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i & L_i\sin\theta_i \\ 0 & 0 & 1 \end{bmatrix}$$

**Reading the HTM:**
- Top-left 2×2 = Rotation submatrix (tells orientation)
- Right column (top 2 entries) = Translation vector (tells position)
- Bottom row [0, 0, 1] = Homogeneous convention

### HTM for a Planar Prismatic Joint

For a prismatic joint with displacement dᵢ (no rotation, just translation):

$$H_i = \begin{bmatrix} 1 & 0 & d_i \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

### Forward Kinematics Formula (from HTMs)

**H_total = H₁ × H₂ × H₃ × ... × Hₙ**

The final position and orientation of the end-effector is read directly from H_total:
- x = H_total[0][2]  (top-right entry)
- y = H_total[1][2]  (middle-right entry)
- θ = θ₁ + θ₂ + ... + θₙ  (sum of all joint angles — for revolute joints)

---

## Q1 & Q2 — 2-Link RR Manipulator: FK + Numerical Evaluation

### Problem Statement
- Robot: Planar 2-link RR (two Revolute joints)
- Link lengths: L₁, L₂
- Joint angles: θ₁ = 35°, θ₂ = 50°, L₁ = 4, L₂ = 3
- Find: End-effector position (x, y) and orientation θ

---

### Step 1 — Derive HTM for Link 1 (Joint 1)

**WHY:** The first joint rotates the arm by θ₁ and the first link extends L₁ from joint 1.

$$H_1 = \begin{bmatrix} \cos\theta_1 & -\sin\theta_1 & L_1\cos\theta_1 \\ \sin\theta_1 & \cos\theta_1 & L_1\sin\theta_1 \\ 0 & 0 & 1 \end{bmatrix}$$

### Step 2 — Derive HTM for Link 2 (Joint 2)

**WHY:** The second joint rotates by θ₂ (relative to link 1's orientation) and the second link extends L₂.

$$H_2 = \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & L_2\cos\theta_2 \\ \sin\theta_2 & \cos\theta_2 & L_2\sin\theta_2 \\ 0 & 0 & 1 \end{bmatrix}$$

### Step 3 — Multiply H_total = H₁ × H₂

**WHY:** Each transformation matrix describes how one frame relates to the previous frame. Multiplying them chains the transformations from base to end-effector.

Computing H₁ × H₂ using matrix multiplication:

**Position (top-right element x):**
```
x = cos θ₁ × (L₂ cos θ₂) + (−sin θ₁) × (L₂ sin θ₂) + L₁ cos θ₁
  = L₂(cos θ₁ cos θ₂ − sin θ₁ sin θ₂) + L₁ cos θ₁
  = L₂ cos(θ₁+θ₂)  + L₁ cos θ₁          [using: cos A cos B − sin A sin B = cos(A+B)]
```

**Position (middle-right element y):**
```
y = sin θ₁ × (L₂ cos θ₂) + cos θ₁ × (L₂ sin θ₂) + L₁ sin θ₁
  = L₂(sin θ₁ cos θ₂ + cos θ₁ sin θ₂) + L₁ sin θ₁
  = L₂ sin(θ₁+θ₂)  + L₁ sin θ₁          [using: sin A cos B + cos A sin B = sin(A+B)]
```

**Rotation (top-left 2×2):**
```
cos(θ₁+θ₂)  −sin(θ₁+θ₂)
sin(θ₁+θ₂)   cos(θ₁+θ₂)
```

**Final H_total:**

$$\boxed{H_{\text{total}} = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) \\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2) \\ 0 & 0 & 1 \end{bmatrix}}$$

**Forward Kinematics Equations (extracted from H_total):**
$$\boxed{x = L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2)}$$
$$\boxed{y = L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)}$$
$$\boxed{\theta = \theta_1 + \theta_2}$$

---

### Step 4 — Substitute Numerical Values

**Given:** θ₁ = 35°, θ₂ = 50°, L₁ = 4, L₂ = 3

**Compute cumulative angle:**
θ₁ + θ₂ = 35° + 50° = **85°**

**Look up trigonometric values:**
| Angle | cos | sin |
|-------|-----|-----|
| 35° | 0.8192 | 0.5736 |
| 85° | 0.0872 | 0.9962 |

**Compute x:**
```
x = L₁ × cos(35°) + L₂ × cos(85°)
  = 4 × 0.8192   + 3 × 0.0872
  = 3.2768        + 0.2616
  = 3.5384
```

**Compute y:**
```
y = L₁ × sin(35°) + L₂ × sin(85°)
  = 4 × 0.5736   + 3 × 0.9962
  = 2.2944        + 2.9886
  = 5.2830
```

**Compute θ:**
```
θ = 35° + 50° = 85°
```

### ✅ Final Answer (Q1 & Q2)

| Parameter | Value |
|-----------|-------|
| x | **3.538 units** |
| y | **5.283 units** |
| θ (orientation) | **85°** |

**Verification using full HTM multiplication:**

$$H_1 = \begin{bmatrix} 0.8192 & -0.5736 & 3.2768 \\ 0.5736 & 0.8192 & 2.2944 \\ 0 & 0 & 1 \end{bmatrix}, \quad H_2 = \begin{bmatrix} 0.6428 & -0.7660 & 1.9284 \\ 0.7660 & 0.6428 & 2.2981 \\ 0 & 0 & 1 \end{bmatrix}$$

$$H_{\text{total}} = H_1 \times H_2 = \begin{bmatrix} 0.0872 & -0.9962 & 3.5384 \\ 0.9962 & 0.0872 & 5.2830 \\ 0 & 0 & 1 \end{bmatrix}$$

Confirmed: x = 3.5384, y = 5.2830, θ = 85° ✓

---

## Q3 — 3-Link RRR Manipulator: Inverse Kinematics

### Problem Statement
- Robot: Planar 3-link RRR (three Revolute joints)
- Target pose: M(x=3.90, y=3.54, φ=90°)
- Link lengths: L₁=3, L₂=2, L₃=1
- Find: Joint angles θ₁, θ₂, θ₃

---

### Core IK Strategy for 3-Link RRR

**The key insight:** For a 3-link planar robot, decouple the problem:
1. Find the **wrist point** (end of link 2, start of link 3) by removing L₃'s contribution
2. Use **2-link IK** to find θ₁ and θ₂ for the wrist point
3. Use the constraint **θ₁ + θ₂ + θ₃ = φ** to find θ₃

---

### Step 1 — Find the Wrist Point

**WHY:** The last link L₃ contributes to the end position. Since we know the final orientation φ and L₃, we can subtract L₃'s effect to find where joint 3 is (the "wrist").

The end-effector position:
```
x_ee = x_wrist + L₃ × cos(φ)
y_ee = y_wrist + L₃ × sin(φ)
```

Solving for wrist position:
```
x_wrist = x_ee − L₃ × cos(φ) = 3.90 − 1 × cos(90°) = 3.90 − 1×0 = 3.90
y_wrist = y_ee − L₃ × sin(φ) = 3.54 − 1 × sin(90°) = 3.54 − 1×1 = 2.54
```

**Wrist point: (3.90, 2.54)**

---

### Step 2 — 2-Link IK for (x_w, y_w) = (3.90, 2.54) with L₁=3, L₂=2

#### Step 2a — Find θ₂ using the Law of Cosines

**WHY:** The triangle formed by link 1, link 2, and the line from origin to wrist point has sides L₁, L₂, and r = distance to wrist. Law of cosines relates these.

Distance from base to wrist:
```
r² = x_w² + y_w² = 3.90² + 2.54² = 15.21 + 6.4516 = 21.6616
r  = √21.6616 = 4.6543
```

Law of cosines:
```
r² = L₁² + L₂² − 2·L₁·L₂·cos(π − θ₂)
r² = L₁² + L₂² + 2·L₁·L₂·cos(θ₂)
```

Rearranging:
$$\cos\theta_2 = \frac{r^2 - L_1^2 - L_2^2}{2 L_1 L_2} = \frac{21.6616 - 9 - 4}{2 \times 3 \times 2} = \frac{8.6616}{12} = 0.7218$$

$$\theta_2 = \arccos(0.7218) = \mathbf{43.79°}$$

(Using elbow-up configuration — positive θ₂)

```
sin(θ₂) = √(1 − cos²θ₂) = √(1 − 0.5210) = √0.4790 = 0.6921
```

#### Step 2b — Find θ₁ using atan2 decomposition

**WHY:** θ₁ is the angle from the x-axis to link 1. It equals the angle to the wrist point, minus the angle that link 2 "steals" by bending.

$$\theta_1 = \underbrace{\arctan\!\left(\frac{y_w}{x_w}\right)}_{\text{angle to wrist}} - \underbrace{\arctan\!\left(\frac{L_2\sin\theta_2}{L_1 + L_2\cos\theta_2}\right)}_{\text{correction for link 2}}$$

Computing each part:
```
Part A: arctan(y_w / x_w) = arctan(2.54 / 3.90) = arctan(0.6513) = 33.09°

Part B numerator:   L₂ × sin(θ₂) = 2 × 0.6921 = 1.3842
Part B denominator: L₁ + L₂ × cos(θ₂) = 3 + 2 × 0.7218 = 3 + 1.4436 = 4.4436
Part B: arctan(1.3842 / 4.4436) = arctan(0.3115) = 17.30°
```

$$\theta_1 = 33.09° - 17.30° = \mathbf{15.79°}$$

#### Step 2c — Find θ₃

**WHY:** The total orientation of the end-effector is the sum of all joint angles. Given φ = 90°:

$$\theta_3 = \phi - \theta_1 - \theta_2 = 90° - 15.79° - 43.79° = \mathbf{30.42°}$$

---

### Verification

```
x = L₁cos(θ₁) + L₂cos(θ₁+θ₂) + L₃cos(θ₁+θ₂+θ₃)
  = 3×cos(15.79°) + 2×cos(59.58°) + 1×cos(90°)
  = 3×0.9631      + 2×0.5075      + 1×0
  = 2.8893        + 1.0150        + 0
  = 3.904 ≈ 3.90 ✓

y = L₁sin(θ₁) + L₂sin(θ₁+θ₂) + L₃sin(θ₁+θ₂+θ₃)
  = 3×sin(15.79°) + 2×sin(59.58°) + 1×sin(90°)
  = 3×0.2721      + 2×0.8617      + 1×1
  = 0.8163        + 1.7234        + 1.0000
  = 3.540 ≈ 3.54 ✓
```

### ✅ Final Answer (Q3)

| Joint | Angle |
|-------|-------|
| θ₁ | **15.79° ≈ 15.8°** |
| θ₂ | **43.79° ≈ 43.8°** |
| θ₃ | **30.42° ≈ 30.4°** |

---

## Q4 — RRP Manipulator: Forward Kinematics

### Problem Statement
- Robot: Planar RRP (Revolute-Revolute-Prismatic)
- Joint params: θ₁=30°, θ₂=45°, d₃=3
- Link lengths: L₁=4, L₂=2
- Find: End-effector position and orientation

---

### Robot Structure Explanation

```
Base → [Joint 1: Revolute θ₁] → Link L₁ → [Joint 2: Revolute θ₂] → Link L₂ → [Joint 3: Prismatic d₃] → End-Effector
```

- Joints 1 & 2 are revolute (rotation) → contribute link lengths via cosine/sine
- Joint 3 is prismatic (linear slide) → extends along the current arm direction by d₃

---

### Step 1 — HTM for Joint 1 (Revolute, angle θ₁, link L₁)

$$H_1 = \begin{bmatrix} \cos\theta_1 & -\sin\theta_1 & L_1\cos\theta_1 \\ \sin\theta_1 & \cos\theta_1 & L_1\sin\theta_1 \\ 0 & 0 & 1 \end{bmatrix}$$

### Step 2 — HTM for Joint 2 (Revolute, angle θ₂, link L₂)

$$H_2 = \begin{bmatrix} \cos\theta_2 & -\sin\theta_2 & L_2\cos\theta_2 \\ \sin\theta_2 & \cos\theta_2 & L_2\sin\theta_2 \\ 0 & 0 & 1 \end{bmatrix}$$

### Step 3 — HTM for Joint 3 (Prismatic, displacement d₃ along local x-axis)

**WHY no rotation?** A prismatic joint only slides — it does not rotate. So the rotation part of the HTM is the identity. The translation is d₃ along the current arm direction.

$$H_3 = \begin{bmatrix} 1 & 0 & d_3 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 3 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

### Step 4 — Compute H₂ × H₃

**WHY multiply H₂ × H₃ first?** Chain from right to left — joints closer to the end-effector are applied first to subsequent links.

Position contribution from H₂ × H₃:
```
x-part: L₂cos(θ₂) + d₃×cos(θ₂) = (L₂ + d₃)×cos(θ₂) = (2+3)×cos(45°) = 5×0.7071 = 3.5355
y-part: L₂sin(θ₂) + d₃×sin(θ₂) = (L₂ + d₃)×sin(θ₂) = (2+3)×sin(45°) = 5×0.7071 = 3.5355
```

**WHY does d₃ add to L₂ along the same direction?** Because the prismatic joint extends *along the local x-axis of frame 2*, which points in direction θ₁+θ₂. So both L₂ and d₃ contribute in the same angular direction.

### Step 5 — Compute H₁ × (H₂ × H₃) → Final x, y

```
cos(30°) = 0.8660, sin(30°) = 0.5000, cos(75°) = 0.2588, sin(75°) = 0.9659
```

**Formula form:**
$$x = L_1\cos\theta_1 + (L_2 + d_3)\cos(\theta_1+\theta_2)$$
$$y = L_1\sin\theta_1 + (L_2 + d_3)\sin(\theta_1+\theta_2)$$
$$\theta = \theta_1 + \theta_2$$

**Numerical computation:**
```
θ₁ + θ₂ = 30° + 45° = 75°

x = 4×cos(30°)  + (2+3)×cos(75°)
  = 4×0.8660    + 5×0.2588
  = 3.4641      + 1.2941
  = 4.7582

y = 4×sin(30°)  + (2+3)×sin(75°)
  = 4×0.5000    + 5×0.9659
  = 2.0000      + 4.8296
  = 6.8296

θ = 30° + 45° = 75°
```

### ✅ Final Answer (Q4)

| Parameter | Value |
|-----------|-------|
| x | **4.758 units** |
| y | **6.830 units** |
| θ (orientation) | **75°** |

---

## Q5 — RPR Manipulator: Inverse Kinematics

### Problem Statement
- Robot: Planar RPR (Revolute-Prismatic-Revolute)
- Target pose: P(x=4.8, y=−2.5, φ=140°)
- Find: θ₁, d₂, θ₃

---

### Robot Structure

```
Base → [Joint 1: Revolute θ₁] → [Joint 2: Prismatic d₂ — slides along direction θ₁] → [Joint 3: Revolute θ₃] → End-Effector
```

No fixed link lengths — all positions come from θ₁, d₂, and θ₃.

### Forward Kinematics of RPR (to understand the inverse)

The prismatic joint d₂ extends along the direction determined by θ₁:
```
x = d₂ × cos(θ₁)
y = d₂ × sin(θ₁)
θ_ee = θ₁ + θ₃
```

### Step 1 — Find d₂

**WHY:** d₂ is the distance from the base to joint 3. Since the prismatic joint extends from the origin in direction θ₁, d₂ is simply the Euclidean distance to the end position (before the last rotation).

$$d_2 = \sqrt{x^2 + y^2} = \sqrt{4.8^2 + (-2.5)^2} = \sqrt{23.04 + 6.25} = \sqrt{29.29} = \mathbf{5.412}$$

### Step 2 — Find θ₁

**WHY:** θ₁ is the angle from the x-axis to the line from origin to the point (x, y). This is directly the four-quadrant arctangent.

$$\theta_1 = \text{atan2}(y, x) = \text{atan2}(-2.5, 4.8)$$

Since x > 0 and y < 0 → angle is in the **4th quadrant** (negative angle):

$$\theta_1 = \arctan\!\left(\frac{-2.5}{4.8}\right) = \arctan(-0.5208) = \mathbf{-27.47°}$$

### Step 3 — Find θ₃

**WHY:** The total end-effector orientation is the sum of all joint angle contributions. For RPR: θ₁ + θ₃ = φ.

$$\theta_3 = \phi - \theta_1 = 140° - (-27.47°) = \mathbf{167.47°}$$

### Verification

```
x = d₂ × cos(θ₁) = 5.412 × cos(-27.47°) = 5.412 × 0.8874 = 4.802 ≈ 4.80 ✓
y = d₂ × sin(θ₁) = 5.412 × sin(-27.47°) = 5.412 × (-0.4613) = -2.497 ≈ -2.50 ✓
θ = θ₁ + θ₃ = -27.47° + 167.47° = 140° ✓
```

### ✅ Final Answer (Q5)

| Parameter | Value |
|-----------|-------|
| θ₁ | **−27.47° ≈ −27.5°** |
| d₂ | **5.412 units** |
| θ₃ | **167.47° ≈ 167.5°** |

---

## Q6 — 3-Link RRR Forward Kinematics: 5 Cases

### Formula

For a 3-link RRR planar robot:

$$x = L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) + L_3\cos(\theta_1+\theta_2+\theta_3)$$
$$y = L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2) + L_3\sin(\theta_1+\theta_2+\theta_3)$$

**WHY:** Each link contributes a vector in the direction of the *cumulative* angle up to that joint. Link 1 points at θ₁, link 2 points at θ₁+θ₂ (relative to base), link 3 points at θ₁+θ₂+θ₃.

---

### Case 1: L₁=5, L₂=5, L₃=5 | θ₁=45°, θ₂=0°, θ₃=0°

**Cumulative angles:**
- After joint 1: 45°
- After joint 2: 45°+0°= 45°
- After joint 3: 45°+0°+0° = 45°

**WHY all the same?** θ₂=0 and θ₃=0 mean joints 2 and 3 don't bend — the entire arm is a straight line at 45°.

```
x = 5×cos45° + 5×cos45° + 5×cos45° = 15×cos45° = 15×0.7071 = 10.607
y = 5×sin45° + 5×sin45° + 5×sin45° = 15×sin45° = 15×0.7071 = 10.607
```

**✅ Result: x = 10.607, y = 10.607**

---

### Case 2: L₁=10, L₂=8, L₃=4 | θ₁=90°, θ₂=−90°, θ₃=90°

**Cumulative angles:**
- After joint 1: 90°
- After joint 2: 90°+(−90°) = 0°
- After joint 3: 0°+90° = 90°

**WHY interesting?** Link 1 goes straight up (90°). Joint 2 bends back 90° so link 2 points right (0°). Joint 3 bends up again 90° so link 3 points up (90°). The arm makes a U-shape.

```
x = 10×cos90° + 8×cos0°  + 4×cos90°
  = 10×0      + 8×1      + 4×0
  = 0         + 8        + 0
  = 8.000

y = 10×sin90° + 8×sin0°  + 4×sin90°
  = 10×1      + 8×0      + 4×1
  = 10        + 0        + 4
  = 14.000
```

**✅ Result: x = 8.000, y = 14.000**

---

### Case 3: L₁=2, L₂=6, L₃=2 | θ₁=60°, θ₂=30°, θ₃=30°

**Cumulative angles:**
- After joint 1: 60°
- After joint 2: 60°+30° = 90°
- After joint 3: 90°+30° = 120°

**Key values:**

| Angle | cos | sin |
|-------|-----|-----|
| 60° | 0.5000 | 0.8660 |
| 90° | 0.0000 | 1.0000 |
| 120° | −0.5000 | 0.8660 |

```
x = 2×cos60°  + 6×cos90° + 2×cos120°
  = 2×0.5     + 6×0      + 2×(−0.5)
  = 1.000     + 0        − 1.000
  = 0.000

y = 2×sin60°  + 6×sin90° + 2×sin120°
  = 2×0.8660  + 6×1      + 2×0.8660
  = 1.732     + 6.000    + 1.732
  = 9.464
```

**WHY x = 0?** The robot is symmetric about the y-axis at this configuration. Link 1 goes right-and-up; link 3 goes left-and-up in equal amounts — canceling the x components.

**✅ Result: x = 0.000, y = 9.464**

---

### Case 4: L₁=4, L₂=3, L₃=5 | θ₁=0°, θ₂=45°, θ₃=−45°

**Cumulative angles:**
- After joint 1: 0°
- After joint 2: 0°+45° = 45°
- After joint 3: 45°+(−45°) = 0°

**WHY interesting?** Links 1 and 3 both point at 0° (along x-axis); only link 2 is angled up. The arm makes a "tent" shape, then comes back down to horizontal.

```
x = 4×cos0°  + 3×cos45°  + 5×cos0°
  = 4×1      + 3×0.7071  + 5×1
  = 4.000    + 2.121     + 5.000
  = 11.121

y = 4×sin0°  + 3×sin45°  + 5×sin0°
  = 4×0      + 3×0.7071  + 5×0
  = 0        + 2.121     + 0
  = 2.121
```

**✅ Result: x = 11.121, y = 2.121**

---

### Case 5: L₁=7, L₂=7, L₃=2 | θ₁=120°, θ₂=−30°, θ₃=−60°

**Cumulative angles:**
- After joint 1: 120°
- After joint 2: 120°+(−30°) = 90°
- After joint 3: 90°+(−60°) = 30°

**Key values:**

| Angle | cos | sin |
|-------|-----|-----|
| 120° | −0.5000 | 0.8660 |
| 90° | 0.0000 | 1.0000 |
| 30° | 0.8660 | 0.5000 |

```
x = 7×cos120°  + 7×cos90°  + 2×cos30°
  = 7×(−0.5)  + 7×0       + 2×0.8660
  = −3.500    + 0          + 1.732
  = −1.768

y = 7×sin120°  + 7×sin90°  + 2×sin30°
  = 7×0.8660  + 7×1       + 2×0.5
  = 6.062     + 7.000     + 1.000
  = 14.062
```

**WHY x is negative?** Link 1 angles up-left (120° from x-axis), strongly pulling x negative. Links 2 and 3 partially compensate but not fully.

**✅ Result: x = −1.768, y = 14.062**

---

### Q6 Summary Table

| Case | L₁,L₂,L₃ | θ₁,θ₂,θ₃ | Cumulative Angles | x | y |
|------|-----------|-----------|-------------------|---|---|
| 1 | 5,5,5 | 45°,0°,0° | 45°,45°,45° | **10.607** | **10.607** |
| 2 | 10,8,4 | 90°,−90°,90° | 90°,0°,90° | **8.000** | **14.000** |
| 3 | 2,6,2 | 60°,30°,30° | 60°,90°,120° | **0.000** | **9.464** |
| 4 | 4,3,5 | 0°,45°,−45° | 0°,45°,0° | **11.121** | **2.121** |
| 5 | 7,7,2 | 120°,−30°,−60° | 120°,90°,30° | **−1.768** | **14.062** |

---

# UNIT 2 — FEATURE EXTRACTION

---

## Core Theory: Moravec Corner Detection

### What Are We Computing?

The Moravec detector checks whether a pixel is a **corner** by measuring how much the image content changes when we slide a small window in different directions.

**Three types of regions:**
- **Flat region:** Window looks similar in ALL directions → all SSD values small
- **Edge:** Window changes a lot in ONE direction, not in the perpendicular → some SSD large, some small
- **Corner:** Window changes significantly in EVERY direction → all SSD values large

### SSD Formula

$$\text{SSD}_d = \sum_{i,j \in W} \left(I_{\text{reference}}(i,j) - I_{\text{shifted}}(i,j)\right)^2$$

Where:
- Reference window W = 3×3 block centered on the target pixel
- Shifted window = the same block shifted by 1 pixel in direction d
- Sum is over all 9 pixels in the window

### Decision Rule

```
C = min(SSD₁, SSD₂, ..., SSD₈)   ← minimum across all 8 directions

If C > threshold → CORNER
If C ≤ threshold → NOT a corner (edge or flat region)
```

**To distinguish edge from flat (when not a corner):**
- Large variation in SSDs (some very high, some low) → **Edge**
- All SSDs are small → **Flat region**

---

## U2-Q1 — 5×5 Matrix: Center at Pixel 0

### Given Matrix (rows 0–4, cols 0–4)

```
     col0  col1  col2  col3  col4
row0: 18     2    22     7     2
row1: 12     9     1    27    29
row2: 23     6     0     9    19
row3:  9    18    30    16    28
row4: 13    17    30    11     9
```

**Pixel value 0 is at: (row=2, col=2)**  — this is our target pixel.

---

### Task a — Extract 3×3 Reference Window Centered at (2,2)

**WHY:** The reference window is the 3×3 neighborhood around the pixel we want to classify. We compare this to shifted versions to detect corners.

Window spans rows 1–3, cols 1–3:

```
Reference Window W:
 9   1  27    ← row 1, cols 1-3
 6   0   9    ← row 2, cols 1-3
18  30  16    ← row 3, cols 1-3
```

**Position mapping:**
```
W[0][0]=9   W[0][1]=1   W[0][2]=27
W[1][0]=6   W[1][1]=0   W[1][2]=9
W[2][0]=18  W[2][1]=30  W[2][2]=16
```

---

### Task b — Compute SSD for Each Shift

**HOW TO SHIFT:** Moving the window right means the shifted window's center moves from (2,2) to (2,3). So the shifted window covers rows 1–3, cols 2–4. We then compute (reference_pixel − shifted_pixel)² for each of the 9 pairs.

---

#### Right Shift (center → (2,3), window rows 1–3, cols 2–4)

```
Shifted Window:
 1  27  29    ← row 1, cols 2-4
 0   9  19    ← row 2, cols 2-4
30  16  28    ← row 3, cols 2-4
```

SSD calculation (each pair: Reference − Shifted):

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 1 | 8 | 64 |
| (0,1) | 1 | 27 | −26 | 676 |
| (0,2) | 27 | 29 | −2 | 4 |
| (1,0) | 6 | 0 | 6 | 36 |
| (1,1) | 0 | 9 | −9 | 81 |
| (1,2) | 9 | 19 | −10 | 100 |
| (2,0) | 18 | 30 | −12 | 144 |
| (2,1) | 30 | 16 | 14 | 196 |
| (2,2) | 16 | 28 | −12 | 144 |

**SSD_Right = 64+676+4+36+81+100+144+196+144 = 1445**

---

#### Left Shift (center → (2,1), window rows 1–3, cols 0–2)

```
Shifted Window:
12   9   1    ← row 1, cols 0-2
23   6   0    ← row 2, cols 0-2
 9  18  30    ← row 3, cols 0-2
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 12 | −3 | 9 |
| (0,1) | 1 | 9 | −8 | 64 |
| (0,2) | 27 | 1 | 26 | 676 |
| (1,0) | 6 | 23 | −17 | 289 |
| (1,1) | 0 | 6 | −6 | 36 |
| (1,2) | 9 | 0 | 9 | 81 |
| (2,0) | 18 | 9 | 9 | 81 |
| (2,1) | 30 | 18 | 12 | 144 |
| (2,2) | 16 | 30 | −14 | 196 |

**SSD_Left = 9+64+676+289+36+81+81+144+196 = 1576**

---

#### Up Shift (center → (1,2), window rows 0–2, cols 1–3)

```
Shifted Window:
 2  22   7    ← row 0, cols 1-3
 9   1  27    ← row 1, cols 1-3
 6   0   9    ← row 2, cols 1-3
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 2 | 7 | 49 |
| (0,1) | 1 | 22 | −21 | 441 |
| (0,2) | 27 | 7 | 20 | 400 |
| (1,0) | 6 | 9 | −3 | 9 |
| (1,1) | 0 | 1 | −1 | 1 |
| (1,2) | 9 | 27 | −18 | 324 |
| (2,0) | 18 | 6 | 12 | 144 |
| (2,1) | 30 | 0 | 30 | 900 |
| (2,2) | 16 | 9 | 7 | 49 |

**SSD_Up = 49+441+400+9+1+324+144+900+49 = 2317**

---

#### Down Shift (center → (3,2), window rows 2–4, cols 1–3)

```
Shifted Window:
 6   0   9    ← row 2, cols 1-3
18  30  16    ← row 3, cols 1-3
17  30  11    ← row 4, cols 1-3
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 6 | 3 | 9 |
| (0,1) | 1 | 0 | 1 | 1 |
| (0,2) | 27 | 9 | 18 | 324 |
| (1,0) | 6 | 18 | −12 | 144 |
| (1,1) | 0 | 30 | −30 | 900 |
| (1,2) | 9 | 16 | −7 | 49 |
| (2,0) | 18 | 17 | 1 | 1 |
| (2,1) | 30 | 30 | 0 | 0 |
| (2,2) | 16 | 11 | 5 | 25 |

**SSD_Down = 9+1+324+144+900+49+1+0+25 = 1453**

---

#### Up-Left Diagonal Shift (center → (1,1), window rows 0–2, cols 0–2)

```
Shifted Window:
18   2  22    ← row 0, cols 0-2
12   9   1    ← row 1, cols 0-2
23   6   0    ← row 2, cols 0-2
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 18 | −9 | 81 |
| (0,1) | 1 | 2 | −1 | 1 |
| (0,2) | 27 | 22 | 5 | 25 |
| (1,0) | 6 | 12 | −6 | 36 |
| (1,1) | 0 | 9 | −9 | 81 |
| (1,2) | 9 | 1 | 8 | 64 |
| (2,0) | 18 | 23 | −5 | 25 |
| (2,1) | 30 | 6 | 24 | 576 |
| (2,2) | 16 | 0 | 16 | 256 |

**SSD_Up-Left = 81+1+25+36+81+64+25+576+256 = 1145**

---

#### Up-Right Diagonal Shift (center → (1,3), window rows 0–2, cols 2–4)

```
Shifted Window:
22   7   2    ← row 0, cols 2-4
 1  27  29    ← row 1, cols 2-4
 0   9  19    ← row 2, cols 2-4
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 22 | −13 | 169 |
| (0,1) | 1 | 7 | −6 | 36 |
| (0,2) | 27 | 2 | 25 | 625 |
| (1,0) | 6 | 1 | 5 | 25 |
| (1,1) | 0 | 27 | −27 | 729 |
| (1,2) | 9 | 29 | −20 | 400 |
| (2,0) | 18 | 0 | 18 | 324 |
| (2,1) | 30 | 9 | 21 | 441 |
| (2,2) | 16 | 19 | −3 | 9 |

**SSD_Up-Right = 169+36+625+25+729+400+324+441+9 = 2758**

---

#### Down-Left Diagonal Shift (center → (3,1), window rows 2–4, cols 0–2)

```
Shifted Window:
23   6   0    ← row 2, cols 0-2
 9  18  30    ← row 3, cols 0-2
13  17  30    ← row 4, cols 0-2
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 23 | −14 | 196 |
| (0,1) | 1 | 6 | −5 | 25 |
| (0,2) | 27 | 0 | 27 | 729 |
| (1,0) | 6 | 9 | −3 | 9 |
| (1,1) | 0 | 18 | −18 | 324 |
| (1,2) | 9 | 30 | −21 | 441 |
| (2,0) | 18 | 13 | 5 | 25 |
| (2,1) | 30 | 17 | 13 | 169 |
| (2,2) | 16 | 30 | −14 | 196 |

**SSD_Down-Left = 196+25+729+9+324+441+25+169+196 = 2114**

---

#### Down-Right Diagonal Shift (center → (3,3), window rows 2–4, cols 2–4)

```
Shifted Window:
 0   9  19    ← row 2, cols 2-4
30  16  28    ← row 3, cols 2-4
30  11   9    ← row 4, cols 2-4
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 9 | 0 | 9 | 81 |
| (0,1) | 1 | 9 | −8 | 64 |
| (0,2) | 27 | 19 | 8 | 64 |
| (1,0) | 6 | 30 | −24 | 576 |
| (1,1) | 0 | 16 | −16 | 256 |
| (1,2) | 9 | 28 | −19 | 361 |
| (2,0) | 18 | 30 | −12 | 144 |
| (2,1) | 30 | 11 | 19 | 361 |
| (2,2) | 16 | 9 | 7 | 49 |

**SSD_Down-Right = 81+64+64+576+256+361+144+361+49 = 1956**

---

### Task c — Determine Minimum SSD

| Direction | SSD |
|-----------|-----|
| Right | 1445 |
| Left | 1576 |
| **Up** | **2317** |
| Down | 1453 |
| **Up-Left** | **1145** ← MINIMUM |
| Up-Right | 2758 |
| Down-Left | 2114 |
| Down-Right | 1956 |

$$\boxed{C = \min(\text{all SSDs}) = 1145 \text{ (Up-Left direction)}}$$

---

### Task d — Classification with Threshold = 1400

$$C = 1145 < \text{threshold } (1400)$$

**Decision: C < threshold → NOT a corner**

**Further classification:**
- Large variation in SSD values (min=1145, max=2758)
- Some directions show large change (Up-Right=2758) while others are smaller (Up-Left=1145)
- Large directional variation → **EDGE**

$$\boxed{\text{Classification: EDGE (Not a Corner)}}$$

*The pixel at value 0 is at the edge of a significant intensity transition — high values above/right (27, 29), low values at center (0), high below (30). The directional asymmetry in SSDs confirms an edge structure.*

---

## U2-Q2 — 5×5 Symmetric Matrix: Center at Pixel 35

### Given Matrix

```
     col0  col1  col2  col3  col4
row0: 15    16    18    16    15
row1: 16    22    25    22    16
row2: 18    25    35    25    18
row3: 16    22    25    22    16
row4: 15    16    18    16    15
```

**Pixel 35 is at: (row=2, col=2)** — center of the matrix.

---

### Task a — Extract 3×3 Reference Window Centered at (2,2)

Window spans rows 1–3, cols 1–3:

```
Reference Window W:
22  25  22    ← row 1, cols 1-3
25  35  25    ← row 2, cols 1-3
22  25  22    ← row 3, cols 1-3
```

*Notice: W is symmetric — same value in each diagonal pair.*

---

### Task b — Compute SSD for Horizontal and Vertical Shifts

#### Right Shift (center → (2,3), window rows 1–3, cols 2–4)

```
Shifted Window:
25  22  16
35  25  18
25  22  16
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 22 | 25 | −3 | 9 |
| (0,1) | 25 | 22 | 3 | 9 |
| (0,2) | 22 | 16 | 6 | 36 |
| (1,0) | 25 | 35 | −10 | 100 |
| (1,1) | 35 | 25 | 10 | 100 |
| (1,2) | 25 | 18 | 7 | 49 |
| (2,0) | 22 | 25 | −3 | 9 |
| (2,1) | 25 | 22 | 3 | 9 |
| (2,2) | 22 | 16 | 6 | 36 |

**SSD_Right = 9+9+36+100+100+49+9+9+36 = 357**

---

#### Left Shift (center → (2,1), window rows 1–3, cols 0–2)

```
Shifted Window:
16  22  25
18  25  35
16  22  25
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 22 | 16 | 6 | 36 |
| (0,1) | 25 | 22 | 3 | 9 |
| (0,2) | 22 | 25 | −3 | 9 |
| (1,0) | 25 | 18 | 7 | 49 |
| (1,1) | 35 | 25 | 10 | 100 |
| (1,2) | 25 | 35 | −10 | 100 |
| (2,0) | 22 | 16 | 6 | 36 |
| (2,1) | 25 | 22 | 3 | 9 |
| (2,2) | 22 | 25 | −3 | 9 |

**SSD_Left = 36+9+9+49+100+100+36+9+9 = 357**

---

#### Up Shift (center → (1,2), window rows 0–2, cols 1–3)

```
Shifted Window:
16  18  16
22  25  22
25  35  25
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 22 | 16 | 6 | 36 |
| (0,1) | 25 | 18 | 7 | 49 |
| (0,2) | 22 | 16 | 6 | 36 |
| (1,0) | 25 | 22 | 3 | 9 |
| (1,1) | 35 | 25 | 10 | 100 |
| (1,2) | 25 | 22 | 3 | 9 |
| (2,0) | 22 | 25 | −3 | 9 |
| (2,1) | 25 | 35 | −10 | 100 |
| (2,2) | 22 | 25 | −3 | 9 |

**SSD_Up = 36+49+36+9+100+9+9+100+9 = 357**

---

#### Down Shift (center → (3,2), window rows 2–4, cols 1–3)

```
Shifted Window:
25  35  25
22  25  22
16  18  16
```

| Pair | Ref | Shift | Diff | Diff² |
|------|-----|-------|------|-------|
| (0,0) | 22 | 25 | −3 | 9 |
| (0,1) | 25 | 35 | −10 | 100 |
| (0,2) | 22 | 25 | −3 | 9 |
| (1,0) | 25 | 22 | 3 | 9 |
| (1,1) | 35 | 25 | 10 | 100 |
| (1,2) | 25 | 22 | 3 | 9 |
| (2,0) | 22 | 16 | 6 | 36 |
| (2,1) | 25 | 18 | 7 | 49 |
| (2,2) | 22 | 16 | 6 | 36 |

**SSD_Down = 9+100+9+9+100+9+36+49+36 = 357**

---

### Task c — Moravec Response Value (Minimum SSD)

| Direction | SSD |
|-----------|-----|
| Right | 357 |
| Left | 357 |
| Up | 357 |
| Down | 357 |

**WHY are all SSDs identical?** The matrix is perfectly symmetric (radially). When you slide the window in any direction, the change in values is exactly the same by symmetry. Every shift encounters the same "slope" of intensity change.

$$\boxed{C = \min(\text{SSDs}) = 357}$$

---

### Task d — Classification with Threshold = 100

$$C = 357 > \text{threshold } (100)$$

**Decision: C > threshold → CORNER** (by Moravec criterion)

**Important note on Moravec's limitation:**
This result reveals a well-known weakness of the Moravec detector. The pixel at value 35 is the **center of a symmetric Gaussian-like blob** — not a geometric corner. The high SSD in all directions is caused by the strong central peak, not by two edges meeting at a point. The Moravec detector falsely classifies blob centers as corners. This is one of the main motivations for the Harris detector, which uses eigenvalue analysis and is not fooled by symmetric blobs.

$$\boxed{\text{Classification: CORNER (by Moravec) — but actually a symmetric blob — known limitation}}$$

---

## U2-Q3 — Harris Corner Detection on Symmetric Matrix

### Given Matrix

```
     col0  col1  col2  col3  col4
row0: 15    18    20    18    15
row1: 18    25    30    25    18
row2: 20    30    40    30    20
row3: 18    25    30    25    18
row4: 15    18    20    18    15
```

**Center pixel = 40 at (row=2, col=2)**

---

### Core Theory: Harris Detection Method

**The Harris detector uses image gradients** (partial derivatives of intensity), not finite window shifts like Moravec. This makes it:
- Rotation-invariant (tests all directions, not just 8)
- Less sensitive to noise (gradients are smoother)

**Gradient formula (central difference):**
$$I_x = \frac{\partial I}{\partial x} \approx I(\text{row, col+1}) - I(\text{row, col-1})$$
$$I_y = \frac{\partial I}{\partial y} \approx I(\text{row+1, col}) - I(\text{row-1, col})$$

**WHY central difference?** It approximates the derivative at the center pixel by looking one step on each side, giving a symmetric, unbiased estimate of the local slope.

**Harris Matrix (Second Moment Matrix / Structure Tensor):**
$$M = \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix}$$

**Harris Corner Response:**
$$R = \det(M) - k \cdot [\text{trace}(M)]^2 \quad \text{where } k \approx 0.04$$

**Interpretation of R:**
- R >> 0 (large positive) → **CORNER** (both eigenvalues large)
- R < 0 → **EDGE** (one large eigenvalue, one small)
- |R| ≈ 0 → **FLAT REGION** (both eigenvalues small)

---

### Task a — Compute Iₓ and I_y at Center Pixel (2,2)

**Reading values from the matrix:**
- Left of center: I(2,1) = 30
- Right of center: I(2,3) = 30
- Above center: I(1,2) = 30
- Below center: I(3,2) = 30

$$I_x = I(2,3) - I(2,1) = 30 - 30 = \mathbf{0}$$
$$I_y = I(3,2) - I(1,2) = 30 - 30 = \mathbf{0}$$

**WHY are both zero?** The matrix is perfectly symmetric. The pixel at value 40 is the global maximum. At any maximum (peak), the gradient is zero — it's flat at the top of the "hill," like the tip of a mountain where the slope is zero.

---

### Task b — Compute Harris Matrix M

$$I_x^2 = 0^2 = 0$$
$$I_y^2 = 0^2 = 0$$
$$I_x I_y = 0 \times 0 = 0$$

$$M = \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix} = \begin{bmatrix} 0 & 0 \\ 0 & 0 \end{bmatrix}$$

---

### Task c — Compute Harris Response R (k = 0.04)

$$\det(M) = (0)(0) - (0)(0) = 0$$
$$\text{trace}(M) = 0 + 0 = 0$$
$$R = \det(M) - k \cdot [\text{trace}(M)]^2 = 0 - 0.04 \times 0^2 = \mathbf{0}$$

---

### Task d — Classification

$$R = 0 \approx 0 \Rightarrow \text{neither large positive nor negative}$$

$$\boxed{\text{Classification: FLAT REGION}}$$

**Explanation:** The Harris detector correctly identifies this as a flat region at the single-pixel level. Even though the symmetric matrix has strong intensity variation around the center, the center pixel itself sits at a zero-gradient point (it's the peak of a symmetric "hill"). Both eigenvalues of M are zero → no directional preference → flat.

**Contrast with Moravec:** Moravec (Q2) classified a similar symmetric blob as a CORNER (incorrectly), while Harris correctly identifies the center as FLAT. This demonstrates Harris's superior discrimination ability — it correctly doesn't confuse a symmetric blob peak with a genuine image corner.

---

## U2-Q4 — Harris Corner Detection on Asymmetric Matrix

### Given Matrix

```
     col0  col1  col2  col3  col4
row0:  8    10    12    10     8
row1: 10    15    20    15    10
row2: 12    20    30    25    12
row3: 10    15    25    20    10
row4:  8    10    12    10     8
```

**Center pixel = 30 at (row=2, col=2)**

**Key observation:** This matrix is NOT perfectly symmetric — the value at (2,3) is 25 while (2,1) is 20, and (3,2) is 25 while (1,2) is 20. There is an asymmetry in both directions.

---

### Task a — Compute Iₓ and I_y at Center Pixel (2,2)

**Reading surrounding values:**
- Left: I(2,1) = 20
- Right: I(2,3) = 25
- Above: I(1,2) = 20
- Below: I(3,2) = 25

$$I_x = I(2,3) - I(2,1) = 25 - 20 = \mathbf{5}$$
$$I_y = I(3,2) - I(1,2) = 25 - 20 = \mathbf{5}$$

**WHY non-zero?** Unlike Q3's symmetric matrix, this matrix has higher values to the right and below the center than to the left and above. This asymmetry creates non-zero gradients — there is a "slope" in both the x and y directions.

$$\boxed{I_x = 5, \quad I_y = 5}$$

---

### Task b — Compute Iₓ², I_y², and Iₓ I_y

$$I_x^2 = 5^2 = \mathbf{25}$$
$$I_y^2 = 5^2 = \mathbf{25}$$
$$I_x I_y = 5 \times 5 = \mathbf{25}$$

---

### Task c — Construct the Structure Tensor Matrix M

$$M = \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix} = \begin{bmatrix} 25 & 25 \\ 25 & 25 \end{bmatrix}$$

**WHY does the off-diagonal element equal the diagonal?** Because Iₓ = Iy = 5, so Iₓ² = Iy² = IₓIy = 25. All three are equal. This creates a rank-1 matrix — it has only ONE non-zero eigenvalue.

---

### Task d — Compute Harris Response R (k = 0.05)

**Step 1 — Compute determinant:**
$$\det(M) = I_x^2 \cdot I_y^2 - (I_x I_y)^2 = 25 \times 25 - 25^2 = 625 - 625 = \mathbf{0}$$

**WHY det = 0?** When Iₓ = Iy = 5, the matrix M has rows [25,25] and [25,25] — one row is a multiple of the other. This means the matrix is singular (rank 1), which always gives det = 0.

**Step 2 — Compute trace:**
$$\text{trace}(M) = I_x^2 + I_y^2 = 25 + 25 = \mathbf{50}$$

**Step 3 — Compute R:**
$$R = \det(M) - k \cdot [\text{trace}(M)]^2$$
$$R = 0 - 0.05 \times (50)^2$$
$$R = 0 - 0.05 \times 2500$$
$$R = \mathbf{-125}$$

---

### Task e — Classification (Threshold R > 400)

- Is R > 400? → **NO** (R = −125 < 400) → Not a corner
- Is R < 0? → **YES** → **EDGE**

$$\boxed{\text{R = -125} < 0 \Rightarrow \text{EDGE}}$$

**Physical interpretation:** The eigenvalues of M can be inferred from det and trace:
- λ₁ × λ₂ = det(M) = 0 → one eigenvalue is 0
- λ₁ + λ₂ = trace(M) = 50 → the other eigenvalue is 50

So the eigenvalues are λ₁ = 50 and λ₂ = 0.

| Condition | Meaning |
|-----------|---------|
| λ₁ large, λ₂ ≈ 0 | Strong gradient in ONE direction only |
| This gives R < 0 | **EDGE** |

The center pixel is on an edge that runs diagonally (both Iₓ and Iy are equal and non-zero, meaning the edge is at 45° to both axes). The gradient is entirely in the direction (1,1) and zero in the perpendicular direction (1,−1).

---

## Summary of All Unit 2 Answers

| Question | Matrix | Center Pixel | Method | Result |
|----------|--------|-------------|--------|--------|
| Q1 | Irregular 5×5 | Value = 0, position (2,2) | Moravec (8 directions) | Min SSD=1145 < threshold 1400 → **EDGE** |
| Q2 | Symmetric 5×5 | Value = 35, position (2,2) | Moravec (H+V only) | Min SSD=357 > threshold 100 → **CORNER** (blob artifact) |
| Q3 | Symmetric 5×5 | Value = 40, position (2,2) | Harris (k=0.04) | R=0 → **FLAT REGION** |
| Q4 | Asymmetric 5×5 | Value = 30, position (2,2) | Harris (k=0.05) | R=−125 < 0 → **EDGE** |

---

## Key Formulas Reference Card

### Unit 1 — Kinematics

| Formula | Use |
|---------|-----|
| Hᵢ = [[cosθᵢ, −sinθᵢ, Lᵢcosθᵢ], [sinθᵢ, cosθᵢ, Lᵢsinθᵢ], [0,0,1]] | HTM for revolute joint i |
| x = Σ Lᵢ cos(cumulative angle up to i) | FK x-position (all RR) |
| y = Σ Lᵢ sin(cumulative angle up to i) | FK y-position (all RR) |
| θ = θ₁ + θ₂ + ... + θₙ | End-effector orientation |
| cos(θ₂) = (r² − L₁² − L₂²) / (2L₁L₂) | 2-link IK for θ₂ |
| θ₁ = atan2(y,x) − atan2(L₂sinθ₂, L₁+L₂cosθ₂) | 2-link IK for θ₁ |
| x_wrist = x − L₃cos(φ) | 3-link IK wrist decoupling |
| θ₃ = φ − θ₁ − θ₂ | 3-link IK for θ₃ |

### Unit 2 — Feature Detection

| Formula | Use |
|---------|-----|
| SSD = Σ(Ref − Shifted)² | Moravec response per direction |
| C = min(all SSDs) | Moravec corner score |
| C > threshold → Corner | Moravec decision rule |
| Iₓ = I(row, col+1) − I(row, col−1) | Horizontal gradient |
| Iy = I(row+1, col) − I(row−1, col) | Vertical gradient |
| M = [[Iₓ², IₓIy], [IₓIy, Iy²]] | Harris structure tensor |
| R = det(M) − k × trace(M)² | Harris corner response |
| R >> 0 → Corner | Harris: both λ large |
| R < 0 → Edge | Harris: λ₁ ≫ λ₂ |
| R ≈ 0 → Flat | Harris: both λ small |
