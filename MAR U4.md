# Mobile and Autonomous Robotics — Unit 4: Navigation
### Complete Notes | Course Code: UE23CS343BB7 | PES University

---

> **How to use these notes:** Every concept is explained from first principles with full definitions, analogies, mathematics, and practical examples. These notes are slide-independent and completely self-contained.

---

## Table of Contents

1. [Introduction to Navigation](#1-introduction-to-navigation)
2. [Path Planning — Overview and Strategies](#2-path-planning--overview-and-strategies)
3. [Graph Construction Methods](#3-graph-construction-methods)
   - 3.1 [Visibility Graphs](#31-visibility-graphs)
   - 3.2 [Voronoi Diagrams](#32-voronoi-diagrams)
   - 3.3 [Exact Cell Decomposition](#33-exact-cell-decomposition)
   - 3.4 [Approximate Cell Decomposition](#34-approximate-cell-decomposition)
   - 3.5 [Lattice Graph Search](#35-lattice-graph-search)
4. [Deterministic Graph Search Algorithms](#4-deterministic-graph-search-algorithms)
   - 4.1 [Cost Function Terminology](#41-cost-function-terminology)
   - 4.2 [Breadth-First Search (BFS)](#42-breadth-first-search-bfs)
   - 4.3 [Depth-First Search (DFS)](#43-depth-first-search-dfs)
   - 4.4 [Dijkstra's Algorithm](#44-dijkstras-algorithm)
   - 4.5 [A* Algorithm](#45-a-algorithm)
   - 4.6 [D* Algorithm](#46-d-algorithm)
5. [Randomised Graph Search — RRT](#5-randomised-graph-search--rrt)
6. [Potential Field Path Planning](#6-potential-field-path-planning)
7. [Obstacle Avoidance](#7-obstacle-avoidance)
   - 7.1 [Bug Algorithm (Bug1, Bug2, Tangent Bug)](#71-bug-algorithm-bug1-bug2-tangent-bug)
   - 7.2 [Vector Field Histogram (VFH)](#72-vector-field-histogram-vfh)
   - 7.3 [Bubble Band Technique](#73-bubble-band-technique)
   - 7.4 [Curvature Velocity Method (CVM)](#74-curvature-velocity-method-cvm)
   - 7.5 [Dynamic Window Approach (DWA)](#75-dynamic-window-approach-dwa)
   - 7.6 [Other Obstacle Avoidance Approaches](#76-other-obstacle-avoidance-approaches)
8. [Navigation Architectures](#8-navigation-architectures)
   - 8.1 [Why Architecture Matters](#81-why-architecture-matters)
   - 8.2 [Modularity and Control Localisation](#82-modularity-and-control-localisation)
   - 8.3 [Decompositions](#83-decompositions)
   - 8.4 [Temporal Decomposition — Four Trends](#84-temporal-decomposition--four-trends)
   - 8.5 [Control Decomposition — Serial vs. Parallel](#85-control-decomposition--serial-vs-parallel)
   - 8.6 [Case Study: Tiered Robot Architecture](#86-case-study-tiered-robot-architecture)
9. [Reinforcement Learning for Navigation](#9-reinforcement-learning-for-navigation)
   - 9.1 [Core RL Terminology](#91-core-rl-terminology)
   - 9.2 [The RL Learning Process](#92-the-rl-learning-process)
   - 9.3 [Exploration vs. Exploitation](#93-exploration-vs-exploitation)
   - 9.4 [Real-World Example: Mini Cheetah](#94-real-world-example-mini-cheetah)
   - 9.5 [Challenges of RL in Robotics](#95-challenges-of-rl-in-robotics)
   - 9.6 [Applications of RL](#96-applications-of-rl)
   - 9.7 [Social Implications of RL](#97-social-implications-of-rl)
10. [Quick Reference Summary](#10-quick-reference-summary)

---

## 1. Introduction to Navigation

### What is Navigation?

**Navigation** is the "cognitive" level of a mobile robot — the ability to move from a starting point to a goal position *reliably and intelligently*, using a combination of prior knowledge (maps) and real-time sensor data.

Navigation is not just *movement* — it is *purposeful, goal-directed movement* with decision-making. A robot that just drives forward is moving; a robot that plans a route, detects obstacles, reroutes around them, and reaches its destination is navigating.

*Analogy:* The difference between a ball rolling downhill (movement, no decision) and a taxi driver planning a route across a city, adapting to traffic jams and road closures (navigation, with planning and real-time adaptation).

### The Two Core Competences

Navigation requires two distinct, complementary capabilities:

**1. Path Planning (Strategic, Long-term)**
- Identifies a trajectory from current position to goal, based on a *map*
- Operates *before* and *during* movement at a higher level
- Concerned with: "Which route should I take across the city?"
- Computationally expensive; often done with more time available
- Requires global knowledge of the environment

**2. Obstacle Avoidance (Tactical, Real-time)**
- *Modulates* the planned trajectory to prevent immediate collisions
- Operates *continuously* during movement at a lower, faster level
- Concerned with: "There's a person stepping into my path right now — turn!"
- Must be extremely fast (milliseconds)
- Works with recent local sensor data

*Key insight:* These are not opposites — they are complementary. Without a plan, a robot wanders with no direction. Without reactive avoidance, a perfectly planned route will result in a crash if an unexpected obstacle appears.

### Integrated Navigation

The ideal navigation system **merges planning and reaction**:
- It maintains a high-level plan (strategic goal)
- Continuously receives sensor data
- Updates the plan in real-time as new obstacles or changes are detected
- Maintains progress toward the goal while staying safe

### Belief State and Completeness

**Belief State:** A robot can never be 100% certain of its exact physical location. It operates based on a *probabilistic estimate* (belief) of its current state — a probability distribution over possible positions and orientations.

A navigation plan is essentially a trajectory designed to move the robot from its *initial belief state* to a *goal belief state*.

**Completeness:** A navigation system is **complete** if it is *guaranteed to find a solution whenever one exists*. If a valid path exists from start to goal, a complete algorithm will always find it.

In practice, designers often *sacrifice completeness* for faster computation — incomplete algorithms are sometimes acceptable if failures are rare or easily detected.

---

## 2. Path Planning — Overview and Strategies

### Historical Context

Path planning was first intensively studied for **industrial robot manipulators** (robot arms) because:
- Robot arms have multiple joints → high-dimensional configuration space
- Industrial applications demand precision, speed, and throughput
- Both *kinematics* (geometry of motion) and *dynamics* (forces and accelerations) must be considered

For **mobile robots**:
- Fewer degrees of freedom (typically 3: x, y, heading θ)
- Often operate at lower speeds → *kinematics* is usually sufficient; dynamics are less critical
- Problem is simpler → algorithms are generally simpler

### The Two Main Path Planning Strategies

**Strategy 1 — Graph Search:**
- Build a *connectivity graph* of the robot's free space
- Search this graph for the best path from start to goal
- Graph construction is often done *offline* (computed once, stored)
- Graph search is then performed online (during operation)

**Strategy 2 — Potential Field Planning:**
- Define a mathematical *potential energy function* over the entire free space
- The function has a global minimum at the goal and local maxima (peaks) at obstacles
- The robot follows the gradient (downhill direction) of this function — like a ball rolling to the lowest point
- Elegant and simple, but has known failure modes (local minima)

---

## 3. Graph Construction Methods

Before searching for a path, we need to *build the graph* — a representation of where the robot can go and how spaces connect. Multiple methods exist, each with different trade-offs between completeness, optimality, and computational cost.

---

### 3.1 Visibility Graphs

**Concept:** Connect every pair of obstacle vertices that can "see" each other — i.e., the straight line between them passes entirely through free space (no obstacle in between).

**Construction:**
1. Represent all obstacles as *convex polygons* in the configuration space
2. Identify all polygon *vertices* (corners)
3. Add the *start position* and *goal position* as special nodes
4. For every pair of nodes (vertices + start + goal), check if they can see each other — if the straight line between them does not pass through any obstacle
5. Connect all visible pairs with *edges* (roads)
6. The resulting graph is the **visibility graph** — a roadmap of mutually visible vertices

**Path Planning on Visibility Graph:**
Search the visibility graph for the *shortest path* from start to goal (using Dijkstra's or A*). Because the edges are unobstructed straight lines, these paths are *geometrically shortest* — they follow the minimum total distance.

*Analogy:* Imagine a city where buildings are the obstacles and you can only walk in straight lines without entering a building. You'd naturally walk along the edges and corners of buildings to get from A to B. The visibility graph captures exactly these "corner-cutting" routes.

**Why it works:** The shortest path in polygonal environments always travels along straight lines between obstacle vertices — it never needs to curve or travel through interior space (except at convex corners).

**Advantages:**
- Simple to implement when obstacles are already described as polygons
- Finds the *mathematically shortest* path
- Complete — if a path exists, it will be found

**Disadvantages (Two Primary Caveats):**

**1. Computational inefficiency in dense environments:**
The number of edges scales with obstacle count — O(n²) edges for n vertices. In complex environments with many obstacles, the graph becomes enormous.

**2. Safety concern — paths graze obstacles:**
The shortest path literally touches the corners of obstacles. While mathematically optimal in length, this is dangerous in practice:
- Robot dimensions mean the robot's body may collide even if its reference point (usually center) follows the path
- Any localization error could cause the robot to clip an obstacle corner

**Safety Fix:** Inflate obstacle boundaries by the robot's radius before constructing the visibility graph. This creates a safety buffer. However, this inflated path is *no longer the absolute shortest path* — a trade-off between safety and optimality.

**Visual description of the diagram (from slides):**
- Polygonal obstacles (orange rectangle, green triangle, red pentagon) in a 2D space
- Start (top-left) and goal (bottom-right) as special nodes
- Blue lines = all visibility graph edges connecting mutually visible vertices
- Green/highlighted path = the shortest path found through the visibility graph

---

### 3.2 Voronoi Diagrams

**Concept:** Build a roadmap that *maximizes distance from all obstacles at all times* — the opposite safety philosophy from visibility graphs. If visibility graphs hug obstacles, Voronoi roads pass as far as possible from everything.

**Construction:**
1. For every point in free space, compute its *distance to the nearest obstacle*
2. Visualize this distance as a height map — mountains rise near the center of free space between obstacles
3. Find the *ridgelines* — points equidistant from two or more obstacles
4. These ridgelines form the **Voronoi diagram**

Mathematically, the Voronoi diagram consists of all points P where:
> distance(P, obstacle_A) = distance(P, obstacle_B) for some pair of obstacles A, B

For polygonal obstacles, the diagram consists of:
- **Straight line segments** — equidistant from two parallel lines or two line segments
- **Parabolic segments** — equidistant from a line and a point (vertex)

**Path Planning on Voronoi Diagram:**
1. Map the robot's start position onto its nearest Voronoi edge (by following the direction of fastest distance increase to obstacles)
2. Map the goal position similarly
3. Search the Voronoi graph for a path between these mapped positions
4. The result: a path that always stays as far as possible from all obstacles

*Analogy:* If you're hiking in a valley between mountain ranges, you'd naturally walk along the valley floor — equidistant from both mountain slopes. The Voronoi path is the "valley floor" of the robot's free space.

**Advantages:**
- **Complete:** Guaranteed to find a path if one exists
- **Maximum safety:** Robot always maximizes its distance from all obstacles
- **Executable:** Simple control rules suffice (e.g., "maintain equal distance to left and right walls using a range sensor")
- Useful for **map construction** — following Voronoi edges while mapping with range sensors is efficient and well-structured

**Disadvantages:**
- **Path length:** Voronoi paths are typically *much longer* than the shortest path — not optimal in distance
- **Localization challenge:** Maximizing distance from obstacles can make localization harder — you need nearby features to locate yourself, but Voronoi paths keep you in the center of large open spaces
- In sparse environments, Voronoi edges may require traveling enormous detours

**Voronoi vs. Visibility Graph:**

| Property | Visibility Graph | Voronoi Diagram |
|---|---|---|
| Path length | Shortest (optimal) | Longer (far from optimal) |
| Safety | Poor (grazes obstacles) | Maximum (maximizes clearance) |
| Completeness | Yes | Yes |
| Localization | Easier (near features) | Harder (away from features) |
| Control simplicity | Harder (point-to-point) | Easier (follow ridge) |

---

### 3.3 Exact Cell Decomposition

**Concept:** Divide the free space into a finite set of *non-overlapping cells*, where each cell is *entirely free* or *entirely occupied*. Navigation then becomes moving between adjacent free cells.

**Key property:** The decomposition is *exact* — the boundaries of cells exactly match the boundaries of obstacles. No cell is partially occupied; every cell is definitively one or the other.

**How it works:**
1. Identify *critical points* in the environment — typically the x-coordinates of obstacle vertices
2. Draw vertical lines through all critical points — these lines divide the space into trapezoidal or rectangular strips
3. Within each strip, further divide into cells at obstacle boundaries
4. Each resulting cell is either entirely within an obstacle (occupied) or entirely outside all obstacles (free)
5. Build a graph connecting adjacent free cells
6. Search this graph for a path from the start cell to the goal cell

**Why completeness is maintained:**
The robot just needs to be able to traverse between adjacent free cells — it doesn't matter exactly *where* in a cell the robot is, just that it can cross from one to the next. This abstraction preserves completeness.

**Advantages:**
- **Complete** — if any path exists, the cell decomposition captures it
- **Exact** — no information is lost about the obstacle boundaries
- **Efficient in sparse environments** — fewer cells = faster computation when obstacles are few and far apart

**Disadvantages:**
- **Difficult implementation** — computing exact cell boundaries is geometrically complex
- **Dense environments** — number of cells grows with obstacle density → becomes computationally expensive
- **Rarely used in practice** precisely because of implementation difficulty; approximate methods are usually preferred

---

### 3.4 Approximate Cell Decomposition

**Concept:** Instead of exactly matching obstacle boundaries, use a *regular grid* (or hierarchical grid) to decompose the space. Each cell is classified as free, occupied, or mixed. Mixed cells are either treated as occupied (conservative) or further decomposed.

This is the most widely used method in practical mobile robotics — it underlies the *occupancy grid*, which is the standard environmental representation.

#### Fixed-Size Cell Decomposition (Occupancy Grid)

**Construction:**
- Divide the entire environment into a uniform grid of identically sized square cells
- For each cell, determine if it is free or occupied (based on sensor data or map)
- Navigation treats the grid as a graph — each free cell is a node, connected to its free neighbors (4-connectivity or 8-connectivity)

**Advantages:**
- **Extremely simple** — just a 2D array
- **Directly produced by sensor fusion** — LiDAR readings, sonar readings, etc. naturally fill an occupancy grid
- **Low computational complexity** — O(V) where V is the number of cells

**Disadvantages:**
- **Narrow passages:** If the grid resolution is too coarse, a narrow corridor between two obstacles may be entirely classified as "occupied" even though the robot could fit through it
- **Inexact:** Obstacle boundaries are not represented exactly — a cell near an obstacle boundary may be misclassified
- **Mitigation:** Use small cell sizes to reduce inexactness, at the cost of more memory and computation

#### Variable-Size Approximate Cell Decomposition (Quadtree)

**Concept:** Use a *hierarchical* grid — start with a coarse grid, then recursively subdivide cells that contain both free and occupied space.

**Algorithm:**
1. Start with the entire environment as one cell
2. Subdivide into 4 equal quadrants (quadtree split)
3. For each quadrant:
   - If entirely free → label FREE, stop subdividing
   - If entirely occupied → label OCCUPIED, stop subdividing
   - If *mixed* (contains both) → subdivide again into 4 (recurse)
4. Continue until all cells are pure or a maximum resolution limit is reached

**Result:** Coarse cells in open areas (efficient), fine cells near obstacles (accurate). The three cell types:
- **White (Free):** Entirely in free space
- **Black (Occupied):** Entirely inside an obstacle
- **Gray (Mixed):** Contains the boundary between free and occupied; subdivided or treated as occupied

**Path Planning:** Hierarchical — plan at coarse resolution first, then refine into finer cells if needed. Allows fast coarse planning with local refinement.

**Advantages:**
- Automatically adapts resolution — coarse where unneeded, fine where obstacles are complex
- More memory-efficient than uniform grid in large sparse environments
- Still computationally efficient (O(n log n) construction)

---

### 3.5 Lattice Graph Search

**Concept:** Precompute a set of *motion primitives* (short, feasible robot trajectories — arcs, straight lines, turns) and tile this set across the configuration space to form a graph. Every edge in the graph corresponds to a trajectory that the actual robot can execute.

**Key distinction from other methods:** Lattice graph edges are *directly executable* robot motions — they already account for the robot's kinematics. Following the solution path gives you a ready-to-execute sequence of control commands, not just a geometric path that you then need to convert to controls.

**Construction:**
1. Define a finite set of *base motion primitives* for the robot (given its kinematic model) — e.g., straight forward, arc left 30°, arc right 30°, etc.
2. Fix a *discretization* of the configuration space (position + heading + other states)
3. At each discrete state, all applicable motion primitives generate the successor states and edges
4. The resulting graph is the **state lattice**

**Example from slides:** 16-directional state lattice for a planetary exploration rover
- State space: (x, y, θ, k) — 2D position, heading, curvature
- 2D-shift invariant (same set of edges repeated everywhere in the space)
- Partially rotation-invariant
- All successor edges from state (0, 0, 0, 0) are precomputed and stored

**Advantages:**
- **Kinematically feasible:** Every edge is a valid robot motion — no post-processing needed
- **Precomputed:** Stored in memory, very fast to look up during online planning
- **Flexible:** Can encode complex kinematic constraints (minimum turn radius, velocity limits)
- Edges can be used directly as *feed-forward commands* to controllers

**Relationship to other methods:** Lattice graphs are a type of approximate cell decomposition, but with motion-model-aware edges rather than simple cell-adjacency connections.

---

## 4. Deterministic Graph Search Algorithms

Once a graph has been constructed (by any of the methods above), we need to *search* it for the best path from the start node to the goal node.

### 4.1 Cost Function Terminology

All graph search algorithms use some combination of these cost measures:

| Symbol | Name | Definition |
|---|---|---|
| **g(n)** | Path cost | Accumulated cost from the *start* node to node n — how expensive was the journey to get here? |
| **c(n, n')** | Edge traversal cost | Cost of traveling from node n to adjacent node n' — how expensive is this one step? |
| **h(n)** | Heuristic cost | *Estimated* cost from node n to the *goal* — how far do we still have to go? (an estimate, not guaranteed exact) |
| **f(n)** | Total expected cost | Combined cost: f(n) = g(n) + ε·h(n) — total expected cost of a path through node n |
| **ε** | Heuristic weight | Algorithm-dependent parameter. ε = 0 → no heuristic (Dijkstra/BFS); ε = 1 → A*; ε > 1 → weighted A* |

*Analogy:*
- g(n) = how many kilometers you've already driven to reach this intersection
- c(n, n') = the distance of the next road segment
- h(n) = your GPS estimate of remaining distance to the destination (straight-line approximation)
- f(n) = total estimated trip distance = driven so far + estimated remaining

---

### 4.2 Breadth-First Search (BFS)

**Core Idea:** Explore the graph *level by level* — first all nodes 1 step from start, then all nodes 2 steps away, then 3, and so on. Think of it as expanding a wavefront outward from the start.

**Algorithm:**
1. Initialize: Put the start node in a **queue** (FIFO — first in, first out)
2. Mark start as visited
3. While the queue is not empty:
   a. Dequeue the front node n
   b. If n is the goal → STOP, reconstruct path
   c. For each unvisited neighbor n' of n:
      - Mark n' as visited
      - Record that n' was reached from n
      - Enqueue n'
4. Reconstruct path by backtracking from goal to start

**Why FIFO works:** The queue ensures that nodes closer to the start (fewer edge hops) are processed before nodes farther away. This is exactly the level-by-level expansion needed.

**Properties:**

**Optimality:** BFS finds the path with the *fewest edges* (minimum number of steps). If all edges have *equal cost*, this is also the minimum-cost path. For *non-uniform* edge costs, BFS may not find the cheapest path (a longer route with cheaper edges could beat a shorter route with expensive edges).

**Completeness:** Complete — BFS will always find the goal if it exists in a finite, connected graph.

**Robotics Application — Wavefront Expansion:**
BFS is applied to grid-based maps as the *wavefront expansion* (or NF1/grassfire algorithm):
1. Start from the *goal cell*, not the start — work backwards
2. Label every adjacent free cell with distance 1
3. Label their neighbors with distance 2, and so on, expanding outward
4. When the wavefront reaches the robot's start cell, every cell now has a label = distance to goal
5. Navigation: always move to the neighbor with the smallest label → guaranteed shortest path

This is *enormously* useful because once computed, any start position can follow the gradient to reach the goal. One wavefront expansion serves unlimited start positions.

**Complexity:**
- **Time:** O(V + E) — visits every vertex and edge at most once
- **Space:** O(V + E) — must store all nodes in the queue and their visited status

---

### 4.3 Depth-First Search (DFS)

**Core Idea:** Explore as *deep as possible* along one branch before backtracking. Go all the way to the end of one path, then come back and try another.

**Algorithm:**
1. Initialize: Put the start node in a **stack** (LIFO — last in, first out) — or use recursion
2. While the stack is not empty:
   a. Pop the top node n
   b. If n has been visited → skip
   c. Mark n as visited
   d. If n is the goal → STOP, reconstruct path
   e. Push all unvisited neighbors of n onto the stack
3. Reconstruct path

**Why LIFO works:** The stack ensures the most recently added node is explored next — forcing the search to go deeper before coming back to explore siblings.

**Properties:**

**Optimality:** DFS does *not* guarantee the shortest path. It finds *a* path (whichever it happens to go down first), not necessarily the best one. In robotics, DFS is rarely used for path planning because non-optimal paths are generally unacceptable.

**Completeness:** DFS can get trapped in infinite loops if the graph has cycles and visited nodes aren't tracked. With proper visited-node tracking, it is complete on finite graphs.

**Key Advantage over BFS — Space Complexity:**
DFS only needs to remember the *current path* from start to the current node (the recursion stack), plus the unexplored neighbors at each point on the path. Once a node and all its descendants are explored, they can be removed from memory.

**Comparison:**
| Property | BFS | DFS |
|---|---|---|
| Strategy | Level-by-level | Deep-first |
| Data structure | Queue (FIFO) | Stack (LIFO) |
| Time complexity | O(V + E) | O(V + E) |
| Space complexity | O(V + E) | **O(V)** — better |
| Optimal (uniform cost)? | Yes | No |
| Finds shortest path? | Yes (uniform cost) | No |

*Analogy:* BFS is like searching a building by checking every room on Floor 1 before going to Floor 2. DFS is like going down one staircase all the way to the basement before backtracking and trying a different staircase.

---

### 4.4 Dijkstra's Algorithm

**Core Idea:** BFS extended to handle *non-uniform edge costs*. Always expand the node with the *lowest total path cost g(n)* from the start. This guarantees optimality even when different edges have different costs.

**How it differs from BFS:** BFS uses a regular queue (expand in order of arrival). Dijkstra uses a *priority queue* (expand in order of g(n) — lowest cost first).

**Algorithm:**
1. Initialize: g(start) = 0; g(all others) = ∞; priority queue with start
2. While priority queue not empty:
   a. Extract node n with lowest g(n)
   b. If n is goal → STOP
   c. For each neighbor n' of n:
      - Tentative cost = g(n) + c(n, n')
      - If tentative cost < g(n') → update g(n'), record predecessor, add to priority queue

**ε = 0 in the f(n) formula** — pure path cost, no heuristic.

**Properties:**
- **Optimal:** Guaranteed to find the minimum-cost path for non-negative edge costs
- **Complete:** Yes, for finite graphs with non-negative costs
- **Time complexity:** O((V + E) log V) with a binary heap priority queue

*Analogy:* Dijkstra's is like planning a road trip. At each junction, you choose to explore the road that gives you the cheapest total journey so far — not necessarily the road with fewest turns (BFS), but the cheapest total bill.

---

### 4.5 A* Algorithm

**Core Idea:** Dijkstra + heuristic. Instead of expanding the node with lowest *past cost* g(n), expand the node with lowest *total estimated cost* f(n) = g(n) + h(n). The heuristic h(n) guides the search toward the goal, dramatically reducing wasted exploration.

**ε = 1 in the f(n) formula:**
> **f(n) = g(n) + h(n)**

**The Heuristic h(n):**
- h(n) is an estimate of the cost from n to the goal
- Must be **admissible** (never overestimates the true cost) to guarantee optimality
- Common heuristics for grid-based robot navigation:
  - **Euclidean distance:** h(n) = straight-line distance to goal (always admissible if robot can move in straight lines)
  - **Manhattan distance:** h(n) = |Δx| + |Δy| (admissible for 4-connected grids)

**Why A* is better than Dijkstra's:**
Dijkstra explores in all directions equally (like a growing circle). A* focuses the exploration *toward the goal*, skipping large portions of the search space.

*Analogy:* Dijkstra's is like a blind person systematically checking every possible route from home. A* is like someone who can see the destination in the distance — they still systematically plan the cheapest route, but they naturally focus their search toward where they're going.

**Properties:**
- **Optimal:** Yes, if the heuristic is admissible
- **Complete:** Yes, for finite graphs with non-negative costs
- **Faster than Dijkstra's** in practice (often by orders of magnitude)
- **Time complexity:** O(E) in best case with perfect heuristic; O(V log V) typical

**Weighted A* (ε > 1):** Multiply the heuristic by ε > 1 to explore even more aggressively toward the goal. Finds solutions faster but sacrifices optimality (path cost is within ε-optimal, not strictly optimal). Used when speed matters more than absolute optimality.

---

### 4.6 D* Algorithm

**Core Idea:** D* (Dynamic A*) is an *online* re-planning algorithm designed for environments that *change during navigation* — new obstacles appear as the robot moves.

**Limitation of A*:** A* requires the complete map in advance. If a new obstacle blocks the planned path, A* must completely re-plan from scratch.

**How D* works:**
- Plans an initial path like A* (backward from goal to start)
- As the robot moves and sensors reveal new information (new obstacles, updated costs), D* efficiently *repairs* the existing plan rather than replanning from scratch
- Only the affected portion of the path is recomputed — highly efficient for incremental changes

**Use cases:** Navigation in partially known or dynamic environments where the robot's sensors reveal new information as it moves (e.g., discovering a blocked corridor mid-journey).

---

## 5. Randomised Graph Search — RRT

**Why randomised methods?**

For *high-dimensional configuration spaces* (e.g., a robot arm with 7 joints has a 7D configuration space; molecule docking problems have hundreds of dimensions), deterministic exhaustive search becomes computationally *infeasible*. The state space is simply too large to enumerate.

Additionally:
- No suitable heuristic function may exist for the specific problem
- Velocity and acceleration constraints cannot be easily dimensionally reduced without violating physical safety requirements

**Randomized search forgoes optimality for tractability** — it finds *a* solution quickly rather than *the best* solution slowly.

### RRT (Rapidly-Exploring Random Trees)

**Core Idea:** Build a tree that rapidly explores the configuration space by repeatedly growing toward randomly sampled points.

**Algorithm:**

```
Initialize: Tree T with root = start configuration q_start

Repeat until goal is found or max iterations reached:
  1. q_rand ← SampleRandom()        # Sample a random configuration in free space
  2. q_near ← NearestNeighbor(T, q_rand)  # Find nearest existing node in T
  3. q_new ← Extend(q_near, q_rand)  # Move from q_near toward q_rand by a fixed step
                                     # (checking for collision along the way)
  4. If extension is collision-free:
       Add q_new to T; Add edge (q_near, q_new) to T
  5. If q_new is close enough to goal:
       Connect to goal → PATH FOUND
```

**Key steps visualized:**
- q_rand = random sampled point (could be anywhere in free space)
- q_near = nearest node already in the tree
- q_new = new node placed one step toward q_rand from q_near (or at q_rand if distance < step size)
- The tree grows outward in random directions, rapidly exploring the entire space

**Why does it work?**
The Voronoi bias property: the probability of a node being selected as q_near is proportional to the *volume of its Voronoi region* — meaning nodes in *unexplored* areas (large Voronoi regions) are more likely to be selected. This naturally drives exploration into unexplored space.

**RRT evolution:** Over many iterations, the tree rapidly fills the free configuration space, eventually reaching the goal.

**Efficiency Improvements:**

**Bidirectional RRT (RRT-Connect):** Grow two trees simultaneously — one from the start, one from the goal — and connect them when they meet. Significantly faster convergence.

**Goal biasing:** Instead of purely random sampling, occasionally (e.g., 5–10% of the time) sample the *goal configuration* directly as q_rand. This biases the tree to grow toward the goal without sacrificing exploration.

**Properties of RRT:**
- **Not optimal:** The found path is unlikely to be the shortest possible path
- **Not deterministically complete:** Cannot guarantee a solution in a finite number of iterations
- **Probabilistically complete:** As the number of iterations → ∞, the probability of finding a solution (if one exists) → 1
- **Very practical:** Widely used in motion planning for robot arms, autonomous vehicles, and game AI

**RRT* (RRT-Star):** An improved variant that adds a *rewiring* step to gradually improve path quality toward optimality as more samples are added. Asymptotically optimal — converges to the optimal solution given enough time.

*Analogy:* RRT is like exploring an unknown cave by always advancing a random antenna toward a randomly chosen direction in the cave. Over time, the antenna fills the entire cave, and one of its branches will eventually reach the exit.

---

## 6. Potential Field Path Planning

### Core Concept

**Potential Field Planning** treats the robot as a physical particle placed in an *artificial potential energy field* defined over the entire configuration space.

- The **goal** creates an **attractive potential** — a "valley" (minimum) that pulls the robot toward it
- Each **obstacle** creates a **repulsive potential** — a "mountain" (maximum) that pushes the robot away

The total potential U(q) at any configuration q is the *superposition* (sum) of all attractive and repulsive contributions:

> **U(q) = U_attractive(q) + U_repulsive(q)**

The robot moves by following the *negative gradient* of U — the direction of steepest downhill:

> **Force on robot = −∇U(q)**

Like a ball placed on a hilly landscape, the robot naturally rolls toward valleys (goal) and away from mountains (obstacles).

*Analogy:* Imagine placing a marble on a surface shaped like a satellite dish (attractive bowl toward the center/goal) with small bumps (repulsive peaks at obstacles). The marble naturally rolls toward the center, deflecting around the bumps along the way.

### Mathematical Formulation

**Attractive Potential (toward goal):**

> **U_att(q) = ½ k_att × d(q, q_goal)²**

Where d(q, q_goal) is the distance from current position to goal, and k_att is an attraction scaling constant.

**Attractive Force:** F_att = −∇U_att = k_att × (q_goal − q) — always points toward the goal.

**Repulsive Potential (away from obstacles):**

For each obstacle o, define a repulsive potential that grows sharply near the obstacle and decays to zero beyond a radius ρ₀:

> U_rep(q) = ½ k_rep × (1/d(q, o) − 1/ρ₀)² if d(q, o) ≤ ρ₀
>           = 0                               if d(q, o) > ρ₀

Where d(q, o) is the distance to the nearest obstacle point, k_rep is a repulsion scaling constant, and ρ₀ is the obstacle influence radius.

**Total force:** The sum of all attractive and repulsive forces gives the net direction of travel.

### How the Robot Moves

1. At each time step, compute U(q) at current position
2. Compute the gradient ∇U(q) — direction of steepest potential increase
3. Move in the direction of −∇U(q) — steepest descent
4. Repeat until goal is reached (U = minimum) or convergence fails

**Diagram description (from slides):**
- Configuration a: Two obstacles (gray blobs) between start (top-left) and goal (bottom-right)
- Configuration b: Equipotential contour lines around obstacles; path follows the contours from start to goal
- Configuration c: 3D surface plot of potential — high "mountains" at obstacles, deep "valley" at goal, gradient guides robot along a smooth curve

### Advantages
- **Smooth, continuous paths** — naturally curved trajectories, no sharp corners
- **Real-time updates** — if a new obstacle appears, add its repulsive potential and the robot immediately deflects
- **Simple to implement** — just gradient descent
- **Works well when obstacles are few and well-separated**

### Critical Problem: Local Minima

**Definition:** A local minimum is a configuration where ∇U(q) = 0 (no net force) but q is not the goal.

This happens when attractive and repulsive forces exactly cancel each other. The robot gets "stuck" — it thinks it has reached the goal but has only reached a saddle point or local valley in the potential field.

*Analogy:* Imagine a ball rolling toward the center of a satellite dish, but there are two bumps that create a small bowl between them and the center. The ball can get stuck in that small bowl, even though the true destination (the dish center) is still further away.

**Mitigation Strategies:**
- **Randomized escaping:** Introduce random perturbations when the robot detects it's stuck
- **Navigation functions:** Design special potential functions guaranteed to have only one minimum (the goal) — mathematically complex but provably correct
- **Hybrid approaches:** Use potential fields for local avoidance, graph search for global planning — the combination avoids local minima while retaining real-time responsiveness

---

## 7. Obstacle Avoidance

While path planning provides a global route, **obstacle avoidance** handles the *real-time, local* problem of modifying trajectory to dodge obstacles detected by sensors during movement.

Local obstacle avoidance:
- Is a function of *current and recent sensor readings*
- Also considers the *relative direction and distance to the goal*
- Must operate very fast (milliseconds) to prevent collisions

---

### 7.1 Bug Algorithm (Bug1, Bug2, Tangent Bug)

**Overview:**

The Bug algorithms are the simplest possible obstacle avoidance strategies. They use *only immediate sensor values* (contact sensing or range sensing) and only need to know the *approximate direction to the goal*. No map is required.

**Basic concept:** When an obstacle is encountered, follow its contour (walk around its boundary) until you can again move directly toward the goal.

*Analogy:* You're hiking in the dark with only a compass (you know which direction is home) and you can feel when you touch a wall. Bug algorithms are what you'd naturally do: walk toward home, when you hit a wall, follow it until you can walk toward home again.

#### Bug1

**Strategy:** Full-perimeter exploration before departing

**Steps:**
1. Move directly toward the goal (straight-line motion)
2. When an obstacle is hit at a **hit point (H)** → begin following the obstacle's contour
3. Completely circle the obstacle (full perimeter traversal)
4. Record the **leave point (L)** — the location on the perimeter closest to the goal
5. Return to that leave point
6. Resume moving toward the goal

**Properties:**
- **Guaranteed to reach any reachable goal** — if any path exists, Bug1 will eventually find it (complete)
- **Very inefficient** — always fully circumnavigates each obstacle, even if a shortcut exists early in the traversal
- **Path length bound:** Known upper bound on total path length

*Analogy:* You hit a building while walking. Bug1 says: walk all the way around the building (full circle), find the doorway closest to your destination, go back to that doorway, then continue to your destination. Even if you passed a closer door earlier, you still complete the full circle.

#### Bug2

**Strategy:** Leave the obstacle immediately when possible

**Steps:**
1. Move toward goal along the *straight line M* connecting start to goal
2. When an obstacle is hit at a hit point (H):
   - Follow the obstacle contour
   - At the *first moment* you can move directly toward the goal AND you are on line M → depart (this is the leave point L)
3. Resume moving toward goal

**Improvement over Bug1:**
- Does not require full circumnavigation
- Generally produces significantly shorter total paths
- Still complete (will always find a solution if one exists)
- Still not optimal — one can construct adversarial environments where Bug2 also makes very long detours

**Disadvantage:**
- On complex obstacles (concave shapes, mazes), Bug2 can still visit long sections of the obstacle boundary before finding a valid departure point

#### Tangent Bug

**Enhancement:** Adds a *range sensor* (e.g., LiDAR) and maintains a **Local Tangent Graph (LTG)** — a local map of the nearby environment.

**Key advantages of LTG:**
- Robot can "see" gaps and passages in obstacles *before* reaching them
- Can plot more efficient routes around obstacles by anticipating their geometry
- Can identify shortcuts earlier — departs from the obstacle sooner than Bug2
- In simpler environments, Tangent Bug often finds *globally optimal* paths

**Two states of Bug2 (practical implementation):**

```
STATE: GOAL-SEEK
  - Move directly toward goal
  - If obstacle detected → switch to WALL-FOLLOW

STATE: WALL-FOLLOW  
  - Follow the obstacle contour
  - If able to move toward goal (on line M, for Bug2) → switch to GOAL-SEEK
```

This state-machine structure is the key practical implementation pattern of Bug-type algorithms.

---

### 7.2 Vector Field Histogram (VFH)

**Problem with Bug-type algorithms:** The robot's behavior at each instant depends *only on its most recent sensor reading*. A single noisy or incorrect sensor reading can cause dangerous behavior.

*Example:* A sonar sensor gets a spurious reflection from a glass wall → the robot suddenly thinks there's a wall where there isn't one → it swerves unnecessarily → potential instability.

**VFH Solution:** Build a *local map* (occupancy grid) of the environment using *multiple recent sensor readings*. Filter and combine these readings to get a more robust picture of nearby obstacles. Then plan a safe steering direction from this local map.

**VFH Algorithm (Step by Step):**

**Step 1 — Build local occupancy grid:**
- Maintain a small 2D occupancy grid centered on the robot (e.g., 5m × 5m area)
- Update cells using recent range readings (LiDAR/ultrasonic)
- Each cell has a *certainty value* based on how many times it's been detected as occupied

**Step 2 — Construct the Polar Histogram:**
- Convert the occupancy grid to a *1D polar histogram*
- X-axis: angle (0° to 360°)
- Y-axis: *obstacle density* in that direction — weighted sum of occupied cell certainties in each angular sector
- High bar = many/certain obstacles in that direction
- Low bar = clear passage in that direction

*Analogy:* Imagine standing in a room and making a bar chart of "how much stuff is in each direction around me." Tall bars = walls or furniture. Short bars = open space.

**Step 3 — Identify candidate steering directions:**
- Find *valleys* (angular sectors with low obstacle density) — these are candidate steering directions wide enough for the robot to pass through
- Filter valleys too narrow for the robot to safely traverse

**Step 4 — Select best steering direction using cost function:**

> **G = α × target_direction + β × wheel_orientation + γ × previous_direction**

Where:
- **target_direction** = how well this candidate aligns with the goal direction (low = well-aligned = preferred)
- **wheel_orientation** = how much the robot's wheels need to turn from current orientation to take this direction (low = small steering change = smooth motion)
- **previous_direction** = how different this is from the previously chosen direction (low = consistent motion = no jerky turns)
- **α, β, γ** = tuning weights: higher α = more goal-oriented; higher β = smoother steering; higher γ = more directionally consistent

The candidate direction with the *lowest total cost G* is selected.

**Step 5 — Execute the chosen direction**

**VFH+ Enhancement:**
A refined version that adds a *kinematic constraint reduction* stage:
- Models the robot's possible trajectories as arcs/straight lines based on its turning radius (especially important for Ackermann vehicles with limited turning radius)
- An obstacle blocks all trajectories that pass through it → blocked directions are enlarged in the histogram to account for all kinematically constrained paths
- Produces a *masked polar histogram* where blocked directions reflect true kinematic limitations

**Advantages of VFH:**
- More robust than Bug algorithms (uses multiple sensor readings, not just the most recent)
- Fast — runs in real-time
- Considers robot kinematics (especially in VFH+)
- Works well in crowded, complex environments

**Diagram description (from slides):**
- (a): Robot in environment with two blocking obstacles
- (b): Raw polar histogram — high bars at angles corresponding to obstacle directions
- (c): Masked polar histogram (VFH+) — obstacles enlarged to account for kinematic constraints; only safe trajectory arcs remain as low bars

---

### 7.3 Bubble Band Technique

**Core Concept:** A "bubble" is the *maximum collision-free sphere* around the robot's current configuration — the largest region in which the robot could move in any direction without hitting an obstacle.

*Analogy:* Imagine the robot surrounded by the largest possible soap bubble that doesn't touch any walls or obstacles. The size of this bubble tells you how much room to maneuver — big bubble = lots of space; small/tiny bubble = robot is in a tight corridor.

**Construction of a bubble:**
- Use range data from the robot's sensors to find the nearest obstacle in every direction
- The bubble radius = the distance to the nearest obstacle in the most constrained direction
- Computed using a simplified robot model, but the actual robot shape is considered for final trajectory smoothing

**Bubble Band:**
A *string of consecutive bubbles* from the robot's current position to the goal position. This band shows the expected free space along the entire planned path.

- Each bubble in the band represents the free space at that point on the trajectory
- The band as a whole is the *"tube"* of free space the robot expects to move through
- If two consecutive bubbles overlap, the transition between them is safely traversable

**How it provides obstacle avoidance:**

*Initially (path optimization):*
- The bubble band smooths and optimizes a given path
- The band minimizes *tension* (internal forces trying to straighten the path) while keeping bubbles in free space

*During motion (obstacle avoidance):*
- When a new obstacle appears (sensor detects something unexpected), it encroaches on the bubbles in that region
- The band responds by *deflecting* the path around the new obstacle — the bubbles shift to maintain separation from the intruding obstacle
- Tension minimization drives the deflection to be smooth and minimal

**Advantages:**
- Considers the robot's actual dimensions (not just a point)
- Produces smooth, continuous trajectories
- Integrates path planning (offline) and obstacle avoidance (real-time) naturally

**Limitations:**
- Most effective when the environment is well-known in advance (works best as a refinement of an offline plan)
- Less suitable for completely unknown environments where the planned path may be completely wrong

---

### 7.4 Curvature Velocity Method (CVM)

**Motivation:** Bug algorithms and VFH don't directly account for the robot's *kinematic constraints* — they treat the robot as capable of any direction change instantaneously. Real robots have limited turning radii, maximum speeds, and acceleration limits.

**CVM Framework:**

**Configuration Space → Velocity Space:**
CVM transforms the obstacle avoidance problem from Cartesian space (where obstacles are positioned) to *velocity space (v, ω)* — a 2D space where the axes are:
- **v** = translational velocity (forward speed)
- **ω** = angular velocity (rotation rate)

The robot's trajectory during one time step, given (v, ω), is a circular arc with curvature:
> **c = ω / v**

(Straight line when ω = 0; tighter circle with larger ω/v ratio)

**Two Types of Constraints:**

**1. Robot physical limits:**
> -v_max ≤ v ≤ v_max
> -ω_max ≤ ω ≤ ω_max

These bound the permissible (v, ω) pairs to a rectangle in velocity space.

**2. Obstacle constraints:**
For each obstacle, compute which (v, ω) pairs would lead to a collision:
- For each arc curvature c = ω/v, calculate the distance the robot can travel before hitting the obstacle
- (v, ω) pairs that would lead to collision within a safety distance are marked as forbidden

**Objective function:**
Among all permissible, non-obstacle-blocked (v, ω) pairs, choose the one that maximizes an objective function (e.g., maximize forward speed, minimize distance to goal, maximize distance from nearest obstacle).

**CVM Advantages:**
- Properly accounts for robot kinematic constraints — physically realizable motions only
- Can include some dynamic constraints (acceleration limits) for more realistic motion
- Cartesian obstacle representation + sensor fusion friendly

**CVM Limitations:**
- **Circular simplification:** Obstacles are simplified into circles — may not accurately represent complex shapes, leading to overly conservative or incorrect avoidance in some scenarios
- **Local minima:** No prior/global knowledge in basic CVM → can get trapped in dead ends

**Lane Curvature Method (LCM):**
An extension of CVM that computes a set of *candidate lanes* (corridors of various widths and lengths). Each lane trades off length vs. proximity to the nearest obstacle. The best lane is selected by an objective function, and the robot steers to enter the best lane if not already in it. Experimental results show better performance than basic CVM but requires careful parameter tuning.

---

### 7.5 Dynamic Window Approach (DWA)

**Concept:** Account for robot kinematics by limiting the velocity search to a *dynamic window* — only velocities reachable from the robot's *current speed* within the next control cycle (given the robot's acceleration limits).

Two variants: **Local DWA** and **Global DWA**.

#### Local Dynamic Window Approach

**Framework:**

**Step 1 — Define velocity space:** All (v, ω) tuples where v is translational velocity and ω is angular velocity. Robot moves in circular arcs for each (v, ω) pair.

**Step 2 — Compute the dynamic window:**
Given the robot's *current velocity* (v_curr, ω_curr), only velocities reachable in the next sample period T are considered:
> v_curr - a_v × T ≤ v ≤ v_curr + a_v × T
> ω_curr - a_ω × T ≤ ω ≤ ω_curr + a_ω × T

Where a_v and a_ω are the robot's translational and angular acceleration limits.

The dynamic window is *rectangular* (because translational and rotational dynamics are assumed independent). It's centered on the current velocity and represents all achievable velocities in one step.

**Step 3 — Filter for admissible velocities:**
From the dynamic window, remove all (v, ω) pairs where:
- The robot cannot stop before hitting an obstacle (i.e., following this velocity, the robot will collide before it can brake to a stop)
- Only the remaining *admissible velocities* are considered

**Step 4 — Apply objective function:**
Among all admissible (v, ω) pairs, maximize:

> **O = α × heading(v, ω) + β × velocity(v, ω) + γ × dist(v, ω)**

Where:
- **heading(v, ω):** Measure of alignment with the goal direction — how much does this velocity vector point toward the goal? (Higher = better)
- **velocity(v, ω):** Forward speed v — encourages fast movement (Higher = better)
- **dist(v, ω):** Distance to the closest obstacle along this trajectory — encourages clearance (Higher = better)
- **α, β, γ:** Tuning weights — adjust to trade off goal alignment vs. speed vs. safety

**Step 5 — Execute the chosen (v, ω)** for one sample period, then recompute.

**Properties:**
- Kinematically feasible by construction (dynamic window guarantees reachability)
- Dynamically safe (admissibility filter prevents unavoidable collisions)
- Real-time capable — entire computation is fast enough for online control

#### Global Dynamic Window Approach

**Extension:** Integrates global path information into the local DWA objective function using **NF1 (grassfire)** labels.

**NF1 (Navigation Function 1 / Grassfire):**
- Assign each free cell in the occupancy grid a *label* equal to its *total distance to the goal*
- Computed by BFS from the goal cell (wavefront expansion)
- NF1 labels form a global distance field that reflects the best path to the goal, considering all known obstacles

**Integration into DWA:**
- Replace the simple `heading` term in the DWA objective function with the NF1 label of the cell the robot would reach by following each (v, ω) pair
- This directs the robot along the global plan, not just toward the straight-line direction to the goal
- Handles complex environments where the straight-line to goal is blocked — NF1 naturally routes around obstacles

**NF1 Optimization:**
Computing NF1 for the entire map is expensive. The global DWA optimizes by computing NF1 only in a *selected rectangular region* between the robot and the goal (focusing computation where it matters).

**Failsafe:** If NF1 calculation is blocked (robot surrounded by obstacles on all sides), the method temporarily reverts to pure local DWA until a navigable path opens up.

**Real-world performance:**
- Implementation on an omnidirectional robot with 450 MHz on-board PC
- Cycle frequency: ~15 Hz
- Occupancy grid resolution: 5 cm
- Average sustained speed: > 1 m/s during testing

---

### 7.6 Other Obstacle Avoidance Approaches

**1. The Schlegel Approach**
Uses a geometric model of the robot and obstacle boundaries to compute safe steering commands. Accounts for robot shape more precisely than circular simplifications.

**2. Nearness Diagram (ND)**
Divides the robot's sensor field into sectors, classifies each sector as blocked or free based on obstacle proximity (nearness), and selects steering directions based on this diagram. Effective for both wide-open spaces and narrow corridors.

**3. Gradient Method**
Uses the gradient of the obstacle distance field (essentially, the direction of increasing clearance from all obstacles) as a steering signal. Related to potential field methods.

**4. Adding Dynamic Constraints**
Extensions of any of the above methods that explicitly model the robot's velocity and acceleration limits, ensuring all planned maneuvers are physically executable.

**Obstacle Avoidance Comparison Summary:**

| Method | Map Required? | Kinematics-Aware | Key Strength | Key Weakness |
|---|---|---|---|---|
| Bug1 | No | No | Guaranteed completeness | Very inefficient |
| Bug2 | No | No | Better efficiency than Bug1 | Still can be non-optimal |
| Tangent Bug | Local only | No | Can find optimal paths in simple environments | Requires range sensor |
| VFH | Local occupancy grid | Partial (VFH+) | Robust to sensor noise, real-time | Local minima possible |
| Bubble Band | Planned path | No | Smooth trajectories, shape-aware | Needs pre-planned path |
| CVM | No | Yes | Kinematic constraints enforced | Circular obstacle simplification |
| Local DWA | No | Yes | Physically feasible, real-time | No global guidance |
| Global DWA | Occupancy grid | Yes | Global guidance + local kinematics | Computationally heavier |

---

## 8. Navigation Architectures

### 8.1 Why Architecture Matters

A complete autonomous robot must integrate *multiple capabilities* simultaneously:
- Path planning
- Obstacle avoidance
- Localization
- Perception / feature extraction
- Mapping

**Navigation Architecture** is the *principled design of the software modules* that organize and integrate all these capabilities. It defines:
- What modules exist
- What each module's inputs and outputs are
- How modules communicate
- Which modules run at what frequency
- How they handle conflicts

Without a principled architecture:
- Changing one module (e.g., upgrading the sensor) breaks other modules unexpectedly
- Testing is difficult — you can't isolate components
- Long-term maintenance is nearly impossible as the system grows

---

### 8.2 Modularity and Control Localisation

**Software Modularity:**
The standard software engineering principle of breaking a system into independent, well-defined modules is even *more critical* in mobile robotics than in traditional software, because:

- Robot *hardware* can change dramatically during a project (new sensors, different chassis, updated motors)
- Robot *environment* changes too (new deployment location, different obstacles)
- Traditional computers don't face these physical changes

**Example of modularity in practice:**
- Team adds a Sick laser rangefinder to a robot that previously used only ultrasonic sensors
- The path planning module should not need to change — it just receives obstacle data from whichever sensor module is active
- The obstacle avoidance module should not need to change — it receives the same format data regardless of sensor
- Only the sensor driver module is replaced

**Ideal:** Change any sensor suite or kinematic structure (e.g., from tricycle to differential drive) without needing to modify the obstacle avoidance or navigation modules.

**Control Localisation:**
Localizing specific robot functions to specific modules so that each function can be:
- **Tested independently** — e.g., test obstacle avoidance in simulation without involving the localization or path planner modules
- **Verified exhaustively** — especially high-level decision-making, which can be simulated completely without a physical robot
- **Improved in isolation** — a learning algorithm applied only to one module (e.g., learning to recognize obstacles) without affecting others

*Analogy:* In a hospital, different departments (cardiology, neurology, orthopedics) handle their own specializations. A patient needing cardiac surgery doesn't require the neurologist's involvement. Modular software architecture applies the same specialization principle.

---

### 8.3 Decompositions

**Decompositions** are the organizing principles for dividing robot software into modules. Two key types:

**1. Temporal Decomposition:** Organizes modules by their *timing requirements* — how fast must each module respond?

**2. Control Decomposition:** Organizes modules by how their *outputs combine* to produce the robot's physical actions — do modules execute in sequence or in parallel?

---

### 8.4 Temporal Decomposition — Four Trends

In temporal decomposition, modules are arranged in a *stack* from most real-time (bottom) to least real-time (top):

```
Most real-time → Bottom (fastest cycle time, e.g., 40 Hz)
                 ↕
Least real-time → Top (slowest/offline, e.g., strategic planning)
```

Four properties *correlate* with position in the temporal stack:

#### a. Sensor Response Time

**Definition:** The time between a sensor event being acquired and the module producing a corresponding output change.

- **Low-level (bottom) modules:** Sensor response time is milliseconds — limited only by processor speed and sensor sampling rate. A wheel encoder reading causes an immediate motor speed adjustment.
- **High-level (top) modules:** Sensor response time can be seconds or longer — deliberate reasoning takes time. A new map reading might take several seconds to alter the path plan.

#### b. Temporal Depth

Temporal depth has two components:
- **Temporal Horizon:** How far *ahead* the module looks (plans for)
- **Temporal Memory:** How far *back* the module uses historical data

- **Low-level modules:** Little to no temporal depth. They react to the current sensor reading, forget previous readings quickly, and plan only for the immediate next instant.
- **High-level modules:** Large temporal depth in both directions. A path planner uses a complete map (historical) and plans a route for the next 30 minutes of navigation (horizon).

#### c. Spatial Locality

The physical *geographic scope* of the module's decisions.

- **Low-level modules:** Control wheel speed and orientation — spatially localized (affects the robot's motion over the next centimeter or so)
- **High-level modules:** Generate a route across a city — decisions have no bearing on the current meter, but determine position hundreds of meters in the future

#### d. Context Specificity

How much the module's output depends on *context* beyond its immediate inputs.

- **Low-level modules:** Context-insensitive — same sensor reading → same output, regardless of any other state. A collision avoidance module turns the wheels when an obstacle is detected — always.
- **High-level modules:** Highly context-specific — the same sensor reading (e.g., "open corridor ahead") might produce completely different outputs depending on whether the robot is delivering medicine, exploring an unknown building, or fleeing a threat.

**Four-Level Architecture Example (from slides):**

| Level | Module | Cycle Time | Sensor | Scope |
|---|---|---|---|---|
| 4 (top) | Path Planner | Slow (seconds) | Map data | Global trajectory |
| 3 | Course Deviation | Medium (100ms) | Laser rangefinder | Intermediate obstacles |
| 2 | Emergency Stop | Fast (25ms) | Short-range optical, bumpers | Imminent collision |
| 1 (bottom) | PID Motor Control | Very fast (< 10ms) | Encoders | Wheel speed |

Each adjacent level differs by *orders of magnitude* in cycle time — this is typical in real navigation architectures.

---

### 8.5 Control Decomposition — Serial vs. Parallel

**Control Decomposition** identifies how the outputs of different modules *combine* to produce the robot's actual physical actions.

#### a. Serial (Linear) Decomposition

Modules are connected in a *linear chain* — the output of one module is the input to the next.

```
Sensor Data → [Perception] → [Planning] → [Motion Control] → [Actuators]
```

Each module's output depends entirely on its upstream module's output.

**Advantages:**
- **Easy to verify:** The system is a single well-formed loop — forward simulation fully characterizes behavior
- **Predictable:** Given any input, the output can be deterministically computed
- **Simple debugging:** Follow the chain to find where incorrect behavior originates

**Disadvantages:**
- **Latency:** Data must pass through every module in sequence — real-time response may be compromised if any module is slow
- **Rigidity:** All modules must operate at the same "clock speed" or buffering is needed

#### b. Parallel Decomposition

Multiple modules operate simultaneously and their outputs are combined.

**Switched Parallel:** At each instant, *one* module's output is used (based on a selection mechanism). Like having multiple drivers where only one drives at a time.

```
Module A → ]
Module B → ] → Selector → Actuators
Module C → ]
```

Use case: "Use collision avoidance when close to obstacles; use path following otherwise."

**Mixed Parallel:** Multiple modules' outputs are *mathematically combined* (weighted sum, averaging, etc.).

```
Module A → ]
Module B → ] → Combiner → Actuators
Module C → ]
```

Use case: Behavior-based robotics where many behaviors simultaneously contribute (avoid obstacles + seek goal + follow wall) and are blended.

**Advantages of Parallel:**
- **Biomimetic:** Complex biological organisms benefit from massive parallelism (the human brain runs thousands of processes simultaneously)
- **Responsive:** Fast behaviors can react without waiting for slower ones
- **Modular:** Each behavior module is independent

**Disadvantages of Parallel:**
- **Difficult to verify:** Parallel, multithreaded implementations with complex module interactions are hard to simulate fully
- **Non-deterministic timing:** Sensor timing and race conditions between modules can cause unexpected behaviors
- **Empirical testing required:** Much testing in parallel control is done on physical robots rather than in simulation

---

### 8.6 Case Study: Tiered Robot Architecture

A standard and widely adopted architecture that combines temporal and control decomposition:

**Architecture (from slides diagram):**

```
┌─────────────────────────────────┐
│  PATH PLANNING (Strategic Layer)│  ← Non-real-time, global
└────────────────┬────────────────┘
                 ↕ (bidirectional)
┌────────────────┴────────────────┐
│  EXECUTIVE (Tactical Layer)     │  ← Bridges planning & real-time
└────────────────┬────────────────┘
                 ↕
┌────────────────┴────────────────┐
│  REAL-TIME CONTROLLER           │  ← Fast reactive layer
│  [Behavior 1][Behavior 2][Behav3│
│  [     PID Motion Control      ]│
└────────────────┬────────────────┘
                 ↕
┌────────────────┴────────────────┐
│  ROBOT HARDWARE                 │  ← Physical motors and sensors
└─────────────────────────────────┘
```

**Layer Descriptions:**

**Path Planning (Strategic Layer):**
- Non-real-time — computed once at start, and whenever the plan needs updating
- Generates optimal global paths using complete map information
- Outputs: a sequence of waypoints or a trajectory the robot should follow

**Executive (Tactical Layer):**
- Bridges the gap between strategic planning and real-time execution
- Functions:
  - Activates and deactivates behaviors in the reactive layer based on current context
  - Handles *failures* — if the path planner's plan fails, the executive decides what to do (replan, abort, try alternative)
  - Manages *short-term memory* — sequences of recent events
- Operates at intermediate time scale (seconds)

**Real-Time Controller (Reactive Layer):**
- Implements concrete behaviors that directly control the robot:
  - Obstacle avoidance (e.g., VFH or DWA running continuously)
  - Wall-following
  - Goal-seeking (point toward goal and drive)
- Runs PID loops for low-level motor control
- Must operate at tens of Hz for smooth, stable motion

**Robot Hardware:**
- Executes motor commands from the real-time controller
- Reads low-level sensor data (encoders, bumpers, IMU)
- Provides raw data to perception and control modules

**Why this tiered architecture is effective:**
- Slow, expensive global planning doesn't interfere with fast reactive behavior
- Fast reactive behavior doesn't know or care about the global plan — it just avoids immediate obstacles
- The executive coordinates them — it knows when the reactive layer is handling an obstacle and waits before issuing new planning commands
- Each layer can be tested, replaced, or improved independently

---

## 9. Reinforcement Learning for Navigation

### Motivation

Traditional navigation frameworks (graph search + VFH + architecture) rely on:
- Static or slowly-updated maps
- Deterministic sensor models
- Handcrafted rules and algorithms

These methods work well in known, structured environments. But they *struggle with uncertainty in real-world scenarios*:
- Environments change dynamically
- Sensor models have unexpected failure modes
- No human can anticipate every edge case

**Reinforcement Learning (RL)** offers an alternative: *learn navigation policies through experience* — the robot discovers effective behaviors by interacting with its environment and receiving feedback.

**Imitation Learning (IL)** complements RL by training models to replicate *expert human demonstrations* — faster learning when expert demonstrations are available.

---

### 9.1 Core RL Terminology

**Agent:** The autonomous robot (or the software controller) that perceives data and makes decisions.

**Environment:** The workspace — either a physical arena or a *digital twin* (simulation). The environment receives actions from the agent and produces observations and rewards in return.

**State (S):** A comprehensive snapshot of the robot's current situation, integrating all available information:
- *Proprioceptive data:* Encoder readings, IMU orientation and velocity
- *Exteroceptive data:* LiDAR scans, camera images

The state should be *Markovian* — it should contain all information needed to make an optimal decision (no need to know history beyond the current state).

**Action (A):** The control signals the agent outputs — either *discrete* (e.g., turn left / go straight / turn right) or *continuous* (e.g., exact wheel velocity values, exact joint torques).

**Reward (Rt):** A scalar feedback signal received after each action, telling the agent how good that action was.
- Positive reward → good action (progressed toward goal, avoided obstacle)
- Negative reward (penalty) → bad action (collided, went in wrong direction, wasted time)
- The reward function defines *what the robot is trying to achieve* — getting it right is crucial and non-trivial

**Policy (π):** The learned function that maps states to actions — π(S) = A. The goal of RL is to find the policy that maximizes *cumulative long-term reward*.

**Return:** The total accumulated reward over time:
> **Return = R_t + γ × R_{t+1} + γ² × R_{t+2} + ...**

Where γ (discount factor, 0 ≤ γ ≤ 1) balances immediate vs. future rewards:
- γ → 0: myopic, focuses on immediate reward only
- γ → 1: far-sighted, equally values distant future rewards

---

### 9.2 The RL Learning Process

The RL loop:

```
Agent observes State S_t
      ↓
Agent takes Action A_t (based on current policy π)
      ↓
Environment transitions to new State S_{t+1}
      ↓
Environment provides Reward R_t
      ↓
Agent updates policy π based on (S_t, A_t, R_t, S_{t+1})
      ↓ (repeat)
```

Four key phases:

**1. Exploration — Discovering the Environment**
- The robot tries various actions to discover their effects
- Essential when outcomes of actions are unknown
- *Robotic example:* A robotic arm tries different grip strengths and positions to pick up objects of varying weights — it doesn't know the right grip in advance

**2. Reward Feedback — Evaluating Actions**
- After each action, the agent receives a scalar reward signal R_t
- Good actions (reaching a target, avoiding a collision) → positive reward
- Bad actions (collision, moving away from goal) → negative reward / penalty
- *Robotic example:* Robot receives +10 for reaching a target position; −5 for each collision

**3. Learning and Adaptation — Improving the Policy**
- Update the policy π to increase the probability of actions that led to high rewards
- Decrease probability of actions that led to penalties
- Over time, the policy converges to behavior that maximizes cumulative return
- *Robotic example:* Walking robot adjusts gait parameters (step frequency, height, leg timing) to improve balance and speed

**4. Exploitation — Using Learned Knowledge**
- Once the policy has learned what works, *exploit* that knowledge to achieve goals efficiently
- No more random exploration — use the known best action for each state
- *Robotic example:* Delivery robot consistently chooses the fastest known route to a destination

---

### 9.3 Exploration vs. Exploitation

The central dilemma in RL: at any moment, should the agent:
- **Explore:** Try something new that might be better? (But might waste time or get a penalty)
- **Exploit:** Use the currently known best action? (But might miss even better options)

Too much exploration → never reaches goals (constant random behavior)
Too much exploitation → gets stuck in suboptimal behaviors (never discovers better strategies)

**ε-Greedy Strategy:**
- With probability ε: take a *random action* (explore)
- With probability 1-ε: take the *currently best-known action* (exploit)
- Start with high ε (lots of exploration early), gradually decrease ε (more exploitation as learning matures)

*Analogy:* A new restaurant explorer. Initially (high ε), you try random restaurants to discover the best ones. Eventually (low ε), you mostly go to your proven favorites but occasionally try something new.

**UCB (Upper Confidence Bound):**
- Prioritizes actions with *high uncertainty* — actions that haven't been tried much
- Systematically ensures every corner of the action space is explored before committing
- More sophisticated than ε-greedy; guarantees no part of the environment is permanently ignored

> **UCB score = Q(a) + c × √(ln(t) / N(a))**

Where Q(a) is the estimated value of action a, N(a) is the number of times a was tried, t is the current time step, and c is an exploration constant. Actions tried fewer times get higher UCB scores → naturally explored more.

---

### 9.4 Real-World Example: Mini Cheetah

**MIT's Mini Cheetah Quadruped Robot (trained with RL):**

**Challenge:** Learn to run at high speeds over varied terrain — a task with thousands of interacting parameters that no human could hand-code effectively.

**RL Setup:**
- **Environment:** Physics simulation of the Mini Cheetah and varied terrain
- **State:** Joint angles, joint velocities, IMU data, foot contact states, commanded velocity
- **Actions:** Joint torques for all 12 joints (3 per leg × 4 legs)
- **Reward:** Positive for high forward velocity in commanded direction; penalizes energy usage, joint limit violations, falls

**Training process:**
1. Robot (simulation copy) explores various movement patterns — billions of simulated steps
2. Receives rewards for faster locomotion; penalizes for falling, jerky motion, high energy
3. Policy gradually learns to coordinate all 12 joints efficiently
4. After training, the policy is *transferred* to the physical robot (sim-to-real transfer)

**Results:** The robot learned to run at high speeds and adapt to different terrains (grass, gravel, inclines) without any explicit programming of specific gaits.

---

### 9.5 Challenges of RL in Robotics

**1. Sample Inefficiency**
RL algorithms require vast numbers of interactions — potentially millions of trials. In simulation, this is feasible (run at 10,000× real-time speed). On *physical robots*, millions of real-world trials are impractical (time, wear, breakage). Hence the reliance on simulation and sim-to-real transfer.

**2. Exploration vs. Exploitation Balance**
Striking the right balance is difficult and highly problem-dependent. Too much exploration in physical robots can cause dangerous maneuvers; too little means the policy never improves.

**3. Reward Design (Reward Shaping)**
Designing good reward functions is challenging:
- Too sparse (reward only at goal) → robot receives no feedback for most actions → learning is extremely slow
- Too dense (reward for every small progress) → robot may find unexpected shortcuts (reward hacking) — e.g., a robot told to maximize reward for forward motion might learn to flip onto its back if that somehow triggers the reward
- Poorly designed rewards lead to unintended, undesirable behaviors

**4. Computational Cost**
RL algorithms (especially deep RL with neural network policies) require enormous computation:
- Training a complex policy may take days or weeks on powerful GPU clusters
- On-robot inference is fast, but training is expensive
- High-dimensional state/action spaces (complex robots) exponentially increase requirements

**5. Safety During Training**
Physical robots attempting random exploratory actions can:
- Damage themselves (joint limits exceeded, falls)
- Damage their environment (collisions with people, equipment)
- Pose risks to nearby humans

*Sim-to-real transfer* partially addresses this by training in simulation first, but the gap between simulation and reality means physical testing is still required for final validation.

**6. Multi-Agent Coordination**
When multiple robots operate together (swarm robotics, multi-robot warehouses), each robot's actions affect others' environments — the environment is non-stationary from any single agent's perspective. This dramatically increases complexity: agents must learn to communicate and coordinate, and equilibrium strategies may be complex.

---

### 9.6 Applications of RL

#### 1. Industrial Robotics

**Assembly and Manipulation:**
- Robots learn to grasp, assemble, and place components through trial and error
- Particularly useful for objects with variable properties (different weights, textures, shapes) where hand-coded policies would need millions of cases

**Quality Control:**
- RL agents learn to visually inspect products and identify defects
- Adapt to new product variations without reprogramming
- Can detect subtle defects that rule-based systems miss

**Logistics and Warehousing:**
- Amazon Kiva-type robots navigate dynamic warehouse environments
- Optimize pick paths based on real-time inventory locations and robot positions
- Handle dynamic obstacles (human workers, other robots, new shelving)

#### 2. Service Robotics

**Domestic Robots:**
- Learn household-specific behaviors (where things are stored, user preferences for tidying)
- Adapt to different home layouts without being reprogrammed

**Eldercare Assistance:**
- Personalize medication reminders, object retrieval routines
- Learn individual patient preferences and needs
- Adapt to changing patient mobility and health status

**Search and Rescue:**
- Navigate disaster zones with collapsed structures, debris, smoke
- No two disaster scenarios are identical — rule-based systems cannot anticipate all configurations
- RL agents adapt to extremely unstructured, dangerous, and dynamic environments

#### 3. Healthcare Robotics

**Surgical Robots:**
- Assist surgeons by controlling robotic arms during minimally invasive procedures
- Learn optimal insertion angles, force limits, tissue interaction from surgical expert demonstrations (Imitation Learning)
- Adapt to patient anatomy variations

**Physical Rehabilitation:**
- Personalize exercises based on patient's real-time progress and feedback
- Adjust resistance, range of motion, pacing as the patient improves
- Learn what encourages each individual patient to try harder

#### 4. Autonomous Vehicles

**Self-Driving Cars:**
- RL used to learn traffic rules, merge behaviors, intersection navigation
- Handle unusual situations (jaywalkers, debris in road, emergency vehicles) that are hard to enumerate in rule-based systems
- Ongoing challenge: safety requirements mean failure is very costly

**Delivery Drones:**
- Plan efficient routes considering weather, wind, obstacles
- Adapt to real-time conditions (sudden wind gusts, pop-up obstacles)
- Optimize battery usage vs. delivery time

#### 5. Legged Robots

**Locomotion Control:**
- Learn gaits adapted to terrain (gravel, mud, stairs, inclines)
- Boston Dynamics-style robots increasingly use RL for robust gait generation
- Can discover novel gaits that human engineers wouldn't design

**Humanoid Robots:**
- Learn to walk, run, climb, open doors
- Motor skill transfer from simulation to physical hardware
- Human-like movement patterns that generalize to new environments

---

### 9.7 Social Implications of RL

As RL-powered robots become more capable and widespread, they raise important societal questions:

**1. Job Displacement:**
- Automation powered by RL could displace workers in manufacturing, logistics, delivery, healthcare support
- Historical pattern: automation eliminates some jobs while creating new categories — but the transition creates real economic hardship
- Policymakers and companies must manage transitions: retraining programs, social safety nets

**2. Safety Concerns:**
- RL robots may perform unexpected actions — especially during ongoing learning phases
- In safety-critical applications (self-driving cars, surgical robots), a single failure can be catastrophic
- Need robust *safe RL* frameworks and independent safety validation systems
- Current approaches: constrained RL (add safety constraints to reward function), formal verification, extensive simulation testing

**3. Ethical Considerations — Who Defines "Good Behavior"?**
- The reward function embeds a definition of what behavior is desirable
- Who writes the reward function? Who validates it?
- Misaligned reward functions (even unintentionally) can produce harmful behaviors
- Need ethical frameworks and diverse stakeholder input in defining robot objectives

**4. Bias and Fairness:**
- RL algorithms trained on biased data may perpetuate or amplify societal inequalities
- Example: A medical robot trained primarily on data from one demographic group may perform poorly for underrepresented groups
- Fairness auditing, diverse training data, and inclusive development processes are essential

**5. Transparency and Explainability:**
- Deep RL policies (large neural networks) are "black boxes" — their decisions cannot be easily explained
- For accountability, users and regulators need to understand *why* a robot made a particular decision
- Active research area: Explainable AI (XAI) applied to RL — methods that can provide human-readable explanations of policy decisions

---

## 10. Quick Reference Summary

| Topic | Key Term | One-Line Definition |
|---|---|---|
| Navigation | Goal-directed movement | Moving from A to B reliably using maps + real-time sensors |
| Path planning | Strategic planning | Compute trajectory from start to goal using a map |
| Obstacle avoidance | Tactical reaction | Real-time trajectory adjustment to dodge detected obstacles |
| Belief state | Probabilistic position | Robot's probability distribution over possible current positions |
| Completeness | Guaranteed solution | Property of finding a path whenever one exists |
| Visibility graph | Vertex-to-vertex roadmap | Connects obstacle corners that can "see" each other; yields shortest paths |
| Voronoi diagram | Maximum clearance roadmap | Paths equidistant from all obstacles; maximizes safety clearance |
| Exact cell decomp. | Precise space division | Divides free space into cells with exact obstacle boundaries |
| Approximate cell decomp. | Grid-based division | Fixed or adaptive grid; basis of occupancy grids |
| Quadtree | Adaptive grid | Hierarchical subdivision; coarse in open space, fine near obstacles |
| Lattice graph | Kinematic motion graph | Precomputed feasible motion primitives tiled across config space |
| g(n) | Path cost | Accumulated cost from start to node n |
| h(n) | Heuristic cost | Estimated cost from n to goal (must be admissible for A* optimality) |
| f(n) | Total expected cost | f(n) = g(n) + ε·h(n) — combined cost used for priority in search |
| BFS | Level-by-level search | Explores by hop count; optimal for uniform costs; queue-based |
| DFS | Deep-first search | Explores one branch fully before backtracking; stack-based; space-efficient |
| Dijkstra | Weighted BFS | Priority queue on g(n); optimal for non-uniform costs; no heuristic |
| A* | Heuristic Dijkstra | Priority on f(n)=g(n)+h(n); faster than Dijkstra; optimal if h admissible |
| D* | Dynamic A* | Repairs plans incrementally as map changes; online re-planning |
| RRT | Randomised tree exploration | Grows tree toward random samples; probabilistically complete; not optimal |
| RRT* | Optimal RRT | RRT + rewiring step; asymptotically optimal |
| Potential field | Gradient-following navigation | Goal = valley (attractive); obstacles = peaks (repulsive); follow gradient |
| Local minima | Potential field failure | Zero-gradient point that isn't the goal; robot gets stuck |
| Bug1 | Full-circle avoidance | Circles entire obstacle, departs from closest point to goal |
| Bug2 | Shortcut avoidance | Departs obstacle as soon as it can move toward goal on M-line |
| Tangent Bug | Range-aware avoidance | Adds LTG from range sensing; finds shortcuts earlier |
| VFH | Polar histogram avoidance | Local occupancy grid → polar histogram → choose valley with lowest cost |
| VFH+ | Kinematic VFH | Adds robot kinematic constraint masking to VFH histogram |
| Bubble Band | Free-space tube technique | String of max-clearance spheres along trajectory; deflects around obstacles |
| CVM | Velocity-space avoidance | Maps obstacles into (v,ω) space; chooses best feasible velocity |
| DWA | Dynamic window avoidance | Window of reachable (v,ω); filters admissible; maximizes O = α·heading + β·velocity + γ·dist |
| NF1/Grassfire | Global distance field | BFS from goal; labels each cell with distance to goal; guides DWA globally |
| Navigation architecture | Software design principle | Organized integration of planning, avoidance, localization, perception modules |
| Modularity | Independent modules | Change one component without breaking others |
| Temporal decomposition | Time-based layer stack | Bottom = real-time (40Hz); Top = offline/strategic |
| Control decomposition | Control pathway design | Serial (linear chain) vs. Parallel (simultaneous modules) |
| Serial decomposition | Linear control flow | Easy to verify; predictable; single chain of modules |
| Parallel decomposition | Concurrent control | Biomimetic; responsive; hard to verify fully |
| Switched parallel | Winner-takes-all | One module's output used at a time |
| Mixed parallel | Blended control | Multiple modules' outputs mathematically combined |
| Tiered architecture | 4-layer robot system | Hardware → Real-time Controller → Executive → Path Planner |
| RL Agent | Learning robot | Perceives state, executes actions, receives rewards, updates policy |
| RL Environment | Robot's world | Physical or simulated workspace providing feedback |
| State S | Current situation snapshot | Proprioceptive + exteroceptive data at current instant |
| Action A | Control signal | Wheel velocities, joint torques, etc. chosen by policy |
| Reward R | Feedback signal | Scalar indicating goodness of last action |
| Policy π | Decision function | Maps state → action; what RL learns to optimize |
| Return | Cumulative reward | R_t + γR_{t+1} + γ²R_{t+2} + ... (discounted sum) |
| ε-Greedy | Exploration strategy | With prob ε take random action; else take best-known action |
| UCB | Exploration strategy | Prioritize uncertain (under-tried) actions for balanced exploration |
| Sample inefficiency | RL challenge | Needs millions of interactions — impractical for physical robots |
| Reward shaping | Reward design | Crafting rewards carefully to elicit desired behavior without unintended shortcuts |
| Sim-to-real | Simulation transfer | Train in simulation, deploy on physical robot |

---

*Notes compiled from PES University MAR Unit 4 slides (UE23CS343BB7) — Course by Dr. Ashok Kumar Patil, TA: Gunda Sukesh.*
*All slide content covered; supplemented with full explanations, mathematical derivations, analogies, and extended context for complete exam preparation.*
