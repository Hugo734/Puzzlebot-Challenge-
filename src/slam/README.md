# puzzlebot_slam

2D occupancy-grid SLAM (Simultaneous Localization And Mapping) for the MCR2
Puzzlebot. The package builds a metric map of the environment from LiDAR
scans while simultaneously tracking the robot's pose inside that map.

---

## 1. The SLAM problem in one paragraph

SLAM has to answer two questions at the same time, and they depend on each
other:

- **Where am I?** — needs a map to compare the LiDAR scan against.
- **What does the world look like?** — needs an accurate pose to place the
  scan into the map.

This package solves the chicken-and-egg problem with a classical pipeline:
*odometry gives a first guess, scan-matching (ICP) refines it, and the
refined pose is used to integrate the scan into a probabilistic occupancy
grid.* The pose correction is then published as a `map → odom` TF, which is
the standard ROS way of expressing "the EKF was here, but SLAM thinks the
real position is over there."

---

## 2. Overall architecture

```
                          ┌──────────────────────┐
                          │  EKF localization    │   (separate package)
                          │  /odom + odom→base TF│
                          └──────────┬───────────┘
                                     │
                       LaserScan     │ Odometry
                          │          │
                          ▼          ▼
                       ┌─────────────────────┐
                       │      slam_node      │
                       │                     │
                       │  ┌───────────────┐  │
                       │  │ ICP scan match│  │   icp.py
                       │  └──────┬────────┘  │
                       │         │           │
                       │  ┌──────▼────────┐  │
                       │  │ Occupancy grid│  │   occupancy_grid.py
                       │  │   log-odds    │  │
                       │  └──────┬────────┘  │
                       └─────────┼───────────┘
                                 │
                  /map  /slam_pose  TF: map → odom
```

Three Python modules:

| File | Responsibility |
|---|---|
| `slam_node.py` | ROS node — orchestrates everything, manages frames and timing |
| `icp.py` | Pure-NumPy 2D Iterative Closest Point scan matcher |
| `occupancy_grid.py` | Log-odds occupancy grid with Bresenham ray-casting |

---

## 3. Localization side: how the pose is estimated

### 3.1 Three frames, one TF chain

ROS uses a chain of coordinate frames; SLAM is responsible for the top one:

```
map ──(SLAM)──► odom ──(EKF)──► base_footprint ──(URDF)──► laser
```

- `odom → base_footprint` is published by the EKF (smooth, drifts over time).
- `map → odom` is published by **this node**: it is the *correction* that
  cancels the EKF drift so that `base_footprint` ends up at the right place
  in the global `map` frame.
- `base_footprint → laser` is a static URDF transform (the LiDAR is
  physically offset 5 cm forward and 11 cm above the wheel base).

### 3.2 Why we don't replace odometry — we *correct* it

SLAM does **not** publish the robot's pose directly. Instead, it computes
where the EKF thinks the robot is (in `odom`) and where SLAM thinks the
robot really is (in `map`), then publishes the difference as `map → odom`:

```python
T_map_odom = T_map_base ⊕ inverse(T_odom_base)
```

That is the SE(2) composition you can see in `slam_node.py` lines 303–305.
The benefit: the EKF can keep running at 20 Hz with smooth, low-latency
output, while SLAM corrects for accumulated drift at the 5–10 Hz scan rate
without ever creating a "jumpy" robot.

### 3.3 The pose-estimation pipeline per scan

Every time a `/scan` arrives (`_scan_callback`), the node performs:

1. **Find the robot's pose at the scan's timestamp** —
   not the latest odom! See §3.5 below.
2. **Propagate the SLAM pose from odom** — apply the cached `map → odom`
   correction to the new odom reading. This is the *initial guess*.
3. **(Optional) Refine with ICP** — if `use_icp=True`, run scan-to-scan ICP
   against the previous scan and correct the pose. Disabled by default
   because the EKF + external ICP node already handle this in the C++
   localization stack.
4. **Integrate the scan into the occupancy grid** at the refined pose.
5. **Recompute and publish `map → odom`.**

### 3.4 ICP — Iterative Closest Point (icp.py)

ICP is a classical scan-matching algorithm. Given two point clouds
(`source` and `target`) and a rough initial guess of how they're aligned,
it iteratively finds the rigid 2D transform `(dx, dy, dθ)` that best
overlays them.

The algorithm in `icp.py`:

1. **Apply the initial guess** to the source cloud (so we start close).
2. **Nearest-neighbour matching** — for each source point, find the closest
   target point (brute-force, O(N·M); fine for ≤500 LiDAR rays).
3. **Outlier rejection** — drop pairs whose distance > `reject_dist`. This
   stops moving objects or sensor noise from dragging the solution.
4. **Best-fit rigid transform** via Singular Value Decomposition (SVD).
   Mathematically:
   - Centre both matched sets at their centroids.
   - Compute `H = Aᵀ B`, decompose `H = UΣVᵀ`.
   - The optimal rotation is `R = V Uᵀ` and translation `t = centroid_B − R·centroid_A`.
   - A determinant check ensures we get a proper rotation, not a reflection.
5. **Iterate** — apply the new transform, accumulate it onto the total, and
   repeat until either convergence (translation+rotation below `tolerance`)
   or `max_iter` is reached.
6. **Quality gate** — return `fitness` (mean inlier distance) and a
   `converged` flag. The caller in `slam_node.py` only accepts the
   correction if `fitness < icp_max_fitness`, so a bad scan match cannot
   poison the pose.

### 3.5 Time-aware odometry — fixing the rotation smear

A non-obvious problem: the LiDAR scan stamp is the moment the scan
*started*; the robot keeps moving while the laser sweeps. By the time the
ROS callback fires, the EKF has typically advanced 20–50 ms further. If we
project the scan with the *current* odom, walls fan outward whenever the
robot rotates.

Two fixes are layered in this code:

- **Scan-time interpolation** (`_odom_at`, lines 163–194): the node keeps a
  200-sample circular buffer of recent odom readings and linearly
  interpolates `(x, y, θ)` at the exact `msg.header.stamp` of the scan. The
  yaw interpolation is wrap-safe (handles the ±π discontinuity).
- **Within-scan deskew** (passed into `OccupancyGrid.update_scan`): even
  one full LiDAR revolution (~100 ms on RPLidar A1) is long enough that the
  robot rotates noticeably during a single sweep. The node estimates the
  angular velocity `ω` over the scan period from the odom buffer and passes
  it to the grid; each ray's heading is corrected by `ω · (i/N) · scan_time`.

Together these two corrections turn rotation-induced wall-fanning into
clean, single-pixel walls.

---

## 4. Mapping side: the occupancy grid (occupancy_grid.py)

### 4.1 Why log-odds?

A naive grid would store `P(occupied)` per cell in `[0, 1]`. Updating that
under repeated observations requires multiplying probabilities, which
under/overflows numerically and is slow. The log-odds trick:

```
l(cell) = log( P(occ) / (1 − P(occ)) )
```

- Updates become **additions** instead of multiplications.
- Saturating the cell is just a `clip(l, l_min, l_max)`.
- "Unknown" is `l = 0` (50/50).
- Converting back to a probability is only done when publishing.

Each LiDAR ray contributes:

- `+l_occ` to the **endpoint cell** (something blocked the beam — it's
  probably occupied).
- `+l_free` (a *negative* number) to **every cell along the beam** before
  the endpoint (the beam passed through — they're probably free).

After many observations, real walls accumulate strong positive log-odds and
saturate at `l_max`; open space saturates at `l_min`. The clamps prevent
either side from becoming so confident that it can never be revised
(important for handling moving obstacles or correcting earlier mistakes).

### 4.2 Bresenham ray-casting

For each ray we need to enumerate the integer grid cells the beam passes
through. `_bresenham()` (lines 128–147) is the textbook integer-only line
algorithm: it walks from the laser origin cell to the endpoint cell using
only addition, comparison and sign tracking — no floating point, no
trigonometry per cell. This is what makes the per-scan grid update fast.

### 4.3 The full per-ray pipeline (`update_scan`)

For every ray in the scan:

1. **Skip invalid readings** (NaN, below `range_min`).
2. **Decide hit vs miss** — if the range is < `range_max`, the endpoint is
   marked occupied; otherwise the ray is treated as free out to `range_max`
   with no occupied endpoint.
3. **Compute per-ray heading** including the within-scan deskew offset
   and the LiDAR's static yaw offset relative to the base.
4. **Compute per-ray laser origin** — when the robot is rotating fast, the
   LiDAR's world position shifts slightly *during* the scan because it's
   offset 5 cm from the rotation centre. Recomputing the origin per ray
   matters at high angular velocity.
5. **Bresenham trace** — walk every cell from origin to endpoint, adding
   `l_free` (clamped). Stop at the grid boundary.
6. **Mark the endpoint** — if it was a hit and inside the grid, add
   `l_occ` (clamped).

### 4.4 Tuned log-odds values

The values in `slam_params.yaml` are tuned for the specific tradeoff of
this challenge:

```yaml
l_occ:   0.50    # one hit ≈ 0.62 probability occupied
l_free: -0.20    # one free observation ≈ 0.45 probability occupied
l_min:  -2.0     # free saturates fast (≈5 observations)
l_max:   3.5     # occupied takes longer (≈7 observations)
```

The asymmetric clamps mean walls "remember" longer than free space does.
The 2.5× ratio (`|l_occ| / |l_free|`) makes occupied cells win against
sporadic grazing free-rays, so distant walls don't erode just because the
robot mostly looks past them.

### 4.5 Publishing as `nav_msgs/OccupancyGrid`

`to_ros_data()` thresholds the log-odds grid into the ROS convention:

| Internal log-odds | Published value | Meaning |
|---|---|---|
| `l > l_occ`   | `100` | Occupied (rendered black in RViz) |
| `l < l_free`  | `0`   | Free (rendered light grey) |
| otherwise     | `-1`  | Unknown (rendered dark grey) |

The grid is republished every `map_publish_every` scans (default 5) to keep
ROS bandwidth down — the underlying log-odds array is updated on every
scan regardless.

---

## 5. Putting it together — the data flow per scan

This is the timeline inside `_scan_callback` (slam_node.py lines 219–320):

```
LaserScan arrives  ───►  _scan_callback
                          │
   wait for /odom         │
   resolve TF base→laser  │  (cached after first scan)
                          ▼
   scan_t = msg stamp ─►  _odom_at(scan_t)        # interpolated odom
                          │
                          ▼
   scan_to_points()       # convert ranges to (x,y) in base frame
                          │
                          ▼
   _propagate_from_odom() # SLAM pose = T_map_odom ⊕ T_odom_base (latest)
                          │
                          ▼
   if use_icp:            # optional refinement against last scan
       icp() → SE(2) compose into SLAM pose
                          │
                          ▼
   build SCAN-time pose = T_map_odom ⊕ scan-time odom
   estimate ω over scan period from odom buffer
                          │
                          ▼
   grid.update_scan(...)  # Bresenham + log-odds for every ray
                          │
                          ▼
   recompute T_map_odom = T_map_base ⊕ T_odom_base⁻¹
                          │
                          ▼
   publish /slam_pose, broadcast map→odom TF
   every Nth scan: publish /map
   cache scan in WORLD frame for next ICP iteration
```

The key insight is the **dual pose** trick: the live odom drives the TF and
the visualised pose marker (so they stay synchronised in RViz), but the
*scan-time* odom is what gets used to project rays into the map (so walls
don't smear during rotation).

---

## 6. Why this design and not a more powerful SLAM library?

This is a teaching codebase for TE3003B, so:

- Every algorithm is implemented in plain NumPy / standard ROS interfaces —
  no `slam_toolbox`, no `gmapping`, no `cartographer`. You can read every
  line and trace exactly how a scan turns into a map cell.
- ICP is brute-force (no kd-tree); the grid is a single NumPy array; the
  ray-caster is the textbook Bresenham. The point is *clarity*, not raw
  throughput.
- The pose-correction-via-`map→odom`-TF pattern is the same one that
  `slam_toolbox` and `amcl` use, so the lessons transfer directly.

---

## 7. File-by-file summary

| File | Lines | Purpose |
|---|---|---|
| `slam_node.py` | ~390 | The ROS node. Subscribes to `/scan` and `/odom`, runs the per-scan pipeline, manages the TF tree, and publishes the map. |
| `occupancy_grid.py` | ~163 | The probabilistic map. Log-odds storage, Bresenham ray-casting, per-ray within-scan deskew, ROS serialisation. |
| `icp.py` | ~175 | The scan matcher. Nearest-neighbour search + SVD-based best-fit rigid 2D transform, iterated to convergence. |
| `config/slam_params.yaml` | — | Tuned parameters: grid size, log-odds values, ICP thresholds, scan filtering. |
| `launch/slam.launch.py` | — | Wires the node up with the right parameters and a `lidar_link → laser` static TF for real-hardware mode. |
