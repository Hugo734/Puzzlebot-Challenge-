"""
map_utils.py — Map generation, likelihood field, scoring, and motion for MCL

Activity steps implemented here:
  B. Generate the layout of the environment (small_room.world geometry)
  C. Decide the grid dimensions (metres / pixel ratio)
  E. Assign scores to each particle (based on sums of pixel values)
  F. Filter the particles (keep only those with the highest scores)
  G+H. Dead-reckoning motion applied to survivors + Gaussian noise

The map is generated programmatically from the known world geometry —
no external file needed.  The result is a 300 × 300 numpy array at
0.02 m/pixel covering the 6 × 6 m room.

Coordinate conventions
-----------------------
  World frame  : x right, y up  (standard ROS)
  Image frame  : col right, row down  (standard image)
  Conversion   : col = origin_px + x / resolution
                 row = origin_px - y / resolution   ← y is flipped
"""

import math
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import gaussian_filter


# ── Step C: Grid dimensions ────────────────────────────────────────────────────
MAP_WORLD_SIZE_M = 6.0    # the room is 6 × 6 metres
MAP_RESOLUTION   = 0.02   # metres per pixel  →  2 cm/px resolution
MAP_SIZE_PX      = int(MAP_WORLD_SIZE_M / MAP_RESOLUTION)   # = 300 pixels
MAP_ORIGIN_PX    = MAP_SIZE_PX // 2                         # = 150  (world origin)


# ── Coordinate helpers ─────────────────────────────────────────────────────────

def world_to_pixel(x, y):
    """World (x, y) in metres → image (col, row) in pixels."""
    col = int(round(MAP_ORIGIN_PX + x / MAP_RESOLUTION))
    row = int(round(MAP_ORIGIN_PX - y / MAP_RESOLUTION))   # flip Y
    return col, row


def pixel_to_world(row, col):
    """Image (row, col) in pixels → world (x, y) in metres."""
    x = (col - MAP_ORIGIN_PX) * MAP_RESOLUTION
    y = (MAP_ORIGIN_PX - row) * MAP_RESOLUTION              # flip Y
    return x, y


def _box_corners_world(cx, cy, width, height, theta):
    """
    Return the 4 world-frame corners of a box.
      cx, cy  : centre in metres
      width   : size along the box's local X axis
      height  : size along the box's local Y axis
      theta   : rotation angle in radians (CCW from world +X)
    """
    hw, hh = width / 2.0, height / 2.0
    # Local corners (before rotation)
    local = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return [
        (cx + dx * cos_t - dy * sin_t,
         cy + dx * sin_t + dy * cos_t)
        for dx, dy in local
    ]


# ── Step B: Generate the environment layout ────────────────────────────────────

def generate_small_room_map():
    """
    Build a binary occupancy map for small_room.world.

    Room layout (from the .world file):
      - 6 × 6 m room centred at the origin
      - 4 walls, 0.2 m thick, at x = ±3 m and y = ±3 m
      - 4 box obstacles at (±1.5, ±1.5) with different orientations

    Returns
    -------
    numpy.ndarray, shape (300, 300), dtype uint8
        0   = obstacle  (wall or box)
        255 = free space
    """
    # Start with a completely free (white) canvas
    img  = Image.new('L', (MAP_SIZE_PX, MAP_SIZE_PX), color=255)
    draw = ImageDraw.Draw(img)

    def draw_box(cx, cy, width, height, theta=0.0):
        """Draw a filled black obstacle onto the map image."""
        corners_world = _box_corners_world(cx, cy, width, height, theta)
        corners_px    = [world_to_pixel(x, y) for x, y in corners_world]
        draw.polygon(corners_px, fill=0)

    # ── Walls ──────────────────────────────────────────────────────────────────
    # Each wall: (centre_x, centre_y, along-wall length, thickness, rotation)
    # Note: east/west walls are drawn as tall boxes (rotated 90° = swap w/h)
    draw_box( 0.0,  3.0, 6.4, 0.2, theta=0.0)    # north wall (extra wide to fill corners)
    draw_box( 0.0, -3.0, 6.4, 0.2, theta=0.0)    # south wall
    draw_box( 3.0,  0.0, 0.2, 6.0, theta=0.0)    # east wall
    draw_box(-3.0,  0.0, 0.2, 6.0, theta=0.0)    # west wall

    # ── Box obstacles (from small_room.world) ──────────────────────────────────
    # box_1: pose=(1.5,  1.5), size=(0.8×0.3), theta=0.0000 rad
    # box_2: pose=(-1.5, 1.5), size=(0.8×0.3), theta=1.5708 rad (90°)
    # box_3: pose=(1.5, -1.5), size=(0.8×0.3), theta=0.7854 rad (45°)
    # box_4: pose=(-1.5,-1.5), size=(0.8×0.3), theta=0.0000 rad
    draw_box( 1.5,  1.5, 0.8, 0.3, theta=0.0000)
    draw_box(-1.5,  1.5, 0.8, 0.3, theta=1.5708)
    draw_box( 1.5, -1.5, 0.8, 0.3, theta=0.7854)
    draw_box(-1.5, -1.5, 0.8, 0.3, theta=0.0000)

    return np.array(img, dtype=np.uint8)


def generate_obstacles_world_map():
    """
    Build a binary occupancy map for obstacles.world (the actual Gazebo world).

    Room layout (from obstacles.world):
      - 10 × 10 m room centred at the origin
      - 4 walls at x = ±5 m and y = ±5 m
      - Multiple obstacles: 2 boxes, 1 cylinder, 1 internal wall

    Returns
    -------
    numpy.ndarray, shape (500, 500), dtype uint8
        0   = obstacle  (wall or obstacle)
        255 = free space
    """
    map_size_m = 10.0
    map_size_px = int(map_size_m / MAP_RESOLUTION)
    origin_px = map_size_px // 2

    img = Image.new('L', (map_size_px, map_size_px), color=255)
    draw = ImageDraw.Draw(img)

    def draw_box_obstacles(cx, cy, width, height, theta=0.0):
        """Draw a filled black obstacle onto the map image."""
        corners_world = _box_corners_world(cx, cy, width, height, theta)
        corners_px = []
        for x, y in corners_world:
            col = int(round(origin_px + x / MAP_RESOLUTION))
            row = int(round(origin_px - y / MAP_RESOLUTION))
            corners_px.append((col, row))
        draw.polygon(corners_px, fill=0)

    # Walls at ±5m
    draw_box_obstacles( 0.0,  5.0, 10.2, 0.2, theta=0.0)  # north wall
    draw_box_obstacles( 0.0, -5.0, 10.2, 0.2, theta=0.0)  # south wall
    draw_box_obstacles( 5.0,  0.0, 0.2, 10.0, theta=0.0)  # east wall
    draw_box_obstacles(-5.0,  0.0, 0.2, 10.0, theta=0.0)  # west wall

    # Obstacles from obstacles.world
    draw_box_obstacles( 2.0,  2.0, 0.5, 0.5, theta=0.0)   # box_1
    draw_box_obstacles(-2.0, -1.0, 0.6, 0.6, theta=0.7854) # box_2
    draw_box_obstacles(-1.0,  3.0, 0.6, 0.6, theta=0.0)   # cylinder_1 (approx as box)
    draw_box_obstacles( 1.0, -2.0, 3.0, 0.15, theta=0.3)  # wall_internal

    return np.array(img, dtype=np.uint8)


# ── Build likelihood field ─────────────────────────────────────────────────────

def build_likelihood_field(occ_map, sigma_m=0.2):
    """
    Convert a binary occupancy map into a likelihood field for MCL scoring.

    For each pixel, the likelihood value represents how probable it is that
    a LiDAR beam would terminate there.  Pixels close to obstacles get the
    highest values; the field decays smoothly away from walls.

    Algorithm
    ---------
    1. Invert the occupancy map  →  obstacles become bright (255), free = 0
    2. Gaussian blur with a spread of sigma_m metres
    3. Normalise to [0, 255] so scores are comparable across runs

    This is the "likelihood field" model used in probabilistic robotics
    (Thrun, Burgard & Fox, 2005 — Chapter 6.4).

    Parameters
    ----------
    occ_map : ndarray (H, W) uint8
        Binary occupancy map:  0 = obstacle, 255 = free
    sigma_m : float
        Gaussian spread in metres.  Larger = softer peaks around walls.
        Default 0.2 m = 10 pixels at 0.02 m/px resolution.

    Returns
    -------
    ndarray (H, W) uint8  — likelihood values in [0, 255]
    """
    sigma_px = sigma_m / MAP_RESOLUTION          # metres → pixels (= 10 px)
    inverted = (255.0 - occ_map.astype(np.float64))  # obstacles → 255.0
    blurred  = gaussian_filter(inverted, sigma=sigma_px)

    # Normalise so the max pixel is 255
    peak = blurred.max()
    if peak > 0:
        blurred = blurred * (255.0 / peak)

    return np.clip(blurred, 0, 255).astype(np.uint8)


# ── Phase 2: Particle sampling ─────────────────────────────────────────────────

def sample_free_particles(occ_map, n_particles):
    """
    Activity Step D: scatter N particles around the robot's starting position (0, 0).

    Each particle is a hypothesis about where the robot might be.
    We sample from a Gaussian distribution centered at the robot's known starting
    pose (0, 0) with initial uncertainty, respecting free space boundaries.

    Parameters
    ----------
    occ_map     : ndarray (H, W) uint8  — 0=obstacle, 255=free
    n_particles : int

    Returns
    -------
    ndarray (N, 3) float64
        Each row is [x, y, theta]:
          x, y  in world-frame metres
          theta uniformly in [-π, π]
    """
    free_rows, free_cols = np.where(occ_map > 127)
    if len(free_rows) == 0:
        raise RuntimeError('No free pixels in the occupancy map — '
                           'check MAP_WORLD_SIZE_M and MAP_RESOLUTION.')

    # Map dimensions in pixels (handle both 300x300 and 500x500 maps)
    map_h, map_w = occ_map.shape
    origin_px = map_w // 2

    sigma_xy = 0.5
    xs = []
    ys = []

    for _ in range(n_particles):
        valid = False
        attempts = 0
        while not valid and attempts < 100:
            x = np.random.normal(0.0, sigma_xy)
            y = np.random.normal(0.0, sigma_xy)
            col = int(x / MAP_RESOLUTION + origin_px)
            row = int(origin_px - y / MAP_RESOLUTION)

            if 0 <= row < map_h and 0 <= col < map_w:
                if occ_map[row, col] > 127:
                    xs.append(x)
                    ys.append(y)
                    valid = True
            attempts += 1

        if not valid:
            xs.append(0.0)
            ys.append(0.0)

    xs = np.array(xs)
    ys = np.array(ys)
    thetas = np.random.uniform(-np.pi, np.pi, n_particles)

    return np.column_stack([xs, ys, thetas])


# ── Phase 3: Particle scoring ──────────────────────────────────────────────────

def score_particles(particles, ranges, angle_min, angle_increment,
                    likelihood, range_min=0.12, range_max=12.0, ray_step=5):
    """
    Activity Step E: assign a score to every particle based on the sum of
    likelihood-field pixel values at the projected LiDAR endpoints.

    How it works
    ------------
    For each particle (x, y, θ) and each valid LiDAR ray with range r at
    sensor angle α:

      endpoint in world frame:
          ex = x + r * cos(θ + α)
          ey = y + r * sin(θ + α)

      pixel lookup:
          col = origin_px + ex / resolution
          row = origin_px - ey / resolution   (Y flipped)
          score += likelihood[row, col]

    Particles whose projected scan aligns with the walls in the likelihood
    field accumulate the highest scores.

    The entire computation is vectorised over all N particles and all M rays
    simultaneously using numpy broadcasting — no Python loops.

    Parameters
    ----------
    particles        : (N, 3) float  — [x, y, theta] per particle
    ranges           : (M,)   float  — LiDAR ranges in metres (from /scan)
    angle_min        : float         — angle of ray 0 in radians
    angle_increment  : float         — angular step between rays in radians
    likelihood       : (H, W) uint8  — likelihood field (high = near wall)
    range_min/max    : float         — valid range bounds (from LaserScan msg)
    ray_step         : int           — use every Nth ray (1 = all 360,
                                       5 = 72 rays).  Reduces compute time
                                       with little loss of accuracy.

    Returns
    -------
    scores : (N,) float64  — one score per particle; higher is better
    """
    # ── Select and validate rays ───────────────────────────────────────────────
    # ray_step lets us skip rays for speed.  e.g. ray_step=5 → 72 rays instead of 360.
    indices = np.arange(0, len(ranges), ray_step)
    r_sub   = np.asarray(ranges)[indices]

    valid    = np.isfinite(r_sub) & (r_sub >= range_min) & (r_sub <= range_max)
    r_valid  = r_sub[valid]                              # (M_valid,) ranges
    idx_valid = indices[valid]                           # original ray indices

    if len(r_valid) == 0:
        return np.zeros(len(particles), dtype=np.float64)

    # Absolute angle of each selected ray in the sensor frame
    ray_angles_sensor = angle_min + idx_valid * angle_increment  # (M_valid,)

    # ── Vectorised endpoint computation ───────────────────────────────────────
    # We want: for every (particle, ray) pair → one endpoint (ex, ey).
    # Shape trick: expand particles to (N,1) and rays to (1,M) so numpy
    # broadcasts the multiply to give (N, M) arrays.

    xs     = particles[:, 0, np.newaxis]   # (N, 1)  world x of each particle
    ys     = particles[:, 1, np.newaxis]   # (N, 1)  world y
    thetas = particles[:, 2, np.newaxis]   # (N, 1)  heading

    # World-frame ray angle = particle heading + sensor-frame ray angle
    world_angles = thetas + ray_angles_sensor[np.newaxis, :]   # (N, M_valid)

    # Endpoint coordinates in metres
    ex = xs + r_valid[np.newaxis, :] * np.cos(world_angles)    # (N, M_valid)
    ey = ys + r_valid[np.newaxis, :] * np.sin(world_angles)    # (N, M_valid)

    # ── World → pixel ──────────────────────────────────────────────────────────
    # Use likelihood field shape to handle dynamic map sizes (300x300, 500x500, etc.)
    map_h, map_w = likelihood.shape
    origin_px = map_w // 2

    cols = np.round(origin_px + ex / MAP_RESOLUTION).astype(np.int32)
    rows = np.round(origin_px - ey / MAP_RESOLUTION).astype(np.int32)

    # ── Bounds mask — endpoints outside the map get score 0 ───────────────────
    in_bounds = (
        (rows >= 0) & (rows < map_h) &
        (cols >= 0) & (cols < map_w)
    )

    # Clamp for safe indexing (out-of-bounds values are zeroed by the mask)
    rows_c = np.clip(rows, 0, map_h - 1)
    cols_c = np.clip(cols, 0, map_w - 1)

    # ── Lookup pixel values and accumulate ────────────────────────────────────
    # likelihood[row, col] → high near walls, low in open space
    pixel_vals = likelihood[rows_c, cols_c].astype(np.float64)  # (N, M_valid)
    pixel_vals[~in_bounds] = 0.0                                 # mask invalid

    # Score per particle = sum of all pixel values across its rays
    return pixel_vals.sum(axis=1)   # (N,)


# ── Phase 4: Particle filtering ────────────────────────────────────────────────

def filter_particles(particles, scores, keep_fraction=0.5):
    """
    Activity Step F: keep only the top-scoring particles.

    The bottom (1 - keep_fraction) of particles are discarded entirely.
    The survivors will be used to seed the next generation in propagate_particles.

    Parameters
    ----------
    particles     : (N, 3) float — [x, y, theta] per particle
    scores        : (N,)   float — score from score_particles()
    keep_fraction : float in (0, 1] — fraction of particles to keep

    Returns
    -------
    survivors      : (K, 3) float — top-K particles, sorted best-first
    surv_scores    : (K,)   float — their corresponding scores
    """
    k = max(1, int(len(particles) * keep_fraction))
    # argsort ascending → last k entries are the highest scores
    sorted_idx  = np.argsort(scores)
    top_idx     = sorted_idx[-k:][::-1]    # best-first
    return particles[top_idx], scores[top_idx]


# ── Pose estimation ────────────────────────────────────────────────────────────

def estimate_pose(particles, scores):
    """
    Compute a score-weighted pose estimate from a particle set.

    Position is the weighted centroid.  Orientation uses the circular
    mean of angles weighted by score, which avoids wrap-around artefacts.

    Parameters
    ----------
    particles : (N, 3) float — [x, y, theta]
    scores    : (N,)   float — non-negative weights

    Returns
    -------
    (x, y, theta) : weighted mean pose
    """
    w = scores - scores.min()           # shift so all weights ≥ 0
    total = w.sum()
    if total == 0.0:
        # Degenerate case: all scores equal → uniform weights
        w = np.ones(len(particles))
        total = float(len(particles))

    w /= total   # normalise

    x = float(np.dot(w, particles[:, 0]))
    y = float(np.dot(w, particles[:, 1]))

    # Circular mean: average sin and cos separately to handle ±π wrap
    sin_mean = float(np.dot(w, np.sin(particles[:, 2])))
    cos_mean = float(np.dot(w, np.cos(particles[:, 2])))
    theta = float(math.atan2(sin_mean, cos_mean))

    return x, y, theta


# ── Phase 5 + 6: Propagate particles (G + H) ──────────────────────────────────

def propagate_particles(survivors, surv_scores, delta_s, delta_theta,
                        sigma_xy=0.02, sigma_theta=0.05, n_out=None):
    """
    Activity Steps G + H: resample from survivors, apply dead-reckoning motion,
    and add Gaussian noise.

    Algorithm
    ---------
    1. Resample  : draw n_out particles from survivors with replacement,
                   probability ∝ score (importance resampling).
    2. Propagate : advance each particle by the odometry-derived (delta_s,
                   delta_theta) step, using its current heading.
    3. Noise     : add independent Gaussian noise to x, y, and theta so the
                   cloud spreads to cover pose uncertainty.

    The motion model (unicycle):
        theta_new = theta + delta_theta
        x_new     = x + delta_s * cos(theta + delta_theta / 2)
        y_new     = y + delta_s * sin(theta + delta_theta / 2)

    Parameters
    ----------
    survivors    : (K, 3) float  — filtered particles from filter_particles()
    surv_scores  : (K,)   float  — their scores (used as resampling weights)
    delta_s      : float         — forward displacement (metres) since last step
    delta_theta  : float         — change in heading (radians) since last step
    sigma_xy     : float         — std-dev of position noise in metres
    sigma_theta  : float         — std-dev of heading noise in radians
    n_out        : int or None   — output particle count; defaults to len(survivors)

    Returns
    -------
    new_particles : (n_out, 3) float — propagated + noisy particle set
    """
    k = len(survivors)
    if n_out is None:
        n_out = k

    # ── Step 1: Importance resampling ─────────────────────────────────────────
    w = surv_scores.astype(np.float64)
    w = w - w.min()           # non-negative
    total = w.sum()
    if total == 0.0:
        w = np.ones(k, dtype=np.float64)
    else:
        w /= total             # normalised probabilities
    # Force exact sum = 1.0: float64 accumulation errors can push the sum
    # outside numpy's 1e-8 tolerance and cause ValueError in np.random.choice.
    w = np.clip(w, 0.0, None)
    w[-1] = max(0.0, 1.0 - float(w[:-1].sum()))
    if w.sum() == 0.0:
        w = np.ones(k, dtype=np.float64)
    w /= w.sum()   # final pass to guarantee exactly 1.0

    idx = np.random.choice(k, size=n_out, replace=True, p=w)
    resampled = survivors[idx].copy()      # (n_out, 3)

    # ── Step 2: Apply dead-reckoning motion ────────────────────────────────────
    thetas = resampled[:, 2]
    mid_theta = thetas + delta_theta / 2.0     # heading at midpoint of arc

    resampled[:, 0] += delta_s * np.cos(mid_theta)
    resampled[:, 1] += delta_s * np.sin(mid_theta)
    resampled[:, 2]  = thetas + delta_theta

    # ── Step 3: Add Gaussian noise ─────────────────────────────────────────────
    resampled[:, 0] += np.random.normal(0.0, sigma_xy,    n_out)
    resampled[:, 1] += np.random.normal(0.0, sigma_xy,    n_out)
    resampled[:, 2] += np.random.normal(0.0, sigma_theta, n_out)

    # Wrap theta to [-π, π]
    resampled[:, 2] = (resampled[:, 2] + np.pi) % (2 * np.pi) - np.pi

    return resampled


# ── Load saved maps ────────────────────────────────────────────────────────────

def load_map_from_pgm(pgm_path):
    """
    Load a saved .pgm occupancy map for use with MCL.

    The .pgm format is the standard ROS nav2 map storage format.
    ROS convention: 0=occupied, 254=free, 205=unknown.
    MCL convention: 0=obstacle, 255=free.

    Parameters
    ----------
    pgm_path : str
        Path to the .pgm file

    Returns
    -------
    ndarray (H, W) uint8
        Binary occupancy map: 0=obstacle, 255=free
    """
    img = Image.open(pgm_path).convert('L')
    arr = np.array(img, dtype=np.uint8)

    # Map ROS convention to MCL convention
    # Unknown (205) → obstacle (conservative)
    arr[arr == 205] = 0
    # Free (254) and above → 255
    arr[arr > 127] = 255
    # Occupied and below → 0
    arr[arr <= 127] = 0

    return arr
