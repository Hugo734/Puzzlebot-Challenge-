# Monte Carlo Localization (MCL)

## Overview

Monte Carlo Localization is a **particle filter algorithm** that estimates a robot's position in a known environment using sensor data and motion estimates. It maintains a **cloud of particles** (position hypotheses) that converge toward the robot's true location.

## How It Works

The algorithm follows these steps:

**A. Simulator** — Uses Gazebo with laser scan (`/scan`) and wheel speeds (`/wr`, `/wl`)

**B. Map** — Generate occupancy grid from environment layout

**C. Grid** — Resolution: 0.02 m/pixel, covering 10×10 meters

**D. Sample Particles** — Initialize 500 particles in free space

**E. Score** — For each particle, project laser scan onto map. High score = scan aligns with obstacles

**F. Filter** — Keep top 50% of particles (best matches)

**G. Dead-Reckoning** — Estimate robot motion from wheel speeds

**H. Propagate** — Move surviving particles forward with motion estimate + noise

**I. Repeat** — Loop when new laser scan arrives

## Data Flow

```
Gazebo
  ├→ /scan (laser)
  └→ /wr, /wl (wheel speeds)
       ↓
    Score particles vs. laser
       ↓
    Filter (keep top 50%)
       ↓
    Estimate pose
       ↓
    Propagate + add noise
       ↓
    /particles (RViz)
    /mcl_pose (estimated position)
```

## Core Functions

| Function | Purpose |
|----------|---------|
| `generate_obstacles_world_map()` | Create occupancy grid |
| `build_likelihood_field()` | Convert map to scoring field |
| `sample_free_particles()` | Initialize 500 particles |
| `score_particles()` | Score vs. laser scan |
| `filter_particles()` | Keep top particles |
| `propagate_particles()` | Move + resample particles |

## Running

```bash
ros2 launch puzzlebot_localization mcl.launch.py
```

Launches:
1. Gazebo simulation
2. MCL node
3. RViz (top-down 2D view)
4. Teleop (keyboard control — arrow keys)

## Interpreting Results

**RViz shows:**
- **Grid** — Coordinate system background
- **Robot** — Gray Puzzlebot model
- **Red arrows** — Particle cloud
- **Green arrow** — MCL's estimated position
- **Red rays** — Laser scan

**What to look for:**
- ✅ Particles converge on robot → MCL localizing correctly
- ❌ Particles scattered → position uncertain
- ❌ Particles drift from robot → need better sensor/motion model

## Tuning

Adjust in `mcl.launch.py`:

```bash
ros2 launch puzzlebot_localization mcl.launch.py \
  n_particles:=1000 \
  keep_fraction:=0.3 \
  sigma_xy:=0.01
```

| Parameter | Meaning |
|-----------|---------|
| `n_particles` | Number of particles (more = better but slower) |
| `keep_fraction` | Fraction of particles to keep each cycle |
| `sigma_xy` | Position noise (meters) |
| `sigma_theta` | Heading noise (radians) |

## Files

- `mcl_node.py` — Main MCL algorithm
- `map_utils.py` — Particle filter functions
- `mcl.launch.py` — Launch Gazebo + MCL + RViz
- `instructions.md` — Original algorithm requirements
