# ROS2 MAPF Simulation

A ROS2 (Jazzy) + Gazebo simulation environment for testing and evaluating **Multi-Agent Path Finding (MAPF)** algorithms. The system loads standard benchmark maps from the [Stern et al. 2019](https://ojs.aaai.org/index.php/ICAPS/article/view/3476) MAPF benchmark dataset, reconstructs the obstacle layout in 3D Gazebo, spawns differential-drive robots, and exposes a clean Python API that your MAPF solver can plug into with no ROS2 knowledge required.

---

## Overview

### What this is for

MAPF research involves coordinating multiple agents from start positions to goal positions without collisions. Algorithms are typically validated on abstract grid maps. This system bridges the gap between abstract grid plans and physical simulation — you feed in a grid-based plan, and the system executes it with real robot dynamics in Gazebo, giving you:

- **Physical validation** of collision-free plans under real kinematics
- **Execution feedback** — timing, position error, and per-robot completion status per timestep
- **Plug-and-play integration** — your MAPF solver calls a simple Python API; the simulation handles all ROS2 communication internally

### System architecture

```
Your MAPF Solver
      |
      | (Python import)
      v
 FleetClient                    ← Python API layer — no ROS2 knowledge needed
      |
      | ROS2 topics (nav_msgs/Path, nav_msgs/Odometry, std_msgs/Bool)
      v
 FleetController Node           ← Proportional waypoint controller, 20 Hz
      |
      | ROS2 topics (geometry_msgs/Twist, nav_msgs/Odometry)
      v
 Gazebo (gz sim)                ← Physics simulation
      |
      | ros_gz_bridge
      v
 Differential-drive robots      ← One per agent, namespaced robot_00, robot_01, ...
```

---

## Prerequisites

- ROS2 Humble
- Gazebo (gz-sim)
- `ros_gz` bridge and simulation packages
- Python packages: `xacro`, `PyYAML`

Build the workspace:

```bash
cd ~/your_ws
colcon build --packages-select warehouse_gz
source install/setup.bash
```

---

## Quick Start

### 1. Configure the simulation

Edit `warehouse_gz/config/warehouse.yaml`:

```yaml
map_file: "arena.map"          # filename from warehouse_gz/maps/
map_obstacle_height: 1.5       # meters, height of obstacle boxes in Gazebo
resolution: 0.5                # meters per grid cell
origin: [-10.0, -10.0]         # world coordinates (x, y) of the map's bottom-left corner

spawn:
  robots: 2                    # number of robots to spawn
  pattern: "grid"
  start_region:                # fallback region used when no start_positions are passed in
    xmin: -5
    xmax: 5
    ymin: -5
    ymax: 5
  robot_spacing: 1.1           # meters between robots in the fallback grid pattern
  robot_clearance: 1.1         # minimum clearance from obstacles for spawn positions
```

### 2. Launch the simulation

**Default — robots spawn on an auto-generated grid pattern:**
```bash
ros2 launch warehouse_gz sim.launch.py
```

**With a specific map:**
```bash
ros2 launch warehouse_gz sim.launch.py map_file:=lak103d.map
```

**With MAPF start positions (grid row,col pairs):**
```bash
ros2 launch warehouse_gz sim.launch.py start_positions:="10,5;10,7;10,9"
```

Start positions are specified as semicolon-separated `row,col` pairs — one per robot. The count of pairs overrides the `robots` field in the YAML. Robots are named `robot_00`, `robot_01`, ... in order of the pairs.

### 3. Connect your solver

```python
from warehouse_gz.fleet_client import FleetClient

with FleetClient() as fleet:
    # verify map matches your solver's expectations
    name, dims, resolution, origin = fleet.get_map_info()

    # send a single-step goal (world coordinates)
    fleet.send_goal("robot_00", x=2.5, y=1.5)
    fleet.wait_for_goal("robot_00", timeout=30.0)

    # send a single-step goal (grid coordinates)
    fleet.send_grid_goal("robot_00", row=10, col=5)
    fleet.wait_for_goal("robot_00", timeout=30.0)

    # execute a full timestep-synchronized plan
    plan = [
        {"robot_00": (10, 5), "robot_01": (12, 5)},   # timestep 0
        {"robot_00": (10, 6), "robot_01": (12, 6)},   # timestep 1
        {"robot_00": (10, 7), "robot_01": (12, 7)},   # timestep 2
    ]
    results = fleet.execute_plan(plan)
```

---

## Map Format

Maps must be in the **octile format** from the MAPF benchmark dataset. File structure:

```
type octile
height <H>
width <W>
map
<H rows of W characters>
```

Cell characters:
| Character | Meaning |
|-----------|---------|
| `.` | Free — traversable |
| `G` | Free — traversable (grass) |
| `S` | Free — traversable (swamp) |
| `T` | Blocked — tree/obstacle |
| `@` | Blocked — out of bounds |

Maps are loaded from `warehouse_gz/maps/`. Included maps:

- `arena.map` — 49×49 enclosed arena
- `lak103d.map` — benchmark map from the Dragon Age Origins dataset

To use your own map, copy the `.map` file into `warehouse_gz/maps/`, rebuild (`colcon build`), then set `map_file` in the YAML or pass it as a launch argument.

The map loader automatically wraps maps that lack a border with a ring of `@` cells to ensure robots cannot escape the navigable area.

---

## FleetClient API Reference

Import:
```python
from warehouse_gz.fleet_client import FleetClient
```

### Constructor

```python
FleetClient(
    robot_names: Optional[List[str]] = None,
    robot_count: int = 0,
)
```

- If `robot_names` is `None`, reads robot count from `warehouse.yaml` and generates names `robot_00`, `robot_01`, ...
- If `robot_names` is provided, uses that list exactly — useful when your solver manages its own agent naming.
- Automatically reads map info and ROS2 config from the installed package. Spins a background daemon thread.
- Supports context manager (`with FleetClient() as fleet:`), which calls `shutdown()` on exit.

### Position queries

```python
fleet.get_position(robot_name: str) -> Tuple[float, float, float]
```
Returns `(x, y, yaw)` in world coordinates (meters, radians). Raises `RuntimeError` if no odometry received yet.

```python
fleet.get_all_positions() -> Dict[str, Tuple[float, float, float]]
```
Returns positions for all robots that have reported odometry. Returns an empty dict if none have.

```python
fleet.get_grid_position(robot_name: str) -> Tuple[int, int]
```
Returns `(row, col)` grid cell the robot currently occupies. Returns `None` if out of map bounds.

### Sending goals

```python
fleet.send_goal(robot_name: str, x: float, y: float) -> None
```
Send a single waypoint in **world coordinates** (meters).

```python
fleet.send_grid_goal(robot_name: str, row: int, col: int) -> None
```
Send a single waypoint in **grid coordinates**. Converts to world coordinates internally using the map's resolution and origin.

### Waiting for completion

```python
fleet.wait_for_goal(robot_name: str, timeout: float = 10.0) -> bool
```
Blocks until the robot reaches its current goal or `timeout` seconds elapse. Returns `True` if reached, `False` on timeout.

```python
fleet.is_goal_reached(robot_name: str) -> bool
```
Non-blocking check. Returns `True` if the robot's last goal was reached.

### Plan execution

```python
fleet.execute_plan(plan: List[Dict[str, Tuple[int, int]]]) -> List[Results]
```

Executes a timestep-synchronized MAPF plan. At each timestep, all goals are sent simultaneously, then the method blocks until every robot finishes before advancing to the next timestep.

**Plan format:**
```python
plan = [
    {"robot_00": (row, col), "robot_01": (row, col)},   # timestep 0
    {"robot_00": (row, col), "robot_01": (row, col)},   # timestep 1
    ...
]
```

- Keys are robot name strings (`"robot_00"`, `"robot_01"`, ...)
- Values are `(row, col)` integer tuples — grid coordinates
- Not all robots need to appear in every timestep
- Returns a `List[Results]`, one entry per timestep (see below)

```python
fleet.load_plan_from_file(path: str) -> List[Results]
```

Loads a plan from a JSON file and calls `execute_plan`. The JSON format mirrors the plan structure above, with tuples represented as arrays:

```json
[
    {"robot_00": [10, 5], "robot_01": [12, 5]},
    {"robot_00": [10, 6], "robot_01": [12, 6]}
]
```

### Map info

```python
fleet.get_map_info() -> Tuple[str, Tuple[int, int], float, List[float]]
```

Returns `(map_filename, (height, width), resolution, origin)`. Use this to validate that the simulation is running the same map your solver planned for.

```python
map_name, (h, w), res, origin = fleet.get_map_info()
assert map_name == "arena.map", "Wrong map loaded!"
```

### Lifecycle

```python
fleet.shutdown() -> None
```
Stops the background spin thread and cleans up the ROS2 node. Called automatically when using the context manager.

---

## Results Object

`execute_plan` returns a list of `Results` dataclass instances, one per timestep:

```python
@dataclass
class Results:
    timestep: int                              # index into the plan
    wall_clock_time: datetime.datetime         # wall clock time when timestep started
    sim_time: rclpy.time.Time                  # Gazebo sim time when timestep started
    complete: Dict[str, bool]                  # robot_name -> did it reach the goal?
    completion_time: Dict[str, timedelta]      # robot_name -> time from start to arrival
    position_error: Dict[str, Tuple[float, float]]  # robot_name -> (dx, dy) error in meters
```

**Example — checking results:**
```python
results = fleet.execute_plan(plan)

for r in results:
    print(f"Timestep {r.timestep}:")
    for robot, reached in r.complete.items():
        if not reached:
            print(f"  {robot} TIMED OUT")
        else:
            dx, dy = r.position_error[robot]
            err = (dx**2 + dy**2) ** 0.5
            print(f"  {robot} reached goal, error={err:.3f}m, time={r.completion_time[robot]}")
```

---

## Coordinate System

The simulation uses two coordinate systems that you will interact with:

**Grid coordinates `(row, col)`** — integer indices into the `.map` file.
- Row 0 is the **top** of the map file
- Col 0 is the **left** edge
- Used by MAPF solvers and in plan format

**World coordinates `(x, y)`** — continuous meters in the Gazebo world frame.
- X increases to the right
- Y increases upward
- Origin `[-10.0, -10.0]` in the YAML means the bottom-left corner of the map sits at `x=-10, y=-10` in Gazebo

**Conversion formula** (handled automatically by `send_grid_goal` and `get_grid_position`):
```
x = origin_x + (col + 0.5) * resolution
y = origin_y + (map_height - row - 0.5) * resolution
```

The `+0.5` centers positions in the middle of a cell. The Y-axis is inverted because grid row 0 (top of file) maps to the largest Y value in Gazebo.

---

## Controller Parameters

The fleet controller node (`robot_controller.py`) accepts ROS2 parameters that can be set in `sim.launch.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_count` | from YAML | Number of robots to control |
| `linear_speed` | `0.5` | Maximum forward speed (m/s) |
| `angular_speed` | `1.0` | Maximum rotation speed (rad/s) |
| `goal_tolerance` | `0.15` | Distance (m) at which a waypoint is considered reached |
| `heading_tolerance` | `0.1` | Angle error (rad) below which the robot drives forward |
| `collision_buffer` | `0.8` | Distance (m) below which a robot stops to avoid another |

---

## ROS2 Topics

Per-robot topics (substitute `robot_00`, `robot_01`, etc.):

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/{name}/odom` | `nav_msgs/Odometry` | Gazebo → Controller, FleetClient | Robot position and orientation |
| `/{name}/goal_path` | `nav_msgs/Path` | FleetClient → Controller | Waypoint path to follow |
| `/{name}/goal_status` | `std_msgs/Bool` | Controller → FleetClient | `True` when final waypoint reached |
| `/{name}/cmd_vel` | `geometry_msgs/Twist` | Controller → Gazebo | Velocity commands |

---

## Adding Maps

1. Copy your `.map` file (octile format) into `warehouse_gz/maps/`
2. Rebuild: `colcon build --packages-select warehouse_gz`
3. Set `map_file` in `warehouse_gz/config/warehouse.yaml` or pass `map_file:=yourmap.map` as a launch argument

Maps from the [Moving AI MAPF benchmarks](https://movingai.com/benchmarks/mapf.html) work directly.

---

## Package Structure

```
warehouse_gz/
├── config/
│   └── warehouse.yaml          # Main configuration file
├── launch/
│   ├── sim.launch.py           # Top-level launch — gen world, gz sim, spawn, controller
│   └── spawn.launch.py         # Robot spawning, bridge setup, state publishers
├── maps/
│   ├── arena.map               # Small enclosed test map (49x49)
│   ├── lak103d.map             # Small test map
│   └── ...                     # the rest of Stern et al. (2019) benchmark maps
├── models/simple_bot/          # Differential-drive robot XACRO model
├── warehouse_gz/
│   ├── fleet_client.py         # External program API — import this in your solver
│   ├── robot_controller.py     # ROS2 node — proportional waypoint controller
│   ├── gen_world.py            # Generates Gazebo SDF world from map file
│   └── map_utils.py            # Map parsing and coordinate conversion utilities
└── test/
    └── test_map_utils.py       # Unit tests for map parsing and coordinate conversion
```
