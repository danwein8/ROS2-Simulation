import math
import os
from pathlib import Path
import yaml

SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="warehouse_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
          <!-- simple gray material; we can get fancier if desired -->
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_FOOTER = """
  </world>
</sdf>
"""

def box_model(name: str, x: float, y: float, z: float, sx: float, sy: float, sz: float) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <!-- simple brown material -->
          <material>
            <ambient>0.6 0.3 0.1 1</ambient>
            <diffuse>0.6 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

def task_marker(name: str, x: float, y: float) -> str:
    # visual-only marker: cylinder on the floor
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.05 0 0 0</pose>
      <link name="link">
        <visual name="marker">
          <geometry><cylinder><radius>0.35</radius><length>0.1</length></cylinder></geometry>
          <!-- simple red material for tasks -->
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.yaml, "r"))

    parts = [SDF_HEADER]

    # shelves: each shelf is a rectangle, we model as a box
    for i, sh in enumerate(cfg.get("shelves", []), start=1):
        x = float(sh["x"])
        y = float(sh["y"])
        w = float(sh["w"])
        h = float(sh["h"])
        # height of shelf obstacle in sim
        z = 0.75
        sz = 1.5
        parts.append(box_model(f"shelf_{i}", x, y, z, w, h, sz))

    # tasks as markers
    for t in cfg.get("tasks", []):
        tid = str(t["id"])
        parts.append(task_marker(f"task_{tid}", float(t["x"]), float(t["y"])))

    parts.append(SDF_FOOTER)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(parts))
    print(f"Wrote world: {out_path}")

if __name__ == "__main__":
    main()
