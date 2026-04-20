"""Task data model and Gazebo marker management for warehouse task lifecycle."""

import logging
import os
import subprocess
import tempfile
from dataclasses import dataclass
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)


class TaskStatus(Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class Task:
    task_id: str
    row: int
    col: int
    dwell_time: float = 5.0
    status: TaskStatus = TaskStatus.PENDING
    assigned_robot: Optional[str] = None
    world_x: float = 0.0
    world_y: float = 0.0
    marker_spawned: bool = False


class GazeboMarkerManager:
    """Spawn and remove visual task markers in Gazebo via gz service CLI."""

    WORLD_NAME = "warehouse_world"

    @staticmethod
    def _task_sdf(name: str, x: float, y: float) -> str:
        """Return SDF string for a task marker (matches gen_world.task_marker format)."""
        return (
            "<?xml version='1.0'?>"
            "<sdf version='1.8'>"
            f"<model name='{name}'>"
            "<static>true</static>"
            f"<pose>{x} {y} 0.05 0 0 0</pose>"
            "<link name='link'>"
            "<visual name='marker'>"
            "<geometry><cylinder><radius>0.35</radius><length>0.1</length></cylinder></geometry>"
            "<material>"
            "<ambient>1 0 0 1</ambient>"
            "<diffuse>1 0 0 1</diffuse>"
            "</material>"
            "</visual>"
            "</link>"
            "</model>"
            "</sdf>"
        )

    @classmethod
    def spawn_marker(cls, task_id: str, x: float, y: float) -> bool:
        """Spawn a red cylinder marker in Gazebo. Returns True on success."""
        sdf = cls._task_sdf(task_id, x, y)
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".sdf", delete=False
            ) as f:
                f.write(sdf)
                tmp_path = f.name

            result = subprocess.run(
                [
                    "gz", "service",
                    "-s", f"/world/{cls.WORLD_NAME}/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--req", f'sdf_filename: "{tmp_path}" name: "{task_id}"',
                    "--timeout", "5000",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                logger.warning(
                    "Failed to spawn marker %s: %s", task_id, result.stderr
                )
                return False
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.warning("Marker spawn error for %s: %s", task_id, e)
            return False
        finally:
            if tmp_path is not None:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass

    @classmethod
    def remove_marker(cls, task_id: str) -> bool:
        """Remove a task marker from Gazebo. Returns True on success."""
        try:
            result = subprocess.run(
                [
                    "gz", "service",
                    "-s", f"/world/{cls.WORLD_NAME}/remove",
                    "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean",
                    "--req", f'name: "{task_id}" type: MODEL',
                    "--timeout", "5000",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                logger.warning(
                    "Failed to remove marker %s: %s", task_id, result.stderr
                )
                return False
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.warning("Marker remove error for %s: %s", task_id, e)
            return False
