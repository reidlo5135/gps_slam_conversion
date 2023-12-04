from dataclasses import dataclass
from enum import Enum


@dataclass
class Point():
    x: float = 0.0
    y: float = 0.0


@dataclass
class Map():
    mapping_map_width: float = 0.0
    mapping_map_height: float = 0.0
    slam_rotation_angle: float = 0.0
    

class WorkType(Enum):
    SLAM: str = 'SLAM'
    GPS: str = 'GPS'


__all__ = ['position_domain']