import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely import LineString

class Obstacle:
    def __init__(self, center: np.ndarray, size: np.ndarray):
        self.center = np.array(center)
        self.size = np.array(size)  # [width, height]
        self.half_size = self.size / 2

    def check_collision(self, point_1: np.ndarray, point_2: np.ndarray) -> bool:

        ll_corner = self.center - self.half_size
        lr_corner = (self.center[0] + self.half_size[0], self.center[1] - self.half_size[1])
        hl_corner = (self.center[0] - self.half_size[0], self.center[1] + self.half_size[1])
        hr_corner = self.center + self.half_size
        
        line = LineString([point_1, point_2])
        polygon = Polygon([ll_corner, hl_corner, hr_corner, lr_corner])

        return line.intersects(polygon)