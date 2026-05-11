import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely import LineString

class Obstacle:
    def __init__(self, center: np.ndarray, size: np.ndarray):
        self.center = np.array(center)
        self.size = np.array(size)  # [width, height, depth]
        self.half_size = self.size / 2

        self.min_bound = self.center - self.half_size
        self.max_bound = self.center + self.half_size

    def check_collision(self, point_1: np.ndarray, point_2: np.ndarray) -> bool:

        # ll_corner = self.center - self.half_size
        # lr_corner = (self.center[0] + self.half_size[0], self.center[1] - self.half_size[1])
        # hl_corner = (self.center[0] - self.half_size[0], self.center[1] + self.half_size[1])
        # hr_corner = self.center + self.half_size
        
        # line = LineString([point_1, point_2])
        # polygon = Polygon([ll_corner, hl_corner, hr_corner, lr_corner])

        # return line.intersects(polygon)

        """
        Checks if the line segment from point_1 to point_2 intersects the 3D cube.
        Uses a vectorized Slab Test (AABB-Line Segment intersection).
        """
        p1 = np.array(point_1)
        p2 = np.array(point_2)
        d = p2 - p1
        
        # To avoid division by zero, use a small epsilon
        epsilon = 1e-10
        d_eff = np.where(np.abs(d) < epsilon, epsilon, d)

        # Calculate intersection "times" with the 6 planes of the cube
        t1 = (self.min_bound - p1) / d_eff
        t2 = (self.max_bound - p1) / d_eff

        # Get the entry and exit points for each axis
        t_min = np.minimum(t1, t2)
        t_max = np.maximum(t1, t2)

        # The entry point of the cube is the maximum of the individual min-times
        t_near = np.max(t_min)
        # The exit point of the cube is the minimum of the individual max-times
        t_far = np.min(t_max)

        # Intersection occurs if:
        # 1. The entry time is before or at the exit time (t_near <= t_far)
        # 2. The intersection is within the segment (0 <= t_far and t_near <= 1)
        if t_near <= t_far and t_far >= 0 and t_near <= 1:
            return True
            
        return False