import numpy as np
import trimesh

class Obstacle:
    def __init__(self, center: np.ndarray, size: np.ndarray):
        self.center = np.array(center)
        self.size = np.array(size)  # [width, height, depth]

    def check_collision(self, point_1: np.ndarray, point_2: np.ndarray) -> bool:
        obs = trimesh.creation.box(extents=self.size)
        direction = point_2 - point_1

        locations, index_ray, index_tri = obs.ray.intersects_location(ray_origins=[point_1], ray_directions=[direction])

        if len(locations) > 0:
            distance_to_hit = np.linalg.norm(locations[0] - point_1)
            step_distance = np.linalg.norm(direction)

            if distance_to_hit <= step_distance:
                return True
        else:
            return False

