import numpy as np

class Zone:
    def __init__(self, cuboid, zone_type: str):
        self.cuboid = cuboid
        self.zone_type = zone_type  # "Safe" o "Forbidden"

        self.min_bounds = cuboid.min_point
        self.max_bounds = cuboid.max_point

    def contains(self, point: np.ndarray) -> bool:
        return self.cuboid.contains(point)
    
    def segmento_intersecta_cuboide(self, inicio_segmento, fin_segmento):
        return self.cuboid.segmento_intersecta_cuboide(inicio_segmento, fin_segmento)
    
    def closest_point_on_cuboid(self, point):
        return self.cuboid.closest_point_on_cuboid(point)

