import numpy as np

class Zone:
    def __init__(self, cuboid, zone_type: str):
        self.cuboid = cuboid
        self.zone_type = zone_type  # "Safe" o "Forbidden"

    def contains(self, point: np.ndarray) -> bool:
        return self.cuboid.contains(point)
    
    def segmento_intersecta_cuboide(self, inicio_segmento, fin_segmento):
        return self.cuboid.segmento_intesecta_cuboide(inicio_segmento, fin_segmento)