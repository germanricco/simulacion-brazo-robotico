import numpy as np

class SafetyMonitor:
    def __init__(self):
        self.safe_zones = []
        self.forbidden_zones = []

    def add_zone(self, zone):
        if zone.zone_type == "Safe":
            self.safe_zones.append(zone)
        elif zone.zone_type == "Forbidden":
            self.forbidden_zones.append(zone)
        else:
            raise ValueError(f"Tipo de zona no soportado: {zone.zone_type}")
        
    def is_trajectory_safe(self, trajectory):
        """
        Chequea si una trayectoria es segura.

        Argumentos:
            * trajectory (numpy.array): Un array (n, 3) que representa la trayectoria.

        Retorna:
            * bool: True si la trayectoria es segura. False si no lo es.
        """
        if not isinstance(trajectory, np.ndarray) or trajectory.ndim != 2 or trajectory.shape[1] != 3:
            raise ValueError("Trajectory must be a numpy array of shape (n, 3)")

        for point in trajectory:
            # Check if the point is in a forbidden zone
            for forbidden_zone in self.forbidden_zones:
                if forbidden_zone.contains(point):
                    return False

            # Check if the point is in a safe zone (if safe zones are defined)
            if self.safe_zones:
                in_safe_zone = any(safe_zone.contains(point) for safe_zone in self.safe_zones)
                if not in_safe_zone:
                    return False

        return True