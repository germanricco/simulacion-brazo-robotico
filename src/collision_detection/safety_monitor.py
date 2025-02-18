import numpy as np

class SafetyMonitor:
    def __init__(self, robot):
        """
        Inicializa el gestor de seguridad
        """
        self.safe_zones = []
        self.forbidden_zones = []

    def add_zone(self, zone):
        """
        Funcion para agregar zonas al gestor de seguridad
        """
        if zone.zone_type == "Safe":
            self.safe_zones.append(zone)
        elif zone.zone_type == "Forbidden":
            self.forbidden_zones.append(zone)
        else:
            raise ValueError(f"Tipo de zona no soportado: {zone.zone_type}")
        
    def is_trajectory_safe(self, trajectory):
        """
        Verifica si una trayectoria es segura.

        Argumentos:
            * trajectory (numpy.array): Una matriz de forma (n, 3) que representa la trayectoria.

        Retorna:
            * bool: True si la trayectoria es segura. False si no lo es.
        """
        if not isinstance(trajectory, np.ndarray) or trajectory.ndim != 2 or trajectory.shape[1] != 3:
            raise ValueError("La trayectoria debe ser un numpy array de forma (n, 3)")
        
        # Verificar cada punto de la trayectoria
        for i, point in enumerate(trajectory):
            print(f"i: {i} || Point: {point}")
            # Verifica si el punto esta en una zona prohibida
            for forbidden_zone in self.forbidden_zones:
                if forbidden_zone.contains(point):
                    return False

            # Verifica si el punto esta en una zona segura
            if self.safe_zones:
                in_safe_zone = any(safe_zone.contains(point) for safe_zone in self.safe_zones)
                if not in_safe_zone:
                    return False
            
            # Si el punto es seguro, verificar interseccion del segmento con zona prohibida
            if i > 0:
                punto_inicial = trajectory[i - 1]
                punto_final = point

                # Verificar interseccion con cada zona prohibida
                for forbidden_zone in self.forbidden_zones:
                    if forbidden_zone.segmento_intersecta_cuboide(punto_inicial, punto_final):
                        return False  
        return True
