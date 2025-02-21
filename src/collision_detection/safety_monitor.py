import numpy as np
import sys
from pathlib import Path

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from utils.auxiliary_methods import verificar_path

class SafetyMonitor:
    def __init__(self, robot):
        """
        Inicializa el gestor de seguridad
        """
        self.robot = robot
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
        
    def static_check(self, cartasian_path):
        """
        Verifica si una trayectoria es segura.

        Argumentos:
            * cartasian_path (numpy.array): Una matriz de forma (n, 7) que representa la trayectoria.

        Retorna:
            * bool: True si la trayectoria es segura. False si no lo es.
        """

        # Verifico validez del path
        if not verificar_path(cartasian_path):
            raise TypeError("Path no valido.")
        
        # Extraigo posiciones
        position_path = cartasian_path[:,:3]
        
        # Verificar cada punto de la trayectoria
        for i, point in enumerate(position_path):
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
                punto_inicial = position_path[i - 1]
                punto_final = point

                # Verificar interseccion con cada zona prohibida
                for forbidden_zone in self.forbidden_zones:
                    if forbidden_zone.segmento_intersecta_cuboide(punto_inicial, punto_final):
                        return False  
        return True
    
    def dynamic_check(self, joint_path):
        pass

    def _update_robot_model(self, joints_positions):
        # Logica para actualizar modelo del robot segun pose actual
        pass

    def _check_capsule_collision(self, capsule):
        # Logica para deteccion de colisiones
        pass
