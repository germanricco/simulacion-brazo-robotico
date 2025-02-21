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

        self._safe_limits = None
        self._forbidden_limits = None

    def add_zone(self, zone):
        """
        Agrega zonas a las listas del gestor de seguridad y precalcula minimos y maximos combinados
        """
        if zone.zone_type == "Safe":
            self.safe_zones.append(zone)
        elif zone.zone_type == "Forbidden":
            self.forbidden_zones.append(zone)
        else:
            raise ValueError(f"Tipo de zona no soportado: {zone.zone_type}")
        
        self._update_limits() #Llamada a subrutina
        
    def _update_limits(self):
        """
        Actualiza limites combinados para early rejection
        """
        # Safe zones: bounding boz union
        if self.safe_zones:
            self._safe_limits = (
                np.min([z.min_bounds for z in self.safe_zones], axis=0),
                np.max([z.max_bounds for z in self.safe_zones], axis=0)
            )

        # Forbidden zones: bounding box union
        if self.forbidden_zones:
            self._forbidden_limits = (
                np.min([z.min_bounds for z in self.forbidden_zones], axis=0),
                np.max([z.max_bounds for z in self.forbidden_zones], axis=0)
            )

    def _all_points_in_safe(self, path: np.ndarray) -> bool:
        """
        Verifica que todos las posiciones de path se encuentren dentro de zonas seguras
        """
        if not self.safe_zones:
            return False
        
        # 1. Early rejection: verifica puntos fuera del bounding box global safe
        if self._safe_limits:
            in_global_safe = np.all(
                (path >= self._safe_limits[0]) & 
                (path <= self._safe_limits[1]),
                axis=1
            )

            # Si algun punto esta fuera, la trayectoria es insegura
            if not np.all(in_global_safe):
                return False
        
        # 2. Verificación detallada por zonas
        for point in path:
            if not any(zone.contains(point) for zone in self.safe_zones):
                return False
        return True
    
    def _any_point_in_forbidden(self, path: np.ndarray) -> bool:
        """
        Verifica si alguna posicion se encuentra dentro de una zona prohibida
        """
        if not self.forbidden_zones:
            return False
        
        # Busca puntos candidatos que estan dentro de la zona global prohibida. 
        # Si estan fuera, son seguros y no necesito analizarlos detalladamente.
        if self._forbidden_limits:
            in_global_forbidden = np.all(
                (path >= self._forbidden_limits[0]) & 
                (path <= self._forbidden_limits[1]),
                axis=1
            )
            candidates = path[in_global_forbidden]

        else:
            candidates = path
        
        # Verificar solo puntos candidatos
        for point in candidates:
            if any(zone.contains(point) for zone in self.forbidden_zones):
                return True
        return False
    
    def _any_segment_intersects_forbidden(self, path: np.ndarray) -> bool:
        """
        Verifica si algun segmento intersecta una zona prohibida
        """
        if not self.forbidden_zones or len(path) < 2:
            return False
        
        # Precomputar segmentos
        segments = zip(path[:-1], path[1:])
        print(segments)
        # Early rejection con bounding box global
        min_f, max_f = self._forbidden_limits
        
        for start, end in segments:
            # Early rejection del segmento completo
            if not self._segment_intersects_aabb(start, end, min_f, max_f):
                continue
            
            # Verificar intersección con cada zona
            for zone in self.forbidden_zones:
                if zone.segmento_intersecta_cuboide(start, end):
                    return True
        return False
    
    def _segment_intersects_aabb(self, start: np.ndarray, end: np.ndarray, 
                               min_b: np.ndarray, max_b: np.ndarray) -> bool:
        """Early rejection usando algoritmo Liang-Barsky para AABB combinado"""
        dir_vec = end - start
        t_min, t_max = 0.0, 1.0

        for i in range(3):
            if np.abs(dir_vec[i]) < 1e-9:
                if start[i] < min_b[i] or start[i] > max_b[i]:
                    return False
            else:
                t1 = (min_b[i] - start[i]) / dir_vec[i]
                t2 = (max_b[i] - start[i]) / dir_vec[i]
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))

        return t_min <= t_max

    def is_path_safe(self, path: np.ndarray) -> bool:
        """Interfaz principal manteniendo la lógica original pero optimizada"""
        if not isinstance(path, np.ndarray) or path.ndim != 2:
            raise ValueError("La trayectoria debe ser un numpy array de dim=2")

        # Verificación en 3 etapas optimizadas
        return not (self._any_point_in_forbidden(path) or
                   (self.safe_zones and not self._all_points_in_safe(path)) or
                   self._any_segment_intersects_forbidden(path))

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
