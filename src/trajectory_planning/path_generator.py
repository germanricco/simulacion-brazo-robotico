import numpy as np

class PathGenerator:
    """
    
    """
    def __init__(self, path):
        """path: Matriz de Nx7 de poses"""
        self.original_path = path
        self.current_path = path.copy()

    def segmentar_path(self, path, num_segments):
        """
        Segmenta un path, reduciendo el número de puntos para obtener aproximadamente
        el número deseado de segmentos.

        Parámetros:
            * path: (Pose array) Trayectoria de puntos [ [x0, y0, z0], [x1, y1, z1], ... ]
            * num_segments: (int) Número deseado de segmentos en el path segmentado.

        Retorna:
            * segmented_path: (Pose array): Nuevo path segmentado (subconjunto de Pose de path original).
                    Si num_segments es inválido o mayor que el máximo posible,
                    retorna el path original sin segmentar.
        """

        n_points = len(path)
        # verifica que tenga mas de 2 puntos
        if n_points < 2:
            print(f"Error al segmentar trayectoria. Debe tener mas de 2 puntos")
            return path
        
        # calcula y verifica numero de segmentos
        max_segments = n_points - 1
        if num_segments <= 0:
            raise ValueError("num_segments debe ser mayor que 0.")
        if num_segments >= max_segments:
            return path  # Retornar path original si se piden demasiados segmentos

        indices = np.linspace(0, n_points - 1, num_segments + 1, dtype=int)
        segmented_path = []

        for i in indices:
            segmented_path.append(path[i])

        return segmented_path
    
    def calc_path_length(self, path):
        """
        Calcula la longitud de una trayectoria compuesta por objetos Pose.

        Parámetros:
            * path: (List[Pose]): Trayectoria de poses

        Retorna:
            * float: Longitud total de la trayectoria en unidades de coordenadas.
        """
        num_puntos = len(path)
        if num_puntos < 2:
            return 0
        
        longitud_segmento = np.linalg.norm(np.diff(path, axis=0), axis=1)
        longitud_total = np.sum(longitud_segmento)
        return longitud_total
    
    def resample_path(self, new_n_points):
        pass
    
    def smooth_path(self, window_size = 5):
        """Aplica filtro de media movil al path"""

    def get_path(self):
        return self.current_path
    