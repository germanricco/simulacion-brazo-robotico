import numpy as np
import sys
from pathlib import Path

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from trajectory_planning.bezier_path import BezierPath
from trajectory_planning.orientation_planner import OrientationPlanner
from trajectory_planning.simple_path import SimplePath

class Pose:
    """
    Clase auxiliar para representar posicion (x,y,z) y orientacion (A,B,C)
    """
    def __init__(self, position: np.ndarray, orientation: np.ndarray):
        self.position = position
        self.orientation = orientation

class PathPlanner():
    """
    Planificador de trayectorias. Integra interpolacion de posicion (Bezier, lineales, circulares) y orientacion (SLERP)
    """
    def __init__(self):
        # Principio SOLID. No hereda sino que instancia internamente
        self.bezier_path = BezierPath()
        self.simple_path = SimplePath()
        self.orientation_planner = OrientationPlanner()

    def generate_path(
            self,
            path_type,
            start_pose,
            end_pose,
            num_poses = 100,
            orientation_mode = "slerp"
    ):
        """
        Genera una trayectoria suave integrando posicion y orientacion.

        Parametros:
            * path_type (str): Tipo de trayectoria ('bezier', 'linear', 'circle').
            * start_pose (Pose): Pose inicial (posicion + orientacion)
            * end_pose (Pose): Pose final
            * num_poses (int): numero de poses a interpolar
            * orientation_mode (str): "slerp" (interpolacion)

        Retorna:
            List[Pose]: Lista de poses a lo largo de la trayectoria.
                de la forma (n,6) = [x,y,z,A,B,C]
        """
        positions = self.generate_positions(
            path_type,
            start_position=start_pose.position,
            end_position=end_pose.position,
            num_positions=num_poses)
        
        orientations = self.generate_orientations(
            orientation_mode,
            start_orientation=start_pose.orientation,
            end_orientation=end_pose.orientation,
            num_orientations=num_poses
        )
        # Para pasarlo como matriz
        poses = np.concatenate((positions, orientations), axis=1)
        return poses

        # En cada iteracion se crea un elemento Pose
        # return [Pose(pos, ori) for pos, ori in zip(positions, orientations)]

    def generate_positions(self,
            path_type,
            start_position,
            end_position,
            num_positions,
            **kwargs #! IDK
    ):
        """
        Delega la generacion de puntos de posicion a las clases especificas.
        """
        if path_type == "bezier":
            return self.bezier_path.calc_2points_bezier_path(
                start_point=start_position,
                end_point=end_position,
                num_points=num_positions
            )
        elif path_type == "linear":
            return self.simple_path.calc_linear_path(
                start_point=start_position,
                end_point=end_position,
                num_points=num_positions
            )
        elif path_type == "circle":
            mid_point = kwargs["mid_point"]
            return self.simple_path.calc_circle_path(
                start_point=start_position,
                mid_point=mid_point,
                end_point=end_position,
                num_points=num_positions
            )
        else:
            raise ValueError(f"Tipo de trayectoria no soportado {path_type}")
        
    def generate_orientations(self,
            orientation_mode,
            start_orientation,
            end_orientation,
            num_orientations
    ):
        if orientation_mode == "slerp":
            return self.orientation_planner.plan_orientation(
                start_orientation=start_orientation,
                end_orientation=end_orientation,
                num_points=num_orientations
            )
        else:
            raise ValueError(f"Modo de orientacion no soportado {orientation_mode}")
        
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
        if n_points < 2:
            print(f"Error al segmentar trayectoria. Debe tener mas de 2 puntos")
            return path

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

if __name__ == "__main__":
    start_pose = Pose(
        position=np.array([0,0,0]),
        orientation=np.array([0,0,0])
    )

    end_pose = Pose(
        position=np.array([5,5,5]),
        orientation=np.array([1,0,0])
    )

    planner = PathPlanner()

    # Curva de Bezier de ejemplo
    bezier_path = planner.generate_path(
        path_type="bezier",
        start_pose=start_pose,
        end_pose=end_pose,
        #mid_pos=np.array([2, 3, 1]),  # kwargs para Bezier
        orientation_mode="slerp"
    )

    print(f"Cant. Puntos Path Original: {len(bezier_path)}")
    #print(f" Todo la Lista: {bezier_path[:]}")

    # Muestro en pantalla el objeto 5
    print(f"Pose: {bezier_path[5]}")
    print(f"Posicion: {bezier_path[5,:3]}")
    print(f"Orientacion: {bezier_path[5,3:]} \n")

    # Puebo calculo de longitud de segmento
    longitud_path = planner.calc_path_length(bezier_path)
    print(f"Path de Longitud = {longitud_path} unidades")
    
    # Pruebo Segmentacion del Path
    segmented_bezier_path = planner.segmentar_path(bezier_path, num_segments=4)
    print(f"Segmented Bezier Path: \n {segmented_bezier_path} \n")
    print(f"Cant. Puntos Segmented Path = {len(segmented_bezier_path)}")


