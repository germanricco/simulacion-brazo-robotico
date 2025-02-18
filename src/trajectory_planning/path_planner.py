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

    def generate_trajectory(
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
        # poses = np.concatenate((positions, orientations), axis=1)
        # print(f"Poses: {poses}")
        # return poses

        # En cada iteracion se crea un elemento Pose
        return [Pose(pos, ori) for pos, ori in zip(positions, orientations)]

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
    bezier_trajectory = planner.generate_trajectory(
        path_type="bezier",
        start_pose=start_pose,
        end_pose=end_pose,
        #mid_pos=np.array([2, 3, 1]),  # kwargs para Bezier
        orientation_mode="slerp"
    )

    #lista de Objetos Pose
    print(bezier_trajectory[0])
    #para acceder a ella
    print(f" Posicion: \n {bezier_trajectory[5].position}")
    print(f" Orientacion: \n {bezier_trajectory[5].orientation}")

    for pose in bezier_trajectory:
        print(f"Posición: {pose.position}, Orientación: {pose.orientation}")


