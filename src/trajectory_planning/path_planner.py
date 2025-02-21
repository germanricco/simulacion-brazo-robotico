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
from utils.auxiliary_methods import verificar_pose

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
            * start_pose: Pose inicial. Permite orientacion en Euler y Quaternions
            * end_pose: Pose final.
            * num_poses (int): numero de poses a interpolar
            * orientation_mode (str): "slerp" (interpolacion)

        Retorna:
            List: Lista de poses a lo largo de la trayectoria.
                de la forma (n,6) = [x,y,z,qx,qy,qz,qw]
        """

        # Verifico validez de poses de entrada
        if verificar_pose(start_pose) and verificar_pose(end_pose):
            if isinstance(start_pose, np.ndarray):
                # Extraigo posicion
                start_pos = start_pose[:3]
                end_pos = end_pose[:3]
                # Extraigo orientaciones
                start_ori = start_pose[3:]
                end_ori = end_pose[3:]

                positions = self.generate_positions(
                    path_type=path_type,
                    start_position=start_pos,
                    end_position=end_pos,
                    num_positions=num_poses)

                #Nota: el metodo generate_orientations() funciona unicamente con angulos de Euler

                orientations = self.generate_orientations(
                    orientation_mode,
                    start_orientation=start_ori,
                    end_orientation=end_ori,
                    num_orientations=num_poses)
        else:
            print(f"objeto Pose todavia no soportado")
            return None

        # Para pasarlo como matriz
        poses = np.concatenate((positions, orientations), axis=1)
        return poses


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
        """
        Delega la generacion de orientaciones
        """
        if orientation_mode == "slerp":
            return self.orientation_planner.plan_orientation(
                start_orientation=start_orientation,
                end_orientation=end_orientation,
                num_points=num_orientations
            )
        else:
            raise ValueError(f"Modo de orientacion no soportado {orientation_mode}")

if __name__ == "__main__":

    print(f"\n --Prueba de PathPlanner-- \n")

    planner = PathPlanner()
    start_pose = np.array([0,0,0,0,0,0])
    end_pose = np.array([5,5,0,90,0,0])

    # Curva de Bezier de ejemplo
    bezier_path = planner.generate_path(
        path_type="bezier",
        start_pose=start_pose,
        end_pose=end_pose,
        #mid_pos=np.array([2, 3, 1]),  # kwargs para Bezier
        orientation_mode="slerp",
        num_poses=100
    )

    print(f"Cant de Puntos Original: {len(bezier_path)}")
    #print(f" Todo la Lista: {bezier_path[:]}")

    # Muestro en pantalla el objeto 5
    print(f"\nAnalizamos un punto:")
    print(f"Pose: {bezier_path[5]}")
    print(f"Posicion: {bezier_path[5,:3]}")
    print(f"Orientacion: {bezier_path[5,3:]} \n")
