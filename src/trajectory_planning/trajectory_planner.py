import numpy as np
from typing import Callable, Optional
from scipy.interpolate import CubicSpline
import sys
from pathlib import Path
import matplotlib.pyplot as plt

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from trajectory_planning.utils.data_classes import JointConstraints
from trajectory_planning.utils.data_classes import SegmentProfile
from trajectory_planning.utils.data_classes import SegmentConstraints
from trajectory_planning.utils.data_classes import JointState
from trajectory_planning.velocity_profiles import SCurveProfile
from trajectory_planning.velocity_profiles import ZeroMotionProfile
from trajectory_planning.polinomios import QuinticPolinomial

class TrajectoryPlanner():
    """
    Coordina multiples articulaciones y puntos de paso.
    """
    def __init__(self, joints_constraints: dict[int, JointConstraints]):
        """
        Argumentos:
            * joints_constraints: Diccionario con restriccion por articulacion
                {id: JointConstraints}
        """
        self.joints_constraints = joints_constraints
        self.quintic = QuinticPolinomial()
        
        self.joint_profiles: dict[int, list[SegmentProfile]] = {} # joint_id: list(segmentos)
        self.full_trajectory: dict[int, list[JointState]] = {}
        self.sync_timeline: Optional[np.ndarray] = None

        self._current_joint_ID: int
        self._current_segment_ID: int

    def process_joints_path(self, joints_path: np.ndarray) -> None:
        """
        Procesa la matriz completa de trayectorias articulares creando
        y agregando joint_profiles como list[SegmentProfile]

        Argumentos:
            * joints_path: Matriz (m x n) donde:
                - m: Numero de puntos de trayectoria
                - n: Numero de articulaciones (debe coincidir con joint_constraints)
        """
        self._validate_joints_path(joints_path)

        # Separar por articulaciones y crear perfiles
        for joint_id in self.joints_constraints.keys():
            # Actualizo joint ID
            self._current_joint_ID = joint_id
            joint_positions = joints_path[:, joint_id]  # Toma una columna a la vez
            
            # Crea perfil de velocidades para esa articulacion
            self.joint_profiles[joint_id] = self._create_joint_profile(
                positions=joint_positions,
                constraints=self.joints_constraints[joint_id]
            )

        #! Sincronizar tiempos entre articulaciones
        #self._synchronize_profiles()
        #! Optimizacion global
        #self._optimize_transitions()

    # ================= Metodos Internos ==================
    def _validate_joints_path(self, path: np.ndarray) -> None:
        """Valida la estructura de la matriz joints_path"""
        if path.ndim != 2:
            raise ValueError("joints_path debe ser matriz 2D")
            
        n_joints = len(self.joints_constraints)
        if path.shape[1] != n_joints:
            raise ValueError(f"Número de columnas en joints_path ({path.shape[1]}) no coincide con articulaciones definidas ({n_joints})")
    

    def _create_joint_profile(self, positions: np.ndarray, constraints: JointConstraints) -> list[SegmentProfile]:
        """
        Crea perfil de velocidades completo para una articulación individual
        
        Argumentos:
            * positions: lista de posiciones angulares
            * constraints: objeto JointConstraints particular de articulacion

        Retorna:
            segments = list[SegmentProfile]
        """
        segments = []
    
        # Calcular velocidades iniciales estimadas
        estimated_velocities = self._estimate_initial_velocities(positions, constraints)
        print(f"Estimated Velocities = {estimated_velocities}")

        # Generar lista de segmentos para todo el perfil
        for i in range(len(positions)-1):
            self._current_segment_ID = i
            # Obtengo posiciones y velocidades de siguiente segmento
            q0, q1 = positions[i], positions[i+1]
            v0, v1 = estimated_velocities[i], estimated_velocities[i+1]

            segment = self.generate_segment(q0, q1, v0, v1, constraints=constraints)
            segments.append(segment)
            
        return segments
    

    def _estimate_initial_velocities(self, positions: np.ndarray, constraints: JointConstraints) -> np.ndarray:
        """
        Estima velocidades iniciales usando splines quintics y tiempo basado en longitud
        de la trayectoria.
            
            Argumentos:
                * positions: lista de posiciones de una articulacion [J0, J1, Jn]
                * constraints: restricciones cinematicas
        """
        n = len(positions)
        if n < 2:
            return np.zeros_like(positions)
        
        segment_times = []

        for i in range(n-1):
            # Calcular longitud acumulada de segmento
            delta_q = abs(positions[i+1] - positions[i])
            # Calcular tiempo teorico (sin considerar jerk) de segmento
            t = self._calculate_segment_time(delta_q, constraints)
            segment_times.append(t)

        # 2. Generar vector temporal acumulado
        time_vector = np.zeros(n)
        for i in range(1, n):
            # Agrego tiempo anterior mas tiempo de nuevo
            time_vector[i] = time_vector[i-1] + segment_times[i-1]

        # 3. Crear spline Quintic con condiciones de frontera C2
        #! Aceleracion = 0 en bordes
        cs = CubicSpline(time_vector, positions, bc_type=((2, 0), (2, 0)))

        # 4. Calcular velocidades en cada punto
        velocities = cs(time_vector, 1)  # Primera derivada
        velocities = np.clip(velocities, -constraints.max_velocity, constraints.max_velocity)

        for i in range(1, n):
            delta_q = abs(positions[i] - positions[i-1])
            max_vel = min(
                np.sqrt(2 * constraints.max_acceleration * delta_q),
                constraints.max_velocity
            )
            velocities[i] = np.clip(velocities[i], -max_vel, max_vel)

        velocities[0] = 0
        velocities[n-1] = 0
        return velocities
    
    def generate_segment(self, q0: float, q1: float, v0: float, v1: float, constraints: JointConstraints) -> SegmentProfile:
        """
        Crea un segmento de trayectoria entre dos puntos de paso.

        Argumentos:
            q0: Posicion inicial
            q1: Posicion final
            v0: Velocidad inicial
            v1: Velocidad final
            constraints: Restricciones de la articulacion

        Retorna:
            [SegmentProfile]: Perfil de segmento generado
        """
        EPSILON = 1e-6
        dist = abs(q1 - q0)
        if dist < EPSILON:
            return self._create_dwell_segment(q0, constraints=constraints)
        else:
            return self._create_scurve_segment(q0, q1, v0, v1, constraints)

    def _create_scurve_segment(self, q0:float, q1:float, v0:float, v1:float, constraints: JointConstraints) -> SegmentProfile:
        """
        Crea un segmento de trayectoria entre dos puntos de paso.

        Argumentos:
            q0: Posicion inicial
            q1: Posicion final
            v0: Velocidad inicial
            v1: Velocidad final
            constraints: Restricciones de la articulacion

        Retorna:
            [SegmentProfile]: Perfil de segmento generado
        """
        try:
            # Crear perfil de velocidad
            profile = SCurveProfile(constraints=constraints)
            # Calcular parametros y obtener trayectoria
            profile.plan_trajectory(q0, q1, v0, v1)

            return SegmentProfile(
                # MetaData
                segment_id = self._current_segment_ID,
                joint_id = self._current_joint_ID,
                profile_type = "scurve",
                # Restricciones del segmento
                constraints=SegmentConstraints(
                    q_start=q0,
                    q_end=q1,
                    v_start=v0,
                    v_end=v1,
                    jerk_max=constraints.max_jerk,
                    accel_max=constraints.max_acceleration,
                    vel_max=constraints.max_velocity
                ),
                # Parametros temporales de trayectoria
                Tj1 = profile._parameters[0],
                Ta = profile._parameters[1],
                Tj2 = profile._parameters[2],
                Td = profile._parameters[3],
                Tv = profile._parameters[4],
                # Funcion de trayectoria
                trajectory_func= profile._trajectory_function
            )

        except ValueError as e:
            print(f"Error al generar segmento de movimiento.")

    def _create_dwell_segment(self, q0:float, constraints:JointConstraints,
                              dwell_time:float=2) -> SegmentProfile:
        """
        Crea un segmento de trayectoria de espera (sin movimiento)

        Argumentos:
            q0: Posicion inicial
            constraints: Restricciones de la articulacion

        Retorna:
            [SegmentProfile]: Perfil de segmento generado
        """
        try:
            profile = ZeroMotionProfile(constraints=constraints)
            profile.plan_trajectory(q0, duration=dwell_time)

            return SegmentProfile(
                    # MetaData
                    segment_id = self._current_segment_ID,
                    joint_id = self._current_joint_ID,
                    profile_type = "dwell",
                    # Restricciones del segmento
                    constraints=SegmentConstraints(
                        q_start=q0,
                        q_end=q0,
                        v_start=0,
                        v_end=0,
                        jerk_max=constraints.max_jerk,
                        accel_max=constraints.max_acceleration,
                        vel_max=constraints.max_velocity
                    ),
                    # Parametros temporales de trayectoria
                    Tj1 = 0,
                    Ta = 0,
                    Tj2 = 0, 
                    Td = 0,
                    Tv = dwell_time, #! Tiempo de espera. V=cte=0
                    # Funcion de trayectoria
                    trajectory_func= profile._trajectory_function
                )
        except ValueError as e:
            print(f"Error al generar segmento estatico.")
        

    def _calculate_segment_time(self, delta_q: float, constraints: JointConstraints) -> float:
        """Calcula tiempo mínimo para un segmento usando ecuaciones S-Curve"""
        # Tiempo sin jerk (modelo trapezoidal)
        t_accel = (constraints.max_velocity - 0) / constraints.max_acceleration
        dist_accel = 0.5 * constraints.max_acceleration * t_accel**2
        if delta_q <= 2 * dist_accel:  # Caso sin fase de velocidad constante
            t_trap = 2 * np.sqrt(delta_q / constraints.max_acceleration)
        else:
            t_trap = (delta_q / constraints.max_velocity) + (constraints.max_velocity / constraints.max_acceleration)

        # Tiempo con jerk (modelo S-Curve)
        t_jerk = (constraints.max_acceleration / constraints.max_jerk)
        t_scurve = (delta_q / constraints.max_velocity) + 1.5 * t_jerk

        return max(t_trap, t_scurve)

if __name__ == "__main__":
    from visualization.trajectory_plotter import TrajectoryPlotter

    # Joints_path de prueba
    joints_path = np.array([
        [0],
        [2],
        [4],
        [3],
        [6],
        [4],
        [0]
    ])

    # Configurar restricciones para 2 articulaciones
    joints_constraints = {
        0: JointConstraints(max_velocity=1, max_acceleration=1, max_jerk=3),
        #1: JointConstraints(max_velocity=1, max_acceleration=1, max_jerk=3)
    }

    plotter = TrajectoryPlotter()

    planner = TrajectoryPlanner(joints_constraints)
    planner.process_joints_path(joints_path=joints_path)

    plotter.add_joint_trajectory(0, planner.joint_profiles[0])

    plotter.plot(0)

    plotter.show()