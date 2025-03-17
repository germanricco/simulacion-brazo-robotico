import numpy as np
from typing import Callable, Optional
from scipy.interpolate import CubicSpline
import sys
from pathlib import Path
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from trajectory_planning.utils.data_classes import JointConstraints
from trajectory_planning.utils.data_classes import SegmentProfile
from trajectory_planning.utils.data_classes import SegmentConstraints
from trajectory_planning.utils.data_classes import JointState
from trajectory_planning.velocity_profiles import SCurveProfile
from trajectory_planning.velocity_profiles import ConstantVelocityProfile
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

        # Preprocesar path. Separar por articulaciones
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

        n_points = len(path)
        if n_points < 2:
            raise ValueError(f"El path debe tener al menos 2 puntos y tiene {n_points}")

    def _has_discontinuities(self, positions, threshold=0.05):
        """
        Verifica si un path tiene discontinuidades basándose en cambios abruptos
        en la pendiente entre segmentos consecutivos.
        
        Una discontinuidad se define como un cambio grande en la pendiente
        que excede un umbral determinado, considerando las diferencias
        de los segmentos adyacentes.
        
        Argumentos:
            * positions: Array de posiciones articulares
            * threshold: Umbral para considerar un cambio como discontinuidad
                        (valor predeterminado: 0.05 radianes)
        
        Retorna:
            Boolean: True si se detectan discontinuidades, False en caso contrario
        """
        # Si hay menos de 3 puntos, no podemos detectar discontinuidades
        if len(positions) < 3:
            return False
        
        # Calcular las diferencias entre puntos consecutivos
        diffs = np.diff(positions)
        
        # Establecemos un epsilon para evitar divisiones por cero
        eps = 1e-6
        
        # Detectar cambios abruptos de pendiente
        for i in range(len(diffs) - 1):
            # Si hay un cambio de dirección podria ser max/min suave o discontinuidad
            if diffs[i] * diffs[i+1] < 0:
                if i > 0 and i < len(diffs) - 2:
                    second_deriv_before = diffs[i] - diffs[i-1]
                    second_deriv_after = diffs[i+2] - diffs[i+1]
                    # En un max suave segunda derivada mantiene signo
                    if second_deriv_before * second_deriv_after < 0:
                        if (abs(second_deriv_before) > threshold * abs(diffs[i-1]) and 
                            abs(second_deriv_after) > threshold * abs(diffs[i+1])):
                            return True
                else:
                    # Si no tenemos suficiente puntos usamos criterio basado
                    #en magnitud de las pendientes
                    mag_ratio = abs(diffs[i]) / (abs(diffs[i+1]) + eps)
                
                    # Si la proporción es muy diferente de 1.0, es probable una discontinuidad
                    if mag_ratio > (1.0 + threshold * 2) or mag_ratio < (1.0 / (1.0 + threshold * 2)):
                        # Solo si ambas magnitudes son significativas
                        if abs(diffs[i]) > threshold and abs(diffs[i+1]) > threshold:
                            return True
            else:
                # Sin cambio de direccion, verificamos cambios abruptos en magnitud
                if abs(diffs[i]) > eps and abs(diffs[i+1]) > eps:
                    ratio = diffs[i+1] / diffs[i]
                    
                    # Si el ratio se aleja mucho de 1.0, hay una discontinuidad
                    if ratio > (1.0 + threshold) or ratio < (1.0 - threshold):
                        return True
        
        # Verificar también cambios muy abruptos en magnitud de distancia
        for i in range(len(diffs) - 1):
            # Calcular el cambio en magnitud de diferencias
            mag_change = abs(abs(diffs[i+1]) - abs(diffs[i])) / (abs(diffs[i]) + eps)
            
            # Si el cambio relativo en magnitud es muy grande, hay discontinuidad
            if mag_change > threshold * 3:  # Umbral más alto para magnitud
                return True
        
        # No se detectaron discontinuidades significativas
        return False

    def _smooth_path(self, positions, window_size=5):
        """
        Aplica un filtro de media móvil ponderada a una secuencia de posiciones
        
        Argumentos:
            * positions: Array de posiciones articulares 
                    Puede ser (n_points,) o (n_points, n_joints)
            * window_size: Tamaño de la ventana para promediar
        
        Returns:
            Array de posiciones suavizadas con la misma forma que positions
        """
        
        # Crear una ventana de ponderación (triangular)
        weights = np.concatenate([np.arange(1, window_size//2 + 2), 
                                np.arange(window_size//2, 0, -1)])
        weights = weights / np.sum(weights)
        
        # Convertir a array de numpy si no lo es
        positions_array = np.array(positions)
        
        # Manejar tanto arrays 1D como 2D
        if positions_array.ndim == 1:
            # Caso 1D:
            # Guardo posiciones inicial y final
            initial_pos = positions_array[0]
            final_pos = positions_array[-1]

            if len(positions_array) > 2:
                interior_points = positions_array[1:-1]
                padded = np.pad(interior_points, (window_size//2, window_size//2), 'edge')
                smoothed_interior = np.convolve(padded, weights, mode='valid')

                # Reconstruyo path preservando posiciones inicial y final
                smoothed = np.concatenate(([initial_pos], smoothed_interior, [final_pos]))
            else:
                # Si solo tiene 2 puntos, retornar path original
                smoothed = positions_array.copy()
        else:
            # Caso 2D: procesar cada columna
            n_points, n_joints = positions_array.shape
            smoothed = np.zeros_like(positions_array)
            
            for joint in range(n_joints):
                # Extraigo posiciones para esta articulacion
                joint_positions = positions_array[:, joint]
                initial_pos = joint_positions[0]
                final_pos = joint_positions[-1]

                # Aplico suavizado en puntos interiores
                if n_points > 2:
                    interior_points = joint_positions[1:-1]
                    padded = np.pad(interior_points, (window_size//2, window_size//2), 'edge')
                    smoothed_interior = np.convolve(padded, weights, mode="valid")
                    
                    # Reconstruyo trayectoria
                    smoothed[0, joint] = initial_pos
                    smoothed[1:-1, joint] = smoothed_interior
                    smoothed[-1, joint] = final_pos
                else:
                    # Si solo tiene 2 puntos, retornar path original
                    smoothed[:,joint] = joint_positions.copy()
        
        return smoothed

    def _estimate_initial_velocities(self, positions:np.ndarray,
                                     max_velocity: float) -> np.ndarray:
        """
        Estima velocidades iniciales usando diferencias finitas
            
            Argumentos:
                * positions: lista de posiciones de una articulacion [J0, J1, Jn]
                * constraints: restricciones cinematicas

            Retorna:
                Array de velocidades estimadas
        """
        n_points = len(positions)
        estimated_vel = np.zeros(n_points)

        EPSILON = 1e-6 #cte. para comparar pendientes ctes.
        PORCENTAJE_VEL_CTE = 0.7 # Si velocidad es uniforme usamos fraccion de v_max

        # Para puntos intermedios calculo velocidad basado en puntos adyacentes
        for i in range(1, n_points-1):
            # Pendientes hacia adeltante y hacia atras
            prev_pend = positions[i] - positions[i-1]
            next_pend = positions[i+1] - positions[i]

            # Detectar cambios de direccion
            if prev_pend * next_pend < 0:
                estimated_vel[i] = 0
            else:
                # Promedio de pendientes para estimacion centrada
                pendiente_promedio = 0.5 * (prev_pend + next_pend)
                estimated_vel[i] = np.clip(pendiente_promedio, -max_velocity, max_velocity)

                # Para trayectorias uniformes, ajustar la magnitud
                if abs(prev_pend - next_pend) < EPSILON:
                    # Usamos fraccion de velocidad maxima
                    direccion = np.sign(pendiente_promedio)
                    vel_uniforme = direccion * (max_velocity * PORCENTAJE_VEL_CTE)
                    estimated_vel[i] = vel_uniforme

                    # Usamos minimo entre estimacion anterior y esta
                    #if abs(vel_uniforme) < abs(estimated_vel[i]):
                    #    estimated_vel[i] = vel_uniforme
        
        return estimated_vel

    def _generate_segment(self, q0: float, q1: float, v0: float, v1: float,constraints: JointConstraints) -> SegmentProfile:
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
        desplazamiento = abs(q1 - q0)
        delta_v = abs(v1 - v0)
        if desplazamiento < EPSILON:
            # Si no hay desplazamiento
            return self._create_dwell_segment(q0, constraints=constraints)
        else:
            if delta_v < EPSILON:
                return self._create_constant_segment(q0, q1, v1, constraints)
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
            print(f"Error al generar segmento S-Curve. en segmento {self._current_segment_ID}")
            return None

    def _create_constant_segment(self, q0:float, q1:float, v:float, constraints: JointConstraints) -> SegmentProfile:
        """
        Crea un segmento de trayectoria con velocidad constante.

        Argumentos:
            q0: Posicion inicial
            q1: Posicion final
            v: Velocidad constante deseada
            constraints: Restricciones de la articulacion

        Retorna:
            [SegmentProfile]: Perfil de segmento con velocidad constante
        """
        try:
            profile = ConstantVelocityProfile(constraints=constraints)
            profile.plan_trajectory(q0, q1, v)
            duration = profile.duration()

            return SegmentProfile(
                # Metadata
                segment_id = self._current_segment_ID,
                joint_id = self._current_joint_ID,
                profile_type = "constant_velocity",

                constraints=SegmentConstraints(
                        q_start=q0,
                        q_end=q0,
                        v_start=v,
                        v_end=v,
                        jerk_max=constraints.max_jerk,
                        accel_max=constraints.max_acceleration,
                        vel_max=constraints.max_velocity
                    ),

                # Parametros temporales. Todos los tiempos de aceleracion y jerk son cero.
                Tj1 = 0,  # Sin fase de jerk inicial
                Ta = 0,   # Sin fase de aceleración constante
                Tj2 = 0,  # Sin fase de jerk final de aceleración
                Td = 0,   # Sin fase de desaceleración
                Tv = duration, # Tiempo a velocidad constante

                # Función de trayectoria
                trajectory_func = profile._trajectory_function

            )
        except ValueError as e:
            print(f"Error al generar segmento de velocidad constante: {e}")
            return None

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
            return None

    def _create_joint_profile(self, positions: np.ndarray, constraints: JointConstraints) -> list[SegmentProfile]:
        """
        Crea perfil de velocidades completo para una articulación individual
        
        Argumentos:
            * positions: lista de posiciones angulares
            * constraints: objeto JointConstraints particular de articulacion

        Retorna:
            segments = list[SegmentProfile]
        """

        #! Agregar checkeo de discontinuidades
        working_positions = positions

        # Estimar velocidades en via_points
        estimated_velocities = self._estimate_initial_velocities(working_positions, constraints.max_velocity)
        print(f"Estimated Velocities  = {estimated_velocities}")

        segments = []
        # Generar lista de segmentos para todo el perfil
        for i in range(len(positions) - 1):
            self._current_segment_ID = i
            # Obtengo posiciones y velocidades de siguiente segmento
            q0 = working_positions[i]
            q1 = working_positions[i+1]
            v0 = estimated_velocities[i]
            v1 = estimated_velocities[i+1]

            segment = self._generate_segment(q0, q1, v0, v1, constraints=constraints)
            segments.append(segment)
            
        return segments

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
        [-0.78,  0.69, -0.93,  1.34,  0.81, -1.24],
        [-0.73,  0.52, -0.66,  1.42,  0.74, -1.37],
        [-0.66,  0.4,  -0.48,  1.47,  0.67, -1.44],
        [-0.6,   0.3,  -0.35,  1.5,   0.6,  -1.49],
        [-0.52,  0.22, -0.24,  1.53,  0.52, -1.52],
        [-0.44,  0.15, -0.16,  1.55,  0.44, -1.55],
        [-0.35,  0.09, -0.09,  1.56,  0.35, -1.56],
        [-0.26,  0.05, -0.05,  1.57,  0.26, -1.57],
        [-0.16,  0.02, -0.02,  1.57,  0.16, -1.57],
        [-0.05,  0,   -0,    1.57,  0.05, -1.57],
        [0.05,  0,  -0,  -1.57,  0.05,  1.57],
        [0.16,  0.02, -0.02, -1.57,  0.16 , 1.57],
        [0.26,  0.05, -0.05, -1.57,  0.26,  1.57],
        [0.35,  0.09, -0.09, -1.56,  0.35,  1.56],
        [0.44,  0.15, -0.16, -1.55,  0.44,  1.55],
        [0.52,  0.22, -0.24, -1.53,  0.52,  1.52],
        [0.6 ,  0.3 , -0.35, -1.5,   0.6,   1.49],
        [0.66,  0.4 , -0.48, -1.47,  0.67,  1.44],
        [0.73,  0.52, -0.66, -1.42,  0.74,  1.37],
        [0.78, 0.69, -0.93, -1.34,  0.81,  1.24]
    ])

    # Tomo solo una columna (articulacion) a la vez
    #joint_path = joints_path[:,1:2]
    joint_path = np.array([
        [0],
        [0.5],
        [1],
        [1.5],
        [2],
        [2],
        [2],
        [1.5],
        [1],
        [0.5],
        [0]
    ])

    # Configurar restricciones para 2 articulaciones
    joints_constraints = {
        0: JointConstraints(max_velocity=1, max_acceleration=1, max_jerk=3),
        #1: JointConstraints(max_velocity=1, max_acceleration=1, max_jerk=3)
    }

    # Instancio
    plotter = TrajectoryPlotter()
    planner = TrajectoryPlanner(joints_constraints)

    # Proceso el path
    planner.process_joints_path(joints_path=joint_path)

    # Plot
    plotter.add_joint_trajectory(0, planner.joint_profiles[0])
    #plotter.add_joint_trajectory(1, planner.joint_profiles[1])
    plotter.plot(0)
    #plotter.plot(1)
    plotter.show()