"""
Creditos:
https://github.com/nameofuser1/py-scurve/tree/master
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple
from typing import Callable
import numpy as np
import sys
from pathlib import Path
# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))
from trajectory_planning.utils.data_classes import JointConstraints
from trajectory_planning.utils.data_classes import POSITION_ID
from trajectory_planning.utils.data_classes import SPEED_ID
from trajectory_planning.utils.data_classes import ACCELERATION_ID


OPTIMIZER_THRESHOLD = 0.01
EPSILON = 0.0001

class TrajectoryProfile(ABC):
    """
    Interfaz abstracta para perfiles de velocidad.
    Gestiona restricciones y parametros comunes
    """
    def __init__(self, constraints: JointConstraints):
        self._constraints = constraints
        self._parameters = None
        self._trajectory_function = None
        
        self._inverted = False

    @property
    def constraints(self) -> JointConstraints:
        return self._constraints
    
    @constraints.setter
    def constraints(self, value: JointConstraints):
        self._validate_constraints(value)
        self._constraints = value
        self._clear_parameters()

    @abstractmethod
    def plan_trajectory(self, q0, q1, v0, v1):
        """
        Calcula los parametros de la trayectoria entre dos puntos y almacena
        la trayectoria en funcion del tiempo

        Argumentos:
            * q0: Posicion inicial
            * q1: Posicion final
            * v0: Velocidad inicial
            * v1: Velocidad final
        """
        pass

    @abstractmethod
    def get_state(self, t: float) -> np.ndarray:
        """
        Devuelve el estado de la trayectoria en el tiempo t
        
        Returns:
            np.ndarray: [posición, velocidad, aceleración]
        """
        if not self._trajectory_function:
            raise ValueError("Primero debe calcularse la trayectoria con plan_trayectory()")
        return self._trajectory_function(t)
    
    #! Metodos internos
    def _validate_constraints(self, constraints: JointConstraints):
        """Valida restricciones al asignarlas"""
        if constraints.max_jerk <= 0:
            raise ValueError("El jerk máximo debe ser positivo")
        #! Más validaciones según necesidad
        
    def _clear_parameters(self):
        """Resetea parámetros al cambiar restricciones"""
        self._parameters = None


class SCurveProfile(TrajectoryProfile):
    """
    Implementacion del perfil S-Curve
    """

    def plan_trajectory(self, q0:float, q1:float, v0:float, v1:float):
        #! Agregar validacion de condiciones de borde q0,q1,v0,v1
        try:
            #Transformacion de signos inicial
            q0,q1,v0,v1 = self._sign_transforms(q0, q1, v0, v1)

            # Calculo de parametros caracteristicos
            self._parameters = self._compute_parameters(q0, q1, v0, v1)
            
            # Creo la trayectoria en funcion del tiempo
            self._trajectory_function = self._get_trajectory_function(q0, q1, v0, v1)

        except Exception as e:
            raise ValueError(f"Error S-Curve: {str(e)}") from e
    

    def get_state(self, t):
        return super().get_state(t)


    def _scurve_check_possibility(self, q0, q1, v0, v1):
        """
        Verifica si una trayectoria es posible. Si no ValueError

        pag 80-81. of 'Trajectory planning for automatic machines and robots(2008)
        """
        dv = np.abs(v1 - v0)
        dq = np.abs(q1 - q0)

        a_max = self.constraints.max_acceleration
        j_max = self.constraints.max_jerk

        time_to_reach_max_a = a_max/j_max
        time_to_set_set_speeds = np.sqrt(dv/j_max)

        #Eq. 3.17
        Tj = min(time_to_reach_max_a, time_to_set_set_speeds)

        # Si son iguales la aceleracion alcanza su maximo y existe segmento con jerk=0
        #Eq. 3.18
        if Tj == time_to_reach_max_a:
            return dq > 0.5*(v0 + v1)*(Tj+dv/a_max)
        elif Tj < time_to_reach_max_a:
            return dq > Tj*(v0+v1)
        else:
            raise ValueError("Something went wrong")


    def _compute_maximum_speed_reached(self, q0, q1, v0, v1):
        """
        Case 1. V_lim = V_max
        """
        # Tomo variables internas
        v_max = self.constraints.max_velocity
        a_max = self.constraints.max_acceleration
        j_max = self.constraints.max_jerk

        #Eq 3.19  Acceleration period
        if (v_max-v0)*j_max < a_max**2:
            #Eq 3.21 a_max is not reached
            Tj1 = np.sqrt((v_max-v0)/j_max)
            Ta = 2*Tj1
        else:
            #Eq 3.22 a_max is reached
            Tj1 = a_max/j_max
            Ta = Tj1 + (v_max-v0)/a_max

        #Eq 3.20 Deceleration period
        if (v_max-v1)*j_max < a_max**2:
            # Eq. 3.23 a_min is not reached
            Tj2 = np.sqrt((v_max-v1)/j_max)
            Td = 2*Tj2
        else:
            #Eq 3.24 a_min is reached
            Tj2 = a_max/j_max
            Td = Tj2 + (v_max-v1)/a_max

        # Determino duracion de segmento de V=cte.
        #Eq 3.25
        Tv = (q1-q0)/v_max - (Ta/2)*(1+v0/v_max)-(Td/2)*(1+v1/v_max)

        # Si Tv < 0 -> V_lim < V_max y debemos pasar a caso 2.
        if Tv < 0:
            raise ValueError("Maximum velocity is not reached")

        return Tj1, Ta, Tj2, Td, Tv


    def _compute_maximum_speed_not_reached(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Case 2. V_lim < V_max
        No esta presente el segmento de V-cte.
        """
        # Assuming that a_max/a_min is reached
        #Eq 3.26a
        Tj1 = Tj2 = Tj = a_max/j_max
        Tv = 0

        #Eq 3.27
        v = (a_max**2)/j_max
        delta = ((a_max**4)/(j_max**2)) + 2*((v0**2)+(v1**2)) +\
            a_max*(4*(q1-q0)-2*(a_max/j_max)*(v0+v1))

        #Eq 3.26b y 3.26c
        Ta = (v - 2*v0 + np.sqrt(delta))/(2*a_max)
        Td = (v - 2*v1 + np.sqrt(delta))/(2*a_max)

        if (Ta - 2*Tj < EPSILON) or (Td - 2*Tj < EPSILON):
            # En este caso es conveniente obtener los parametros con una solucion aprox.
            raise ValueError("Max/min acceleration is not reached. Failed to plan trajectory")

        return Tj1, Ta, Tj2, Td, Tv
    

    def _scurve_search_planning(self, q0, q1, v0, v1,
                                l=0.99, max_iter=500,  dt_thresh=0.01, T=None):
        """
        Método para encontrar parámetros de trayectoria S con aceleración ajustada.
        Es decir cuando a_lim < a_max siendo a_lim desconocida
        """
        v_max = self.constraints.max_velocity
        a_max = self.constraints.max_acceleration
        j_max = self.constraints.max_jerk

        current_a_max = a_max
        iteration = 0

        while iteration < max_iter and current_a_max > 1e-9:
            try:
                # 1. Calcular parámetros base con velocidad maxima no alcanzada
                Tj1, Ta, Tj2, Td, Tv = self._compute_maximum_speed_not_reached(
                    q0, q1, v0, v1, v_max, current_a_max, j_max)

                # 2. Manejar casos especiales inmediatamente
                if Ta < 0 or Td < 0:
                    return self._handle_special_cases(q0, q1, v0, v1, Tj1, Ta, Tj2, Td, Tv)
                
                # 3. Validar condiciones de aceleración máxima
                Tj = current_a_max / j_max
                if (abs(Ta - 2*Tj) < 1e-9) or (abs(Td - 2*Tj) < 1e-9):
                    raise ValueError("Ajuste necesario")

                # 4. Validación de tiempo total si es requerido
                if T is not None and abs(T - (Ta + Td + Tv)) > dt_thresh:
                    raise ValueError("Ajuste temporal necesario")

                print(f"Solución encontrada con a_max = {current_a_max:.4f}")
                return Tj1, Ta, Tj2, Td, Tv

            except ValueError as e:
                # 5. Reducción adaptativa de a_max con límite inferior seguro
                current_a_max = max(current_a_max * l, 1e-9)
                iteration += 1

        # 6. Último intento con manejo especial de casos
        try:
            return self._handle_special_cases(q0, q1, v0, v1, 0, 0, 0, 0, 0)
        except ValueError:
            raise ValueError(f"No se encontró solución en {iteration} iteraciones. Último a_max: {current_a_max:.4e}")
            
        
    def _handle_special_cases(self, q0, q1, v0, v1, Tj1, Ta, Tj2, Td, Tv):
        """
        Maneja casos donde Ta o Td son negativos, es decir cuando solo se necesita 1 fase
        de aceleracion/desaceleracion.
        """
        delta_q = q1 - q0
        sum_v = v0 + v1

        j_max = self.constraints.max_jerk
        
        # Caso 1: Solo desaceleración (v0 > v1) Ta<0
        if v0 > v1:
            if abs(sum_v) < 1e-9:
                raise ValueError("Suma de velocidades inválida")
            
            Td = 2 * delta_q / sum_v
            sqrt_term = np.sqrt(j_max * (j_max * delta_q**2 + (v1 - v0) * sum_v**2))
            Tj2 = (j_max * delta_q - sqrt_term) / (j_max * sum_v)
            
            if Tj2 < 1e-9 or Td < 2*Tj2:
                raise ValueError("Parámetros inválidos")
            
            return 0.0, 0.0, Tj2, Td, 0.0

        # Caso 2: Solo aceleración (v1 > v0) Td<0
        elif v1 > v0:
            if abs(sum_v) < 1e-9:
                raise ValueError("Suma de velocidades inválida")
            
            Ta = 2 * delta_q / sum_v
            sqrt_term = np.sqrt(j_max * (j_max * delta_q**2 + (v0 - v1) * sum_v**2))
            Tj1 = (j_max * delta_q - sqrt_term) / (j_max * sum_v)
            
            if Tj1 < 1e-9 or Ta < 2*Tj1:
                raise ValueError("Parámetros inválidos")
            
            return Tj1, Ta, 0.0, 0.0, 0.0

        raise ValueError("Caso especial no manejable")
    

    def _compute_parameters(self, q0, q1, v0, v1):
        """
        Calcula los parametros de la trajectorya s-curve.
            Tj1     --- non-zero constant jerk period while accelerating
            Ta      --- total acceleration period time
            Tj2     --- non-zero constant jerk period while decelerating
            Td      --- total deceleration time
            Tv      --- constant speed time

        Retorna:
            * np.array: lista con parametros ([Tj1, Ta, Tj2, Td, Tv])
        """
        # Si la trayectoria es posible
        if self._scurve_check_possibility(q0, q1, v0, v1):
            # Asumiendo que se alcanza v_max y a_max
            try:
                Tj1, Ta, Tj2, Td, Tv =\
                    self._compute_maximum_speed_reached(q0, q1, v0, v1)
            except ValueError as e:
                # Error si Tv<0
                print(f"Error. {e}")
                try:
                    Tj1, Ta, Tj2, Td, Tv =\
                        self._scurve_search_planning(q0, q1, v0, v1)
                except ValueError as e:
                    # Error si no se encontro solucion iterando
                    print(f"Error. {e}")

            return np.asarray([Tj1, Ta, Tj2, Td, Tv], dtype=np.float32)

        else:
            print(f"Error al calcular parametros. S-Curve")
            raise TypeError("Trajectory is not feasible")


    def _sign_transforms(self, q0, q1, v0, v1):
        """
        Transforma los signos para ser capaz de calcular la trayectoria con q1 < q0

        Look at 'Trajectory planning for automatic machines and robots(2008)'
        (pag. 90)
        """
        v_max = self.constraints.max_velocity
        a_max = self.constraints.max_acceleration
        j_max = self.constraints.max_jerk

        # Asumimos simetria
        v_min = -v_max
        a_min = -a_max
        j_min = -j_max

        s = np.sign(q1-q0)
        if s == -1:
            self._inverted = True

        vs1 = (s+1)/2
        vs2 = (s-1)/2

        #Eq 3.31 y 3.32
        _q0 = s*q0
        _q1 = s*q1
        _v0 = s*v0
        _v1 = s*v1
        
        _v_max = vs1*v_max + vs2*v_min
        _a_max = vs1*a_max + vs2*a_min
        _j_max = vs1*j_max + vs2*j_min

        # Actualizo restricciones
        self.constraints.max_velocity = _v_max
        self.constraints.max_acceleration = _a_max
        self.constraints.max_jerk = _j_max

        return _q0, _q1, _v0, _v1


    def _point_sign_transform(self, q0, q1, p):
        """
        Transforms point back to the original sign
        """
        # NOTA. Cuando q1<q0. En _sign_transforms se inviertieron los signos y q1>q0.
        # Por ello en esta funcion se invierten los signos de nuevo.
        if not self._inverted:
            s = np.sign(q1-q0)
            return s*p
        else:
            return -p


    def _compute_trajectory(self, q0, q1, v0, v1):
        """
        Calcula caracteristicas clave y crea  funcion trajectory = f(t)
        """
        
        # Obtengo parametros almacenados en variables internas
        Tj1 = self._parameters[0]
        Ta = self._parameters[1]
        Tj2 = self._parameters[2]
        Td = self._parameters[3]
        Tv = self._parameters[4]

        j_max = self.constraints.max_jerk

        # Calculo caracteristicas clave
        T = Ta + Td + Tv
        a_lim_a = j_max*Tj1
        a_lim_d = -j_max*Tj2
        v_lim = v0 + (Ta-Tj1)*a_lim_a
        
        def trajectory(t):
            #
            #Returns numpy array with shape (3,) which contains acceleration,
            #    speed and position for a given time t
            #
            # Acceleration phase
            if 0 <= t < Tj1:
                a = j_max*t
                v = v0 + j_max*(t**2)/2
                q = q0 + v0*t + j_max*(t**3)/6

            elif Tj1 <= t < Ta - Tj1:
                a = a_lim_a
                v = v0 + a_lim_a*(t-Tj1/2)
                q = q0 + v0*t + a_lim_a*(3*(t**2) - 3*Tj1*t + Tj1**2)/6

            elif Ta-Tj1 <= t < Ta:
                tt = Ta - t

                a = j_max*tt
                v = v_lim - j_max*(tt**2)/2
                q = q0 + (v_lim+v0)*Ta/2 - v_lim*tt + j_max*(tt**3)/6

            # Constant velocity phase
            elif Ta <= t < Ta + Tv:
                a = 0
                v = v_lim
                q = q0 + (v_lim+v0)*Ta/2 + v_lim*(t-Ta)

            # Deceleration phase
            elif T - Td <= t < T-Td+Tj2:
                tt = t-T+Td

                a = -j_max*tt
                v = v_lim - j_max*(tt**2)/2
                q = q1 - (v_lim+v1)*Td/2 + v_lim*tt -\
                    j_max*(tt**3)/6

            elif T-Td+Tj2 <= t < T-Tj2:
                tt = t-T+Td

                a = a_lim_d
                v = v_lim + a_lim_d*(tt-Tj2/2)
                q = q1 - (v_lim+v1)*Td/2 + v_lim*tt +\
                    a_lim_d*(3*(tt**2) - 3*Tj2*tt + Tj2**2)/6

            elif T-Tj2 <= t < T:
                tt = T-t

                a = -j_max*tt
                v = v1 + j_max*(tt**2)/2
                q = q1 - v1*tt - j_max*(tt**3)/6

            else:
                a = 0
                v = v1
                q = q1

            #Creo y posiciono el punto segun IDs
            point = np.zeros((3,), dtype=np.float32)
            point[ACCELERATION_ID] = a
            point[SPEED_ID] = v
            point[POSITION_ID] = q

            return point

        return trajectory
    

    def _get_trajectory_function(self, q0, q1, v0, v1):
        """
        Retorna trayectoria en funcion del tiempo, modificando signos para casos en los que
        q1<q0
        """
        # 1. Calcula la trayectoria en funcion del tiempo con los parametros
        traj_func = self._compute_trajectory(q0, q1, v0, v1)

        # 2. Modifica signos de puntos de la trayectoria
        def sign_back_transformed(t):
            return self._point_sign_transform(q0, q1, traj_func(t))

        # Retorna la trayectoria final
        return sign_back_transformed


class ZeroMotionProfile(TrajectoryProfile):
    """
    Perfil de trayectoria para mantener una articulacion estatica durante un tiempo determinado

    Genera una trayectoria de posicion constante y velocidades/aceleraciones nulas. Para
    periodos de espera entre movmiientos o articulaciones inactivas durante un segmento.

    Argumentos:
        constraints (JointConstraints): Restricciones de la articulacion
    """
    def __init__(self, constraints: JointConstraints):
        super().__init__(constraints)
        self._duration = 0.0
        self._trajectory_function = None

    def plan_trajectory(self, q:float, duration:float):
        """
        Configura los parametros de la trayectoria estatica

        Argumentos:
            * q: Posicion a mantener
            * duration: Tiempo de espera
        """
        # Verifico duracion y la guardo
        if duration <= 0:
            raise ValueError("La duración debe ser positiva")
        self._duration = duration
        print(f"Duracion de segmento estatico: {self._duration}")

        # Calculo la trayectoria
        self._trajectory_function = self._build_static_trajectory(q)
    
    def get_state(self, t):
        return super().get_state(t)
    
    def _build_static_trajectory(self, q: float) -> Callable[[np.ndarray], np.ndarray]:
        """
        Construye una función de trayectoria vectorizada y compatible con el sistema
        
        Args:
            q: Posición a mantener (constante)
            
        Returns:
            Función que mapea array de tiempos a estados [posición, velocidad, aceleración]
        """
        def trajectory(t: np.ndarray) -> np.ndarray:
            # Manejar tanto escalares como arrays
            is_scalar = np.isscalar(t)
            t_array = np.asarray([t]) if is_scalar else np.asarray(t)
            
            # Crear array de estados (N,3)
            states = np.zeros((t_array.size, 3), dtype=np.float32)
            states[:, POSITION_ID] = q  # q es constante
            
            # Retornar según tipo de entrada
            return states[0] if is_scalar else states
        
        return trajectory





if __name__ == "__main__":

    # Creo restricciones para 1 articulacion
    joint_1_constraints = JointConstraints(max_velocity=5,
                                           max_acceleration=10,
                                           max_jerk=30)

    s_curve = SCurveProfile(constraints=joint_1_constraints)

    print(f"\n TEST CALCULO DE PARAMETROS\n")

    print(f"-- Example 3.9 --")
    print("Tested 'compute_max_speed_reached' \n")
    q0 = 0
    q1 = 10
    v0 = 1
    v1 = 0
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")
    
    print(f"\n -- Example 3.10 --")
    print("Tested 'compute_max_speed_not_reached' \n")
    # Actualizo restricciones cinematicas
    s_curve.constraints = JointConstraints(max_velocity=10,
                                           max_acceleration=10,
                                           max_jerk=30)
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")


    print(f"\n -- Example 3.11 --")
    print("Tested 'max_acceleration_not_reached' \n")
    # Mismas restricciones pero cambio condiciones de borde
    q0 = 0
    q1 = 10
    v0 = 7
    v1 = 0
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")


    print(f"\n -- Example 3.12 --")
    print("Tested 'max_acceleration_not_reached and Special Case 1 (Ta<0)' \n")
    q0 = 0
    q1 = 10
    v0 = 7.5
    v1 = 0
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")


    #! Leer Bibliografia para este ultimo caso. Esta MAL
    print(f"\n -- Example 3.13 --")
    print("")
    s_curve.constraints = JointConstraints(max_velocity=10,
                                           max_acceleration=20,
                                           max_jerk=30)
    q0 = 0
    q1 = 10
    v0 = 0
    v1 = 2
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")
    print(f"Ultima Trayectoria:\n {s_curve._trajectory_function}")


    print(f"\n -- Test Propios --")
    q0 = 2
    q1 = 0
    v0 = 0
    v1 = 0
    s_curve.plan_trajectory(q0, q1, v0, v1)
    print(f"Ta={s_curve._parameters[1]}, Tv={s_curve._parameters[4]}, Td={s_curve._parameters[3]}, Tj1={s_curve._parameters[0]}, Tj2={s_curve._parameters[2]} ")

    print(f"Ultima Trayectoria:\n {s_curve._trajectory_function}")

    # Para mostrar toda la trayectoria
    t = np.linspace(0,2, 50)
    for t in t:
        print(s_curve.get_state(t))


    print(f"\n ------- Prueba de ZeroMotionProfile  -------\n")

    joint_1_constraints = JointConstraints(max_velocity=5,
                                           max_acceleration=10,
                                           max_jerk=30)
    
    zero_motion = ZeroMotionProfile(constraints=joint_1_constraints)
    q0 = 4
    q1 = 4
    v0 = 2
    v1 = 2

    zero_motion.plan_trajectory(q1, duration=5)


    #t = np.linspace(0,zero_motion._duration, 100)
    #for t in t:
    #    print(zero_motion.get_state(t))

