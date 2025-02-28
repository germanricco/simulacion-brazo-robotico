"""
Creditos:
https://github.com/nameofuser1/py-scurve/tree/master
"""

from abc import ABC, abstractmethod
import numpy as np
import sys
from pathlib import Path

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from trajectory_planning.trajectory_planner import TrajectoryPlanner
from trajectory_planning.trajectory import Trajectory


# CONSTANTES
ACCELERATION_ID = 0
SPEED_ID = 1
POSITION_ID = 2

OPTIMIZER_THRESHOLD = 0.01
EPSILON = 0.0001

class VelocityProfile(ABC):
    """
    Interface para perfiles de velocidad
    """

    @abstractmethod
    def plan_trajectory(self, q0, q1, v0, v1, constraints):
        """
        Planifica una trayectoria entre dos puntos con restricciones dadas
        """
        pass


class SCurveProfile(VelocityProfile):
    """
    Implementacion de perfil S-Curve o curva de 7 segmentos
    """
    def __init__(self):
        
        """
        Inicializa un perfil S-Curve con sus restricciones cinemáticas
        """


    def _check_shape(self, *args):
        sh = len(args[0])

        for arg in args:
            if sh != len(arg):
                raise ValueError("All parameters must have the same dimention")

        return (sh,)
    

    def _scurve_check_possibility(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Verifica si una trayectoria es posible. Si no ValueError

        pag 80-81. of 'Trajectory planning for automatic machines and robots(2008)
        """
        dv = np.abs(v1 - v0)
        dq = np.abs(q1 - q0)

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


    def _compute_maximum_speed_reached(self, q0, q1, v0, v1,
                                        v_max, a_max, j_max):
        """
        Case 1. V_lim = V_max
        """

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


    def _compute_maximum_speed_not_reached(self, q0, q1, v0, v1,
                                            v_max, a_max, j_max):
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
    

    def _scurve_search_planning(self, q0, q1, v0, v1, v_max, a_max,
                                j_max, l=0.99, max_iter=500,
                                dt_thresh=0.01, T=None):
        """
        Método para encontrar parámetros de trayectoria S con aceleración ajustada.
        Es decir cuando a_lim < a_max siendo a_lim desconocida
        """
        
        current_a_max = a_max
        iteration = 0

        while iteration < max_iter and current_a_max > 1e-9:
            try:
                # 1. Calcular parámetros base
                Tj1, Ta, Tj2, Td, Tv = self._compute_maximum_speed_not_reached(
                    q0, q1, v0, v1, v_max, current_a_max, j_max)

                # 2. Manejar casos especiales inmediatamente
                if Ta < 0 or Td < 0:
                    return self._handle_special_cases(q0, q1, v0, v1, j_max, Tj1, Ta, Tj2, Td, Tv)

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
            return self._handle_special_cases(q0, q1, v0, v1, j_max, 0, 0, 0, 0, 0)
        except ValueError:
            raise ValueError(f"No se encontró solución en {iteration} iteraciones. Último a_max: {current_a_max:.4e}")
            
        
    def _handle_special_cases(self, q0, q1, v0, v1, j_max, Tj1, Ta, Tj2, Td, Tv):
        """
        Maneja casos donde Ta o Td son negativos, es decir cuando solo se necesita 1 fase
        de aceleracion/desaceleracion.
        """
        delta_q = q1 - q0
        sum_v = v0 + v1
        
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
    

    def _sign_transforms(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Transforma los signos para ser capaz de calcular la trayectoria con q1 < q0

        Look at 'Trajectory planning for automatic machines and robots(2008)'
        (pag. 90)
        """

        # Asumimos simetria
        v_min = -v_max
        a_min = -a_max
        j_min = -j_max

        s = np.sign(q1-q0)
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

        return _q0, _q1, _v0, _v1, _v_max, _a_max, _j_max


    def compute_parameters(self, q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Computes s-curve trajectory parameters which are:
            Tj1     --- non-zero constant jerk period while accelerating
            Ta      --- total acceleration period time
            Tj2     --- non-zero constant jerk period while decelerating
            Td      --- total deceleration time
            Tv      --- constant speed time
        """
        # Si la trayectoria es posible
        if self._scurve_check_possibility(q0, q1, v0, v1, v_max, a_max, j_max):
            # Calcula cambio de signo de condiciones iniciales
            q0, q1, v0, v1, v_max, a_max, j_max = self._sign_transforms(q0, q1, v0, v1, v_max, a_max, j_max)
            
            # Asumiendo que se alcanza v_max y a_max
            try:
                Tj1, Ta, Tj2, Td, Tv =\
                    self._compute_maximum_speed_reached(q0, q1, v0, v1,
                                                         v_max, a_max, j_max)
            except ValueError as e:
                # Error si Tv<0
                print(f"Error. {e}")
                try:
                    Tj1, Ta, Tj2, Td, Tv =\
                        self._scurve_search_planning(q0, q1, v0, v1,
                                                                 v_max, a_max,
                                                                 j_max)
                except ValueError as e:
                    # Error si no se encontro solucion iterando
                    print(f"Error. {e}")

            return np.asarray([Tj1, Ta, Tj2, Td, Tv], dtype=np.float32)

        else:
            raise TypeError("Trajectory is not feasible")





    def _point_sign_transform(self, q0, q1, p):
        """
        Transforms point back to the original sign
        """
        s = np.sign(q1-q0)
        return s*p


    def _get_trajectory_func(self, Tj1, Ta, Tj2, Td, Tv,
                              q0, q1, v0, v1, v_max, a_max, j_max):
        """
        Returns function of time given trajectory parameters
        """
        T = Ta + Td + Tv
        a_lim_a = j_max*Tj1
        a_lim_d = -j_max*Tj2
        v_lim = v0 + (Ta-Tj1)*a_lim_a

        def trajectory(t):
            """
            Returns numpy array with shape (3,) which contains acceleration,
                speed and position for a given time t
            """
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

            point = np.zeros((3,), dtype=np.float32)
            point[ACCELERATION_ID] = a
            point[SPEED_ID] = v
            point[POSITION_ID] = q

            return point

        return trajectory


    def _get_trajectory_function(self, q0, q1, v0, v1, v_max, a_max, j_max,
                                  Tj1, Ta, Tj2, Td, Tv):
        """
        Returns function wich wrapps trajectory function with sign transforms
        """
        zipped_args = self._sign_transforms(q0, q1, v0, v1, v_max, a_max,
                                             j_max)

        traj_func = self._get_trajectory_func(Tj1, Ta, Tj2,
                                               Td, Tv, *zipped_args)

        def sign_back_transformed(t):
            return self.__point_sign_transform(q0, q1, traj_func(t))

        return sign_back_transformed


    def _put_params(self, params_list, params, dof):
        for i in range(len(params_list)):
            params_list[i][dof] = params[i]


    def _get_dof_time(self, params_list, dof):
        return params_list[1][dof] + params_list[3][dof] + params_list[4][dof]


    def _get_traj_params_containers(self, sh):
        T = np.zeros(sh)
        Ta = np.zeros(sh)
        Tj1 = np.zeros(sh)
        Td = np.zeros(sh)
        Tj2 = np.zeros(sh)
        Tv = np.zeros(sh)

        return T, Tj1, Ta, Tj2, Td, Tv


    def _plan_trajectory_1D(self, q0, q1, v0, v1, v_max, a_max, j_max, T=None):
        """
        Calcula la trayectoria con el tiempo minimo o ajustandose al tiempo T

        Retorna:
            Lista de parametros de trajectoria
        """

        zipped_args = self._sign_transforms(q0, q1, v0, v1, v_max, a_max,
                                             j_max)
        
        print(f"Zipped_args:\n {zipped_args}")

        if T is None:
            # Calculo la trayectoria para el tiempo minimo
            res = self.compute_parameters(*zipped_args)

        return res


    def plan_trajectory(self, q0, q1, v0, v1, v_max, a_max, j_max, t=None):
        """
        Planifica una trayectoria S-Curve entre dos puntos
        
        Argumentos:
            * q0 (np.array): Lista con posiciones iniciales
            * q1 (np.array): Lista con posiciones finales
            * v0 (np.array): Lista con velocidades iniciales
            * v1 (np.array): Lista con velocidades finales
            * v_max: velocidad maxima
            * a_max: aceleracion maxima
            * j_max: jerk maximo
            * t: Tiempo objetivo total (opcional)
            
        Retorna:
           Función de tiempo que devuelve aceleración, velocidad y posición
        """
        parameters_shape = self._check_shape(q0, q1, v0, v1)
        ndof = parameters_shape[0]

        print(f"Num DOF: {ndof}")

        # Creo np.array que contiene todos los parametros necesarios para la planificacion.
        task_list = np.asarray([q0, q1, v0, v1, [v_max]*ndof,
                               [a_max]*ndof, [j_max]*ndof],
                               dtype=np.float32)
        
        print(f"Task List:\n {task_list}")
        
        # Obtengo arrays para guardar resultados de parametros
        T, Tj1, Ta, Tj2, Td, Tv = self._get_traj_params_containers(parameters_shape)
        # Creo matriz de parametros
        trajectory_params = np.asarray([Tj1, Ta, Tj2, Td, Tv], dtype=np.float32)

        print(f"Trajectory Parameters:\n {trajectory_params}")
        
        trajectory_funcs = []

        # Obtengo ID (columna) de joint_path con mayor desplazamiento
        dq = np.subtract(q1, q0)
        max_displacement_id = np.argmax(np.abs(dq))
        
        print(f"Max Displacement ID: {max_displacement_id}")

        max_displacement_params =\
            self._plan_trajectory_1D(*task_list[:, max_displacement_id], T=t)
        
        print(f"Max Displacement parameters:\n {max_displacement_params}")

        self._put_params(trajectory_params,
                          max_displacement_params,
                          max_displacement_id)
        
        print(f"Updated Trajectory Parameters:\n {trajectory_params}")

        max_displacement_time = self._get_dof_time(trajectory_params,
                                                    max_displacement_id)
        T[max_displacement_id] = max_displacement_time

        for _q0, _q1, _v0, _v1, ii in zip(q0, q1, v0, v1, range(ndof)):
            if ii == max_displacement_id:
                continue

            #planning_logger.info("Computing %d DOF trajectory" % ii)

            # In case if final velocity is non-zero
            # We need to synchronize it in time with the trajectory with
            # the longest execution time
            if _v1 != 0:
                traj_params =\
                    self._plan_trajectory_1D(_q0, _q1, _v0, _v1, v_max,
                                              a_max, j_max,
                                              T=max_displacement_time)

            # if final velocity is zero we do not need to worry about
            # syncronization
            else:
                traj_params = self._plan_trajectory_1D(_q0, _q1, _v0, _v1,
                                                        v_max, a_max, j_max)

            T[ii] = Ta[ii] + Td[ii] + Tv[ii]
            self._put_params(trajectory_params, traj_params, ii)

        # Getting trajectory functions
        for dof in range(ndof):
            tr_func = self._get_trajectory_function(q0[dof], q1[dof],
                                                     v0[dof], v1[dof], v_max,
                                                     a_max, j_max,
                                                     *trajectory_params[:, dof])
            trajectory_funcs.append(tr_func)

        tr = Trajectory()
        tr.time = (T[max_displacement_id],)
        tr.trajectory = trajectory_funcs
        tr.dof = ndof

        return tr


if __name__ == "__main__":
    s_curve = SCurveProfile()

    print(f"\n -- Example 3.9 --")
    print("Tested 'compute_max_speed_reached' \n")
    q0 = 0
    q1 = 10
    v0 = 1
    v1 = 0
    v_max = 5
    a_max = 10
    j_max = 30

    parameters = s_curve.compute_parameters(q0,q1,v0,v1,v_max,a_max,j_max)

    print(parameters)
    print(f"\n -- Example 3.10 --")
    print("Tested 'compute_max_speed_not_reached' \n")

    v_max = 10

    Tj1, Ta, Tj2, Td, Tv = s_curve.compute_parameters(q0,q1,v0,v1,v_max,a_max,j_max)

    print(f"Ta={round(Ta,4)}, Tv={round(Tv,4)}, Td={round(Td,4)}, Tj1={round(Tj1,4)}, Tj2={round(Tj2,4)} ")

    print(f"\n -- Example 3.11 --")
    print("Tested 'max_acceleration_not_reached' \n")

    v0 = 7

    Tj1, Ta, Tj2, Td, Tv = s_curve.compute_parameters(q0,q1,v0,v1,v_max,a_max,j_max)

    print(f"Ta={round(Ta,4)}, Tv={round(Tv,4)}, Td={round(Td,4)}, Tj1={round(Tj1,4)}, Tj2={round(Tj2,4)} ")


    print(f"\n -- Example 3.12 --")
    print("Tested 'max_acceleration_not_reached and Special Case 1 (Ta<0)' \n")

    q0 = 0
    q1 = 10
    v0 = 7.5
    v1 = 0
    v_max = 10
    a_max = 10
    j_max = 30

    Tj1, Ta, Tj2, Td, Tv = s_curve.compute_parameters(q0,q1,v0,v1,v_max,a_max,j_max)

    print(f"Ta={round(Ta,4)}, Tv={round(Tv,4)}, Td={round(Td,4)}, Tj1={round(Tj1,4)}, Tj2={round(Tj2,4)} ")
