import numpy as np
from scipy.spatial.transform import Rotation as R

class OrientationPlanner:
    def __init__(self):
        self.current_euler_orientation = np.array([0, 0, 0])

    def plan_orientation(self, start_orientation, end_orientation, num_points):
        """
        Planifica una trayectoria de orientacion entre dos orientaciones eulerianas.

        Argumentos:
            start_orientation: [roll, pitch, yaw] orientacion inicial en grados
            end_orientation: [roll, pitch, yaw] orientacion final en grados
            num_points: numero de puntos en la trayectoria

        Retorna:
            (np.array) Trayectoria de orientacion en grados
        """

        # Convierto orientaciones eulerianas a cuaterniones
        start_quat = R.from_euler('xyz', start_orientation, degrees=True).as_quat()
        end_quat = R.from_euler('xyz', end_orientation, degrees=True).as_quat()

        path_orientaciones=np.zeros((num_points, 3))
        t_values = np.linspace(0, 1, num_points)

        for i, t in enumerate(t_values):
            # Interpolacion esferica entre cuaterniones
            cuaternion_interpolado = self.slerp(start_quat, end_quat, t)
            # Convierto nuevamente a euler
            euler_interpolado = R.from_quat(cuaternion_interpolado).as_euler('xyz', degrees=True)
            path_orientaciones[i] = euler_interpolado

        return path_orientaciones
        


    def slerp(self, start_quat, end_quat, t, epsilon=1e-6):
        """
        Interpolacion esferica entre dos cuaterniones

        Argumentos:
            start_quat: Cuaternión de inicio
            end_quat: Cuaternión final
            t: valor entre 0 y 1 que indica la interpolacion
            epsilon: Tolerancia para comparar cuaterniones cercanos

        Retorna:
            (np.array) Cuaternión interpolado
        """
        # Asegura que los cuaterniones sean np.array
        start_quat = np.array(start_quat)
        end_quat = np.array(end_quat)

        norm_start = np.linalg.norm(start_quat)
        norm_end = np.linalg.norm(end_quat)

        # Manejo de cuaterniones nulos o casi nulos
        if norm_start < epsilon:
            start_quat_normalized = np.array([0.0, 0.0, 0.0, 1.0]) # Cuaternión identidad si start es nulo
            print("Advertencia: Cuaternión inicial casi nulo, usando cuaternión identidad para el inicio.")
        else:
            start_quat_normalized = start_quat / norm_start

        if norm_end < epsilon:
            end_quat_normalized = np.array([0.0, 0.0, 0.0, 1.0]) # Cuaternión identidad si end es nulo
            print("Advertencia: Cuaternión final casi nulo, usando cuaternión identidad para el fin.")
        else:
            end_quat_normalized = end_quat / norm_end

        # Calcula el angulo entre los cuaterniones
        dot_product = np.dot(start_quat_normalized, end_quat_normalized)

        # Ajuste para evitar errores de punto flotante fuera del rango [-1, 1] para arccos
        if dot_product > 1.0:
            dot_product = 1.0
        elif dot_product < -1.0:
            dot_product = -1.0

        # Como el modulo de q1 y q2 son cuaterniones normalizados, el angulo entre ellos es arccos(q1*q2)
        alpha = np.arccos(dot_product)

        if np.isnan(alpha):
            alpha = 0
        
        if alpha < epsilon:
            # Interpolacion lineal standard si los cuaterniones son muy cercanos
            print("Utilizando NLERP")
            q = (1 - t) * start_quat + t * end_quat
            return q / np.linalg.norm(q)

        else:
            # SLERP
            if np.sin(alpha) == 0: # Manejo del caso donde alpha es muy cercano a 0 o multiplo de pi (para evitar division por cero)
                q = (1 - t) * start_quat_normalized + t * end_quat_normalized
            else:
                q = (np.sin((1 - t) * alpha) / np.sin(alpha)) * start_quat + (np.sin(t * alpha) / np.sin(alpha)) * end_quat
            return q / np.linalg.norm(q)

    def producto_cuaternion(self, q1, q2):
        """
        Calcula el producto de dos cuaterniones (q1 * q2).

        Parámetros:
        q2, q1 (np.array): Cuaterniones, donde q = [x, y, z, w] = [vector, escalar].
                        Las tres primeras componentes son la parte vectorial (x, y, z) y la última es la parte escalar (w).

        Retorna:
        np.array: Cuaternión resultante del producto q1 * q2.
        """
        v1 = q1[0:3] # Parte vectorial de q1 (x, y, z)
        w1 = q1[3]   # Parte escalar de q1 (w)
        v2 = q2[0:3]
        w2 = q2[3]

        vector_part = w1 * v2 + w2 * v1 + np.cross(v1, v2)
        scalar_part = w1 * w2 - np.dot(v1, v2)

        q_producto = np.array([vector_part[0], vector_part[1], vector_part[2], scalar_part])
        return q_producto

    
if __name__ == "__main__":
    planner = OrientationPlanner()

    path_orientaciones = planner.plan_orientation([0, 0, 0], [90, 0, 0], 10)
    print(f"Trayectoria de orientaciones: {path_orientaciones}")