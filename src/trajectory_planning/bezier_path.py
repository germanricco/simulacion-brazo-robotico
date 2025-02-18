import numpy as np
import matplotlib.pyplot as plt

class BezierPath:
    def __init__(self):
        """
        Representa un camino de Bezier en 3D.
        """
        self.control_points = None

    def calc_2points_bezier_path(self,
                                 start_point,
                                 end_point,
                                 num_points,
                                 offset=3,
                                 start_direction=None,
                                 end_direction=None):
        """
        Calcula los puntos de control y el camino dado el punto de inicio y fin.

        Parametros:
            * start_point: (numpy array) x, y, z del punto inicial
            * end_point: (numpy array) x, y, z del punto final
            * offset: (float) Factor que controla la distancia de los puntos de control
            * start_direction: (numpy array) Vector de dirección en el punto de inicio.
                Si es None, se asume inicio desde reposo
            * end_direction: (numpy array) Vector de dirección en el punto final.
                Si es None, se asume fin en reposo
        Retorna: 
            * (numpy array, numpy array)
        """
        # Calculate the distance of the control points
        control_point_dist = np.linalg.norm(end_point - start_point) / offset
        control_point_dist = np.round(control_point_dist, 4)
        #!print(f"Offset: {offset} || Control Point Dist: {control_point_dist}")
        
        # Verifica que los puntos de inicio y fin no sean iguales
        direccion_vector = end_point - start_point
        if np.linalg.norm(direccion_vector) == 0:
            print("Error: Los puntos de inicio y fin son iguales")
            return None
        else:
            direccion_vector = direccion_vector / np.linalg.norm(direccion_vector)
            
        # Si trabajamos con un vector de dirección en el punto de inicio
        if start_direction is not None and np.linalg.norm(start_direction) != 0:
            start_direction = start_direction / np.linalg.norm(start_direction)
            control_point_1 = start_point + control_point_dist * start_direction
        else:
            control_point_1 = start_point + control_point_dist * direccion_vector

        # Si trabajamos con un vector de direccion en el punto final
        if end_direction is not None and np.linalg.norm(end_direction) != 0:
            end_direction = end_direction / np.linalg.norm(end_direction)
            control_point_2 = end_point - control_point_dist * end_direction
        else:
            control_point_2 = end_point - control_point_dist * direccion_vector

        control_points = np.array(
            [start_point,
            control_point_1,
            control_point_2,
            end_point])
        
        path = self.calc_bezier_path(control_points, n_points=num_points)

        return path#, control_point

    def calc_bezier_path(self, control_points, n_points=100):
        """
        Calcula el camino de bezier (trayectoria) dado los puntos de control.

        Parametros:
            * control_points: (numpy array)
            * n_points: (int) número de puntos en la trayectoria
        Retorna:
            * (numpy array)
        """
        traj = []
        for t in np.linspace(0, 1, n_points):
            traj.append(self.bezier(t, control_points))

        return np.array(traj)

    def bezier(self, t, control_points):
        """
        Retorna un punto en la curva de bezier.

        Parametros:
            * t: (float) número en [0, 1]
            * control_points: (numpy array)
        Retorna:
            * (numpy array) Coordenadas del punto
        """

        # De Casteljau's Algorithm

        n = len(control_points)

        # Caso base: si solo hay un punto de control, es el punto en la curva
        if n == 1:
            return control_points[0]

        # Paso recursivo: interpolar linealmente los puntos de control adyacentes
        intermediate_points = np.zeros((n - 1, control_points.shape[1])) # Inicializar array para puntos intermedios

        for i in range(n - 1):
            intermediate_points[i] = (1 - t) * control_points[i] + t * control_points[i+1]

        # Llamada recursiva con los nuevos puntos de control intermedios
        return self.bezier(t, intermediate_points)
    
    def calc_3points_bezier_path(self,
                                 start_point,
                                 mid_point,
                                 end_point,
                                 num_points,
                                 offset=3,
                                 start_direction=None,
                                 end_direction=None):
        """
        Calcula los puntos de control y el camino dado el punto de inicio y fin.

        Parametros:
            * start_point: (numpy array) x, y, z del punto inicial
            * mid_point: (numpy array) x, y, z del punto medio
            * end_point: (numpy array) x, y, z del punto final
            * offset: (float)
            * start_direction: (numpy array) Vector de dirección en el punto de inicio.
                Si es None, se asume inicio desde reposo
            * end_direction: (numpy array) Vector de dirección en el punto final.
                Si es None, se asume fin en reposo
        Retorna: 
            * (numpy array, numpy array)
        """
        # Calculo dist puntos de control y direccion entre puntos extremos
        control_point_dist = np.linalg.norm(mid_point - start_point) / offset
        control_point_dist = np.round(control_point_dist, 4)
        
        direccion_vector = end_point - start_point
        if np.linalg.norm(direccion_vector) == 0:
            print("Error: Los puntos de inicio y fin son iguales")
            return None
        else:
            direccion_vector = direccion_vector / np.linalg.norm(direccion_vector)

        # Si trabajamos con un vector de dirección en el punto de inicio
        if start_direction is not None and np.linalg.norm(start_direction) != 0:
            start_direction = start_direction / np.linalg.norm(start_direction)
            control_point_1 = start_point + control_point_dist * start_direction
        else:
            control_point_1 = start_point + control_point_dist * direccion_vector

        control_point_2 = mid_point - control_point_dist * direccion_vector

        partial_control_points = np.array(
            [start_point,
            control_point_1,
            control_point_2,
            mid_point])
        
        partial_num_point = num_points/2
        
        path1 = self.calc_bezier_path(partial_control_points, n_points = partial_num_point)
        
        control_point_4 = mid_point + control_point_dist * direccion_vector
        # Si trabajamos con un vector de direccion en el punto final
        if end_direction is not None and np.linalg.norm(end_direction) != 0:
            end_direction = end_direction / np.linalg.norm(end_direction)
            control_point_5 = end_point - control_point_dist * end_direction
        else:
            control_point_5 = end_point - control_point_dist * direccion_vector

        partial_control_points = np.array(
            [mid_point,
            control_point_4,
            control_point_5,
            end_point])
        
        path2 = self.calc_bezier_path(partial_control_points, n_points=partial_num_point)

        control_points = np.array(
            [start_point,
             control_point_1,
             control_point_2,
             mid_point,
             control_point_4,
             control_point_5,
             end_point])

        path = np.concatenate((path1, path2), axis=0)
        return path#,control_points
    
    def bezier_derivatives_control_points(control_points, n_derivatives):
        """
        Calcular los puntos de control de las derivadas sucesivas de una curva de Bézier dada.
        Una derivada de una curva de Bézier es una curva de Bézier.

        Parametros:
            * control_points: (numpy array)
            * n_derivatives: (int)

        Ejemplo:
            n_derivatives=2 -> calcula los puntos de control para 1er y 2da derivada

        Retorna:
            * ([numpy array])
        """
        w = {0: control_points}
        for i in range(n_derivatives):
            n = len(w[i])
            w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                                for j in range(n - 1)])
        return w

    def curvature(dx, dy, ddx, ddy):
        """
        Calcula la curvatura en un punto dadas la 1er y 2da derivada.

        Parametros:
            * dx: (float) First derivative along x axis
            * dy: (float)
            * ddx: (float) Second derivative along x axis
            * ddy: (float)

        Retorna: (float)
        """
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)
    


if __name__ == "__main__":
    start_point = np.array([0, 0, 0])
    mid_point = np.array([2.5, 5, 0])
    end_point = np.array([5, 0, 0])
    
    start_direction = np.array([1, 2, -1])
    end_direction = np.array([1, -2, 1])

    num_points=100

    BazierPath = BezierPath()
    path = BazierPath.calc_2points_bezier_path(start_point, end_point, num_points, start_direction=start_direction, end_direction=end_direction)

    # --- Graficar el Path y los puntos de control en 3D ---
    fig = plt.figure(figsize=(8, 6)) # Opcional: Ajustar el tamaño de la figura
    ax = fig.add_subplot(111, projection='3d') # Añade un subplot 3D

    # Graficar el path de la curva de Bezier
    ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', label='Path de Bezier') # 'b-' para linea azul continua

    # Añadir punto de inicio y fin (opcional, para visualizacion)
    ax.plot([start_point[0]], [start_point[1]], [start_point[2]], 'go', markersize=8, label='Inicio') # 'go' punto verde grande
    ax.plot([end_point[0]], [end_point[1]], [end_point[2]], 'mo', markersize=8, label='Fin') # 'mo' punto magenta grande

    # --- Personalizacion del grafico 3D ---
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z') # Añade label para el eje Z
    ax.set_title('Curva de Bezier y Puntos de Control en 3D')
    ax.legend() # Mostrar la leyenda
    ax.grid(True) # Mostrar la grilla

    plt.show() # Mostrar el grafico