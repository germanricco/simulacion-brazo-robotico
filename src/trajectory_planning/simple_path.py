import numpy as np
import matplotlib.pyplot as plt

class SimplePath:
    def __init__(self):
        """
        Representa caminos simples.
        """
    def calc_linear_path(self, start_pose, end_pose, num_points):
        """
        Interpolacion lineal entre start_point y end_point.

        Argumentos:
            start_pose: [x, y, z] punto de inicio
            end_pose: [x, y, z] punto final
            num_points: numero de puntos en el path
        
        Retorna:
            (np.array) Path lineal
        """
        t = np.linspace(0, 1, num_points)
        path = np.zeros((num_points, 3))

        for i in range(num_points):
            path[i, 0] = start_pose[0] * (1-t[i]) + t[i] * end_pose[0]
            path[i, 1] = start_pose[1] * (1-t[i]) + t[i] * end_pose[1]
            path[i, 2] = start_pose[2] * (1-t[i]) + t[i] * end_pose[2]

        return path
    
    def calc_circle_path(self, start_point, mid_point, end_point, num_points, rotation_angle=None, epsilon=1e-6):
        """
        Calcula una trayectoria circular que pasa por tres puntos en el espacio 3D

        Argumentos:
            start_point: [x, y, z] punto de inicio
            mid_point: [x, y, z] punto medio
            end_point: [x, y, z] punto final
            num_points: Numero de puntos a generar en la trayectoria
            rotation_angle: Angulo de rotacion del circulo (opcional)
            epsilon: Tolerancia para comparaciones numericas
        
        Retorna:
            (np.array) Array de puntos (num_points, 3) con la trayectoria circular

            ValueError: Si los puntos son colineales o hay entrada invalida
        """
        # Validación de entrada
        points = [np.asarray(p, dtype=np.float64) for p in [start_point, mid_point, end_point]]
        if any(p.shape != (3,) for p in points):
            raise ValueError("Todos los puntos deben ser coordenadas 3D")
        
        if num_points < 3:
            raise ValueError("num_points debe ser al menos 3")
        
        # Verifica que los 3 puntos no sean colineales
        start_to_mid = mid_pose - start_pose
        mid_to_end = end_pose - mid_pose
        N = np.cross(start_to_mid, mid_to_end)
        N_norm = np.linalg.norm(N)

        if N_norm < epsilon:
            raise ValueError("Los puntos son colineales y no definen un plano unico")

        # Normaliza el vector N
        N = N / N_norm
        
        # Calculo el radio del circulo usando formula geometrica
        a = start_pose-mid_pose #P0-P1
        b = mid_pose-end_pose   #P1-P2
        c = end_pose-start_pose #P2-P0
        radius = (np.linalg.norm(a)*np.linalg.norm(b)*np.linalg.norm(c))/(2*np.linalg.norm(np.cross(a,b)))

        # Calculo el centro del circulo
        c0 = (np.linalg.norm(b)**2 * np.dot(a,-c)) / (2* np.linalg.norm( np.cross(a,b) )**2)
        c1 = (np.linalg.norm(c)**2 * np.dot(b,-a)) / (2* np.linalg.norm( np.cross(b,c) )**2)
        c2 = (np.linalg.norm(a)**2 * np.dot(c,-b)) / (2* np.linalg.norm( np.cross(c,a) )**2)

        center = c0*start_pose + c1*mid_pose + c2*end_pose

        U = points[0] - center
        U = U / (np.linalg.norm(U)+epsilon)

        V = np.cross(N,U)

        V0 = start_pose - center
        V1 = mid_pose - center
        V2 = end_pose - center
        cross_product = np.cross(V0,V2)

        # Si no lo especifican calculo el angulo alpha
        if rotation_angle is None:
            rotation_angle = np.arccos(np.dot(V0,V2)/(np.linalg.norm(V0)*np.linalg.norm(V2)))

        if np.dot(N, cross_product) < 0:
            rotation_angle = 2*np.pi - rotation_angle

        # Interpolo para obtener path circular
        theta = np.linspace(0, rotation_angle, num_points)
        path = np.zeros((num_points, 3))

        for i in range(num_points):
            path[i, 0] = center[0] + radius*np.cos(theta[i])*U[0] + radius*np.sin(theta[i])*V[0]
            path[i, 1] = center[1] + radius*np.cos(theta[i])*U[1] + radius*np.sin(theta[i])*V[1]
            path[i, 2] = center[2] + radius*np.cos(theta[i])*U[2] + radius*np.sin(theta[i])*V[2]

        return path


if __name__ == "__main__":
    start_pose = np.array([0, 0, 0])
    mid_pose = np.array([2, 2, 0])
    end_pose = np.array([4, 0, 2])

    simple_path = SimplePath()

    linear_path = simple_path.calc_linear_path(start_pose, end_pose, 100)
    circular_path = simple_path.calc_circle_path(start_pose, mid_pose, end_pose, 100, rotation_angle=2*np.pi)

    # --- Graficar el Path y los puntos de control en 3D ---
    fig = plt.figure(figsize=(8, 6)) # Opcional: Ajustar el tamaño de la figura
    ax = fig.add_subplot(111, projection='3d') # Añade un subplot 3D

    # Graficar el path de la curva de Bezier
    #ax.plot(linear_path[:, 0], linear_path[:, 1], linear_path[:, 2], 'b-', label='Recta')
    ax.plot(circular_path[:, 0], circular_path[:, 1], circular_path[:, 2], 'r-', label='Circulo')

    # Añadir punto de inicio y fin (opcional, para visualizacion)
    ax.plot([start_pose[0]], [start_pose[1]], [start_pose[2]], 'go', markersize=8, label='Inicio')
    ax.plot([end_pose[0]], [end_pose[1]], [end_pose[2]], 'mo', markersize=8, label='Fin')

    # --- Personalizacion del grafico 3D ---
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Simple Path')
    ax.legend()
    ax.grid(True)

    ax.set_aspect("equal")

    plt.show() # Mostrar el grafico
