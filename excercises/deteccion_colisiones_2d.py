import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import scipy.special

import sys
from pathlib import Path

# Importo modulos de planificacion de trayectoria
project_root = Path(__file__).resolve().parent.parent
print(f"Project Root: {project_root}")

planificacion_path = project_root / "src" / "planificacion_trayectoria"
sys.path.append(str(planificacion_path))

try:
    import bezier_path_2d
    print("Modulo bezier_path_2d importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudieron importar los modulos ")

class Rectangle():
    def __init__(self, min_point, max_point):
        self.min_point = min_point
        self.max_point = max_point

    def calc_parametros(self):
        lado_x = max_point[0] - min_point[0]
        lado_y = max_point[1] - min_point[1]

        if lado_x >= lado_y:
            largo = lado_x
            ancho = lado_y
        else:
            largo = lado_y
            ancho = lado_x
            

        return largo, ancho

def segmentar_path(path, num_segments):
    """
    Segmenta un path, reduciendo el número de puntos para obtener aproximadamente
    el número deseado de segmentos.

    Parámetros:
        * path: (numpy array) Trayectoria de puntos [ [x0, y0, z0], [x1, y1, z1], ... ]
        * num_segments: (int) Número deseado de segmentos en el path segmentado.

    Retorna:
        * numpy array: Nuevo path segmentado (subconjunto de puntos del path original).
                     Si num_segments es inválido o mayor que el máximo posible,
                     retorna el path original sin segmentar.
    """
    n_points = len(path)
    if n_points < 2:
        return path  # No se puede segmentar si hay menos de 2 puntos

    max_segments = n_points - 1
    if num_segments <= 0:
        raise ValueError("num_segments debe ser mayor que 0.")
    if num_segments >= max_segments:
        return path  # Retornar path original si se piden demasiados segmentos

    indices = np.linspace(0, n_points - 1, num_segments + 1, dtype=int)
    segmented_path = path[indices]
    return segmented_path

def calc_longitud_tramo(path):
    """
    Calcula la longitud de la trayectoria.
    

    Parámetros:
        * path: (numpy array) Trayectoria de puntos [ [x0, y0], [x1, y1], ... ]

    Retorna:
        * float: Longitud total de la trayectoria.
    """
    num_puntos = len(path)
    if num_puntos < 2:
        return 0

    longitud = np.linalg.norm(np.diff(path, axis=0), axis=1)
    longitud_total = np.sum(longitud)
    return longitud_total



if __name__ == "__main__":
    # 2d Cuboid (Rectangulo)
    # Pueden ser de 2 tipos (SafeZone u ForbiddenZone)
    min_point=np.array([0, 0])
    max_point=np.array([4, 2])

    rectangulo = Rectangle(min_point, max_point)
    length, width = rectangulo.calc_parametros()
    print(f"Largo: {length} || Ancho: {width}")
    

    # Bezier curve.
    start_x = 0.0
    start_y = 0.0
    start_yaw = np.radians(0.0)

    end_x = 5.0
    end_y = 5.0
    end_yaw = np.radians(90.0)
    offset = 3.0

    path, control_points = bezier_path_2d.calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    longitud = calc_longitud_tramo(path)
    print(f"Longitud de la trayectoria sin segmentar: {longitud}")

    path_segmentado = segmentar_path(path, 4)
    longitud_2 = calc_longitud_tramo(path_segmentado)
    print(f"Longitud de la trayectoria segmentada: {longitud_2}")

    # -- PLOTEO --
    fig, ax = plt.subplots()
    ax.plot(path.T[0], path.T[1], label="Bezier Path")
    ax.plot(control_points.T[0], control_points.T[1], '--o', label="Control Points")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)
    plt.show()