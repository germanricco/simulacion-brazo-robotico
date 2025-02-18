import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import sys
from pathlib import Path

# Importo modulos de planificacion de trayectoria
project_root = Path(__file__).resolve().parent.parent.parent

src_path = project_root / "src"
sys.path.append(str(src_path))

from collision_detection.geometric_objects import Cuboid
from algebra_lineal.euler import Euler

class Plotter:
    def __init__(self, title="Plot", dimension=2):
        """
        Inicializa el objeto Plotter para graficar elementos dinamicamente

        Parametros:
            * title (str, opcional): Titulo del plot.
            * dimension (int, opcional): Dimension del plot
        """

        # Inicializar la figura y el subplot
        self.fig = plt.figure()
        if dimension == 3:
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.dimension = 3
        else:
            self.ax = self.fig.add_subplot(111)
            self.dimension = 2

        self.ax.set_title(title)

        # Creo una lista para los elementos añadidos
        self.added_elements = []
    
    def add_trajectory(self,
                       trajectory,
                       label="Trajectory",
                       color='blue',
                       linestyle='-',
                       marker=None):
        """
        Añade una trayectoria al plot.

        Parámetros:
            * trajectory (np.array): Array de NumPy con los puntos de la trayectoria.
                                    Forma: [[x0, y0], [x1, y1], ...] o [[x0, y0, z0], [x1, y1, z1], ...].
            * label (str, opcional): Etiqueta para la leyenda. Por defecto "Trajectory".
            * color (str, opcional): Color de la línea. Por defecto 'blue'.
            * linestyle (str, opcional): Estilo de línea. Por defecto '-'.
            * marker (str, opcional): Marcador para los puntos. Por defecto None.
        """
        if self.dimension == 3:
            self.ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                         label=label,
                         color=color,
                         linestyle=linestyle,
                         marker=marker)
        else:
            self.ax.plot(trajectory[:, 0], trajectory[:, 1],
                         label=label,
                         color=color,
                         linestyle=linestyle,
                         marker=marker)
        self.added_elements.append({'type': 'trajectory', 'label': label})

    def add_cuboid(self, cuboid, label="Cuboid", color="red", alpha=0.2):
        """
        Añade un cuboide 3D al plot.

        Parámetros:
            * cuboid: Objeto Cuboid previamente definido (deberías tener una clase Cuboid).
            * label (str, opcional): Etiqueta para la leyenda. Por defecto "Cuboid".
            * color (str, opcional): Color del cuboide. Por defecto 'red'.
            * alpha (float, opcional): Transparencia del cuboide (0 a 1). Por defecto 0.2.
        """
        if self.dimension != 3:
            raise ValueError("Cuboids solo se pueden añadir a plots 3D.")
        
        #! Se asume que el objeto 'cuboid' tiene metodo para obtener vertices
        vertices = cuboid.get_vertices_for_plotting()
        if vertices is None or len(vertices) != 8:
            raise ValueError("El objeto cuboid debe proporcionar 8 vertices para graficar en 3D.")
        
        # Definir las caras del cuboide usando los vertices
        caras = [[vertices[0], vertices[1], vertices[2], vertices[3]], # Cara frontal
                 [vertices[4], vertices[5], vertices[6], vertices[7]], # Cara trasera
                 [vertices[0], vertices[1], vertices[5], vertices[4]], # Cara inferior
                 [vertices[3], vertices[2], vertices[6], vertices[7]], # Cara superior
                 [vertices[1], vertices[2], vertices[6], vertices[5]], # Cara lateral derecha
                 [vertices[0], vertices[3], vertices[7], vertices[4]]] # Cara lateral izquierda

        # Crear la coleccion de poligonos 3D para el cuboide
        cuboide_poly = Poly3DCollection(caras,
                                        facecolors=color,
                                        linewidths=1,
                                        edgecolors='black',
                                        label=label,
                                        alpha=alpha)

        self.ax.add_collection3d(cuboide_poly)
 
        self.added_elements.append({'type': 'cuboid', 'label': label}) # Rastreo opcional

    def add_robot(self, robot, label="Robotic Arm", color="blue"):
        joint_positions = robot.get_joints_for_plotting()
        self.ax.scatter(joint_positions[0,0], joint_positions[0,1], joint_positions[0,2], color='green', s=100, label="GCS")     # GCS en verde
        self.ax.plot(joint_positions[1:,0], joint_positions[1:,1], joint_positions[1:,2], 'bo-', linewidth=2, label="Brazo robótico")  # Dibujar el brazo
        self.ax.scatter(joint_positions[-1,0], joint_positions[-1,1], joint_positions[-1,2], color='b', s=100, label="TCP")

    def add_pose(self, pose, label="Pose"):
        """
        Añade la visualizacion de una Pose al plot

        Argumentos:
            * pose (Pose): Objeto Pose que define posicion y orientacion
            * name (str, opcional): Nombre para etiquetar la Pose
        """
        posicion = pose.position
        angulos_euler_deg = pose.orientation
        angulos_euler_rad = np.deg2rad(angulos_euler_deg)

        # Dibujar punto rojo
        self.ax.scatter(posicion[0], posicion[1], posicion[2],color="red", s=100, label=label)

        # Dibujar sistema de coordenadas
        LONG_EJES = 500

        R_pose = Euler.euler_a_matriz(angulos_euler_rad[0], angulos_euler_rad[1], angulos_euler_rad[2])

        # Ejes locales
        eje_x = R_pose[:, 0] * LONG_EJES
        eje_y = R_pose[:, 1] * LONG_EJES
        eje_z = R_pose[:, 2] * LONG_EJES

        # Dibujar las flechas para los ejes X, Y, Z del TCP
        self.ax.quiver(posicion[0], posicion[1], posicion[2], eje_x[0], eje_x[1], eje_x[2], color='r', length=1.0, arrow_length_ratio=0.1, label=f'X_{label}') # Eje X en rojo
        self.ax.quiver(posicion[0], posicion[1], posicion[2], eje_y[0], eje_y[1], eje_y[2], color='g', length=1.0, arrow_length_ratio=0.1, label=f'Y_{label}') # Eje Y en verde
        self.ax.quiver(posicion[0], posicion[1], posicion[2], eje_z[0], eje_z[1], eje_z[2], color='b', length=1.0, arrow_length_ratio=0.1, label=f'Z_{label}') # Eje Z en azul

        #self.ax.legend()

    
    def customize_plot(self, title=None, xlabel="X", ylabel="Y", zlabel="Z",
                       equal_aspect=False, grid=True, legend=True):
        """
        Personaliza el plot: título, etiquetas de ejes, aspecto, grilla y leyenda.

        Parámetros:
            * title (str, opcional): Nuevo título del plot. Si None, usa el título inicial.
            * xlabel (str, opcional): Etiqueta para el eje X. Por defecto "X".
            * ylabel (str, opcional): Etiqueta para el eje Y. Por defecto "Y".
            * zlabel (str, opcional): Etiqueta para el eje Z (solo para plots 3D). Por defecto "Z".
            * equal_aspect (bool, opcional): Forzar la misma escala en los ejes (X, Y, Z en 3D). Por defecto False.
            * grid (bool, opcional): Mostrar la grilla. Por defecto True.
            * legend (bool, opcional): Mostrar la leyenda. Por defecto True.
        """
        if title:
            self.ax.set_title(title)

        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        if self.dimension == 3:
            self.ax.set_zlabel(zlabel)

        if equal_aspect and self.dimension == 3:
            self.ax.set_aspect('equal') # Importante para plots 3D con escala igual

        self.ax.grid(grid)

        if legend:
            self.ax.legend()

    def show_plot(self):
        """
        Muestra el plot en una ventana
        """
        plt.show()

    def clear_plot(self):
        """
        Limpia el plot, removiendo todos los elementos graficados
        """
        self.ax.cla()
        # Borra elementos añadidios
        self.added_elements = []
        if self.dimension == 3:
            self.ax.set_label("Z")
        self.customize_plot()


if __name__ == '__main__':
    # --- Datos de ejemplo ---
    trayectoria_2d = np.array([[1, 2], [2, 3], [3, 2], [4, 3]])
    trayectoria_3d = np.array([[1, 2, 1], [2, 3, 2], [3, 2, 3], [4, 3, 4]])

    cuboide1 = Cuboid([0, 0, 0], [2, 2, 2])
    cuboide2 = Cuboid([3, 3, 3], [5, 5, 5])

    # --- Ejemplo de plot 2D ---
    plotter_2d = Plotter(title="Trayectoria 2D", dimension=2)
    plotter_2d.add_trajectory(trayectoria_2d, label="Trayectoria 2D", color='green', marker='o')
    plotter_2d.customize_plot(xlabel="Eje X (2D)", ylabel="Eje Y (2D)", equal_aspect=True)
    plotter_2d.show_plot()

    # --- Ejemplo de plot 3D con trayectorias y cuboides ---
    plotter_3d = Plotter(title="Trayectoria 3D y Cuboides", dimension=3)
    plotter_3d.add_trajectory(trayectoria_3d, label="Trayectoria 3D", color='purple', linestyle='--', marker='^')
    plotter_3d.add_cuboid(cuboide1, label="Cuboide 1", color='cyan', alpha=0.3)
    plotter_3d.add_cuboid(cuboide2, label="Cuboide 2", color='orange', alpha=0.3)
    plotter_3d.customize_plot(xlabel="Eje X", ylabel="Eje Y", zlabel="Eje Z", equal_aspect=True, legend=True)
    plotter_3d.show_plot()

    # --- Ejemplo de limpiar y reutilizar el mismo plotter 3D ---
    plotter_3d.clear_plot() # Limpiar el plot 3D
    plotter_3d.add_trajectory(trayectoria_3d * 0.5, label="Trayectoria 3D Escala 0.5", color='red', linestyle='-.', marker='x') # Añadir nueva trayectoria
    plotter_3d.customize_plot(title="Nueva Trayectoria en Plot 3D Limpio", legend=True) # Modificar titulo y leyenda
    plotter_3d.show_plot() # Mostrar el nuevo plot