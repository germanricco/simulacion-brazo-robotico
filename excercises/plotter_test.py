import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection # Importar para cuboides 3D

class Plotter:
    def __init__(self, title="Plot", dimension=2):
        """
        Inicializa un objeto Plotter para graficar elementos dinámicamente.

        Parámetros:
            title (str, opcional): Título del plot. Por defecto "Plot".
            dimension (int, opcional): Dimensión del plot (2 o 3). Por defecto 2.
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
        self.added_elements = [] # Lista para rastrear elementos añadidos (opcional, para gestión interna)


    def add_trajectory(self, trajectory, label="Trajectory", color='blue', linestyle='-', marker=None):
        """
        Añade una trayectoria al plot.

        Parámetros:
            trajectory (np.array): Array de NumPy con los puntos de la trayectoria.
                                    Forma: [[x0, y0], [x1, y1], ...] para 2D o [[x0, y0, z0], [x1, y1, z1], ...] para 3D.
            label (str, opcional): Etiqueta para la leyenda. Por defecto "Trajectory".
            color (str, opcional): Color de la línea. Por defecto 'blue'.
            linestyle (str, opcional): Estilo de línea. Por defecto '-'.
            marker (str, opcional): Marcador para los puntos. Por defecto None.
        """
        if self.dimension == 3:
            self.ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                         label=label, color=color, linestyle=linestyle, marker=marker)
        else: # 2D
            self.ax.plot(trajectory[:, 0], trajectory[:, 1],
                         label=label, color=color, linestyle=linestyle, marker=marker)
        self.added_elements.append({'type': 'trajectory', 'label': label}) # Rastreo opcional


    def add_cuboid(self, cuboid, label="Cuboid", color='red', alpha=0.2):
        """
        Añade un cuboide 3D al plot.

        Parámetros:
            cuboid: Objeto Cuboid previamente definido (deberías tener una clase Cuboid).
            label (str, opcional): Etiqueta para la leyenda. Por defecto "Cuboid".
            color (str, opcional): Color del cuboide. Por defecto 'red'.
            alpha (float, opcional): Transparencia del cuboide (0 a 1). Por defecto 0.2.
        """
        if self.dimension != 3:
            raise ValueError("Cuboids solo se pueden añadir a plots 3D.")

        # --- Asumiendo que 'cuboid' tiene metodos para obtener vertices ---
        vertices = cuboid.get_vertices_for_plotting() # Necesitas implementar este método en tu clase Cuboid
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
        cuboide_poly = Poly3DCollection(caras, facecolors=color, linewidths=1, edgecolors='black', alpha=alpha)

        self.ax.add_collection3d(cuboide_poly)
        self.added_elements.append({'type': 'cuboid', 'label': label}) # Rastreo opcional


    def customize_plot(self, title=None, xlabel="X", ylabel="Y", zlabel="Z",
                       equal_aspect=False, grid=True, legend=True):
        """
        Personaliza el plot: título, etiquetas de ejes, aspecto, grilla y leyenda.

        Parámetros:
            title (str, opcional): Nuevo título del plot. Si None, usa el título inicial.
            xlabel (str, opcional): Etiqueta para el eje X. Por defecto "X".
            ylabel (str, opcional): Etiqueta para el eje Y. Por defecto "Y".
            zlabel (str, opcional): Etiqueta para el eje Z (solo para plots 3D). Por defecto "Z".
            equal_aspect (bool, opcional): Forzar la misma escala en los ejes (X, Y, Z en 3D). Por defecto False.
            grid (bool, opcional): Mostrar la grilla. Por defecto True.
            legend (bool, opcional): Mostrar la leyenda. Por defecto True.
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
        Muestra el plot en una ventana.
        """
        plt.show()


    def clear_plot(self):
        """
        Limpia el plot, removiendo todos los elementos graficados, pero manteniendo la figura y axes.
        """
        self.ax.cla() # Limpia el axes actual
        self.added_elements = [] # Limpia el registro de elementos añadidos (opcional)
        if self.dimension == 3:
            self.ax.set_zlabel("Z") # Restablecer label eje Z (por si se borra con cla())
        self.customize_plot() # Restablecer personalizaciones basicas (titulo, etiquetas, grid, leyenda)

# ---  Asumiendo que tienes definida la clase Cuboid (de la respuesta anterior) ---
# ---  Y que Cuboid tiene un metodo get_vertices_for_plotting() que retorna 8 vertices 3D ---

class Cuboid: # Ejemplo basico de Cuboid para este Plotter - DEBES IMPLEMENTAR get_vertices_for_plotting
    def __init__(self, punto1, punto2):
        self.punto1 = np.array(punto1)
        self.punto2 = np.array(punto2)

    def get_vertices_for_plotting(self): #  --- IMPLEMENTACION DE EJEMPLO - DEBES ADAPTAR A TU CLASE Cuboid REAL ---
        p1 = self.punto1
        p2 = self.punto2
        vertices = np.array([
            [p1[0], p1[1], p1[2]], #0
            [p2[0], p1[1], p1[2]], #1
            [p2[0], p2[1], p1[2]], #2
            [p1[0], p2[1], p1[2]], #3
            [p1[0], p1[1], p2[2]], #4
            [p2[0], p1[1], p2[2]], #5
            [p2[0], p2[1], p2[2]], #6
            [p1[0], p2[1], p2[2]]  #7
        ])
        return vertices

    # ... (otros métodos de Cuboid si los tienes) ...

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