import numpy as np
import matplotlib.pyplot as plt
# Para animación 2D
from matplotlib.animation import FuncAnimation
# Para animación 3D
from mpl_toolkits.mplot3d import Axes3D

def animacion_trayectoria_2D(x_data, y_data, tiempo):
    # Configuracion inicial de figura
    fig, ax = plt.subplots()
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)

    line, = ax.plot([], [], 'r-', label="Trayectoria")

    # Inicialización
    def init():
        line.set_data([], [])
        return line,

    # Actualización: mostrar la trayectoria punto a punto
    def update(frame):
        line.set_data(x_data[:frame], y_data[:frame])
        return line,

    ani = FuncAnimation(fig, update, frames=len(tiempo), init_func=init, blit=True)

    plt.title("Animación de Trayectoria 2D")
    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def animacion_trayectoria_3D(x_data, y_data, z_data, tiempo):
    # Configuración inicial de figura en 3D
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # Definir los límites del espacio 3D
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.set_zlim(-4, 4)

    # Crear la línea de trayectoria vacía
    line, = ax.plot([], [], [], 'r-', label="Trayectoria 3D")

    # Inicialización de la animación
    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        return line,

    # Actualización de la trayectoria punto a punto
    def update(frame):
        line.set_data(x_data[:frame], y_data[:frame])   # Actualizar X, Y
        line.set_3d_properties(z_data[:frame])          # Actualizar Z
        return line,

    ani = FuncAnimation(fig, update, frames=len(tiempo), init_func=init, blit=False)

    ax.set_title("Animación de Trayectoria 3D")
    ax.set_xlabel("Eje X")
    ax.set_ylabel("Eje Y")
    ax.set_zlabel("Eje Z")
    ax.legend()
    ax.grid(True)

    plt.show()

if __name__ == "__main__":
    tiempo = np.linspace(0, 10, 100)
    x_data = 2 * np.sin(tiempo)
    y_data = 2 * np.cos(tiempo)
    z_data = tiempo / 2  # Movimiento en altura
    
    #animacion_trayectoria_2D(x_data, y_data, tiempo)
    
    animacion_trayectoria_3D(x_data, y_data, z_data, tiempo)
    