import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def graficar_cos():
    # Generar datos
    x = np.linspace(-np.pi, np.pi, 10)  #(inicio, fin, cant.muestras)
    y = np.cos(x)

    # Crear el gráfico
    plt.plot(x, y, label="y = cos(x)")

    # Personalizar el gráfico
    plt.title("Función Coseno")
    plt.xlabel("Eje X")
    plt.ylabel("Eje Y")
    plt.legend()        #toma como legend el label al crear el gráfico
    plt.grid(True)

    # Mostrar el gráfico
    plt.show()

def graficar_dispersion():
    """
    Se utiliza para representar puntos de posiciones en el espacio
    """
    x_pos = np.array([1, 2, 3, 4, 5])
    y_pos = np.array([1, 4, 9, 16, 25])

    # Crear el gráfico de dispersión
    plt.scatter(x_pos, y_pos, color='green', marker='x', label="Puntos de trayectoria")

    # Personalización
    plt.title("Gráfico de Dispersión")
    plt.xlabel("Posición X")
    plt.ylabel("Posición Y")
    plt.legend()
    plt.grid(True)

    plt.show()

def graficar_trayectoria_2D():
    t = np.linspace(0,10,100)
    trayectoria_x = 4 * np.cos(t)
    trayectoria_y = 2 * np.sin(t)

    plt.plot(trayectoria_x, trayectoria_y, label="Trayectoria Circular")
    plt.scatter(trayectoria_x[0], trayectoria_y[0], color="green", label="Inicio")
    plt.scatter(trayectoria_x[-1], trayectoria_y[-1], color='red', label="Final")
    plt.title("Trayectoria de un brazo robótico")
    plt.xlabel("Posición X")
    plt.ylabel("Posición Y")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    #graficar_cos()
    #graficar_dispersion()
    #graficar_trayectoria_2D()

    # ANIMACION
    """"
    # Configurar inicial de figura
    fig, ax = plt.subplots()

    x_data, y_data = [], []
    line, = plt.plot([], [], 'bo-', label="Movimiento") # Puntos iniciales vacíos

    def init():
        ax.set_xlim(0, 14)      # Límites del eje X
        ax.set_ylim(0, 14)      # Límites del eje Y
        line.set_data([], [])  # Vaciar los datos de la linea
        return line,

    def update(frame):
        x_data.append(frame)        #Añadir valor en x
        y_data.append(frame ** 2)
        line.set_data(x_data, y_data)
        return line,

    # frames. se define rango de valores de entrada
    # blit=True optimiza la animación, redibujando solo elementos que cambian
    ani = FuncAnimation(fig, update, frames=np.linspace(0, 3, 100), init_func=init, blit=True)
    plt.title("Animación de Movimiento Cuadrático")
    plt.legend()
    plt.grid(True)
    plt.show()
    """
    # Datos de trayectorias predefinidas
    x_data = np.linspace(0, 10, 100)
    y_data = np.sin(x_data)

    fig, ax = plt.subplots()
    ax.set_xlim(0, 10)
    ax.set_ylim(-1.5, 1.5)
    line, = ax.plot([], [], 'r-', label="Seno")

    # Inicialización
    def init():
        line.set_data([], [])
        return line,

    # Actualización: mostrar la trayectoria punto a punto
    def update(frame):
        line.set_data(x_data[:frame], y_data[:frame])
        return line,

    ani = FuncAnimation(fig, update, frames=len(x_data), init_func=init, blit=True)

    plt.title("Animación de función seno")
    plt.legend()
    plt.grid(True)
    plt.show()