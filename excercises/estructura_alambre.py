import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

import sys
from pathlib import Path

# Obtener el directorio raíz del proyecto
project_root = Path(__file__).resolve().parent.parent
print(f"Project Root: {project_root}")

# Agregar la ruta relativa al sys.path
algebra_path = project_root / "src" / "algebra_lineal"
sys.path.append(str(algebra_path))
print(f"Ruta Agregada:", algebra_path)
print("Contenido de la carpeta:", list(algebra_path.glob("*.py")))

# Intentar importar el módulo
try:
    import cinematica
    print("Modulo importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudo importar el modulo 'cinematica' ")

matriz_dh_params = np.array([
        [0, np.pi/2, 1, np.radians(0)],  # a, alpha, d, theta
        [1, 0, 0, np.radians(0)],
        [1, 0, 0, np.radians(0)]
    ])

# Función de actualización de la visualización con sliders
def actualizar(val):
    matriz_dh_params[0, 3] = np.radians(slider_theta1.val)  # Actualiza theta1
    matriz_dh_params[1, 3] = np.radians(slider_theta2.val)  # Actualiza theta2
    matriz_dh_params[2, 3] = np.radians(slider_theta3.val)

    x, y, z = cinematica.calcular_posiciones(matriz_dh_params)

    ax.cla()  # Limpiar el gráfico
    ax.plot(x, y, z, 'bo-', linewidth=2, label="Brazo robótico")  # Dibujar el brazo
    ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label="TCP")  # TCP en rojo

    # Ajustar límites de los ejes
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0, 5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Simulación de Brazo Robótico")
    ax.legend()
    plt.draw()

# Configuración de la figura 3D
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Visualización inicial
x, y, z = cinematica.calcular_posiciones(matriz_dh_params)
ax.plot(x, y, z, 'bo-', linewidth=2)
ax.scatter(x[-1], y[-1], z[-1], color='red', s=100)

# Ajustar límites iniciales de los ejes
ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-0, 5])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Simulación de Brazo Robótico")

# Crear sliders para los ángulos theta1 y theta2
axcolor = 'lightgoldenrodyellow'
ax_theta1 = plt.axes([0.2, 0.02, 0.65, 0.03], facecolor=axcolor)
ax_theta2 = plt.axes([0.2, 0.06, 0.65, 0.03], facecolor=axcolor)
ax_theta3 = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)

slider_theta1 = Slider(ax_theta1, 'Theta 1', -180, 180, valinit=0, valstep=1)
slider_theta2 = Slider(ax_theta2, 'Theta 2', -180, 180, valinit=0, valstep=1)
slider_theta3 = Slider(ax_theta3, 'Theta 3', -180, 180, valinit=0, valstep=1)

# Asignar la función de actualización a los sliders
slider_theta1.on_changed(actualizar)
slider_theta2.on_changed(actualizar)
slider_theta3.on_changed(actualizar)

# Mostrar la visualización interactiva
plt.show()