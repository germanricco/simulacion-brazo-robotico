"""
Se busca crear la estructura alámbrica de un robot de 4GL utilizando los parámetros de Denavit-Hartenberg

Se resaltan los puntos:
* GCS (Global Coordinate System)
* MCS (Machine Coordinate System)
* Joints (J1, J2, J3, J4)
* MP (Mounting Point)
* TCP (Tool Center Point)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

import sys
from pathlib import Path

# Obtener el directorio raíz del proyecto
project_root = Path(__file__).resolve().parent.parent
print(f"Project Root: {project_root}")

# Agregar rutas relativas
cinematica_path = project_root / "src" / "cinematica"
sys.path.append(str(cinematica_path))

algebra_lineal_path = project_root / "src" / "algebra_lineal"
sys.path.append(str(algebra_lineal_path))

# Intentar importar el módulo
try:
    import cinematica
    import orientacion
    print("Modulo cinematica importado con exito")
    print("Modulo algebra_lineal importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudieron importar los modulos ")

# Matriz de parametros geométricos y posición inicial
matriz_dh_params = np.array([
        [1, 0, 0, np.radians(0)],       #GCS to MCS
        [0, np.pi/2, 0.5, np.radians(0)],  #MCS to J1
        [1, 0, 0, np.radians(0)],     #J1 to J2
        [0.8, 0, 0, np.radians(0)],      #J2 to J3
        [0.4, 0, 0, np.radians(0)]       #J3 to TCP (wrist to end of tool)
    ])

# Función de actualización de la visualización con sliders
def actualizar(val):
    # Actualizar los angulos deseados (4 gdl)
    matriz_dh_params[1, 3] = np.radians(slider_theta1.val)
    matriz_dh_params[2, 3] = np.radians(slider_theta2.val)
    matriz_dh_params[3, 3] = np.radians(slider_theta3.val)
    matriz_dh_params[4, 3] = np.radians(slider_theta4.val)

    # Obtener posicion y orientacion
    posiciones, R_tcp  = cinematica.cinematica_directa_dh(matriz_dh_params)

    # Convertir orientacion en angulos de euler
    angulos_euler= orientacion.Euler.matriz_a_euler(R_tcp)

    #Conviertir angulos de euler de rad2deg
    angulos_euler_deg = []
    for angulo in angulos_euler:
        angulos_euler_deg.append(np.degrees(angulo))

    ax.cla()  # Limpiar el gráfico
    ax.scatter(posiciones[0,0], posiciones[0,1], posiciones[0,2], color='green', s=100, label="GCS")     # GCS en verde
    ax.plot(posiciones[1:,0], posiciones[1:,1], posiciones[1:,2], 'bo-', linewidth=2, label="Brazo robótico")  # Dibujar el brazo
    ax.scatter(posiciones[-1,0], posiciones[-1,1], posiciones[-1,2], color='red', s=100, label="TCP")       # TCP en rojo

    # Actualizo textos
    ax.text2D(0.05, 0.95, f"Posición TCP: X={posiciones[-1,0]:.2f}, Y={posiciones[-1,1]:.2f}, Z={posiciones[-1,2]:.2f}",
              transform=ax.transAxes, fontsize=12, color='black')
    ax.text2D(0.05, 0.90, f"Ángulos de Euler: A={angulos_euler_deg[0]:.2f}, B={angulos_euler_deg[1]:.2f}, C={angulos_euler_deg[2]:.2f}",
              transform=ax.transAxes, fontsize=10, color='black')

    # Ajustar límites de los ejes y etiquetas
    ax.set_xlim([-2, 4])
    ax.set_ylim([-2, 4])
    ax.set_zlim([0, 5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Simulación de Brazo Robótico")
    ax.legend()
    plt.draw()

# Configuración de la figura 3D
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Ajustar límites iniciales de los ejes
ax.set_xlim([-2, 4])
ax.set_ylim([-2, 4])
ax.set_zlim([-0, 5])

# Visualización inicial
posiciones, R_tcp = cinematica.cinematica_directa_dh(matriz_dh_params)

ax.scatter(posiciones[0,0], posiciones[0,1], posiciones[0,2], color='green', s=100) 
ax.plot(posiciones[1:,0], posiciones[1:,1], posiciones[1:,2], 'bo-', linewidth=2)
ax.scatter(posiciones[-1,0], posiciones[-1,1], posiciones[-1,2], color='red', s=100)

# Agregar Etiquetas y Titulo
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Simulación de Brazo Robótico")

# Crear sliders para los ángulos theta1 y theta2
axcolor = 'lightgoldenrodyellow'
ax_theta1 = plt.axes([0.2, 0.02, 0.65, 0.03], facecolor=axcolor)
ax_theta2 = plt.axes([0.2, 0.06, 0.65, 0.03], facecolor=axcolor)
ax_theta3 = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_theta4 = plt.axes([0.2, 0.14, 0.65, 0.03], facecolor=axcolor)

slider_theta1 = Slider(ax_theta1, 'Theta 1', -180, 180, valinit=0, valstep=1)
slider_theta2 = Slider(ax_theta2, 'Theta 2', -180, 180, valinit=0, valstep=1)
slider_theta3 = Slider(ax_theta3, 'Theta 3', -180, 180, valinit=0, valstep=1)
slider_theta4 = Slider(ax_theta4, 'Theta 4', -180, 180, valinit=0, valstep=1)

# Asignar la función de actualización a los sliders
slider_theta1.on_changed(actualizar)
slider_theta2.on_changed(actualizar)
slider_theta3.on_changed(actualizar)
slider_theta4.on_changed(actualizar)

# Mostrar la visualización interactiva
plt.show()