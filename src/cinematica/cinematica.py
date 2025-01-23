import numpy as np
import sys
from pathlib import Path

# Obtener el directorio raíz del proyecto
project_root = Path(__file__).resolve().parent.parent.parent
algebra_path = project_root / "src" / "algebra_lineal"
sys.path.append(str(algebra_path))

# Intentar importar el módulo
try:
    import transformaciones
    print("Modulo transformaciones importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudo importar el modulo 'transformaciones'")

def cinematica_directa(matriz_dh_params):
    """
    Calcula las posiciones de las articulaciones utilizando parámetros DH.

    Parámetros:
        matriz_dh_params: Matriz numpy de tamaño (n, 4) con los parámetros DH (a, alpha, d, theta)

        siendo n la cantidad de articulaciones o puntos de interes

    Retorna:
        Matriz numpy con las posiciones x, y, z de las articulaciones y la matriz de orientación final
    """
    # Punto inicial (base del brazo) en coordenadas homogéneas
    puntos_homogeneos = np.array([[0, 0, 0, 1]])  # Base en el origen
    
    # Matriz de transformación acumulativa
    T = np.eye(4)

    # Recorrer los parámetros DH y calcular la posición de cada articulación
    for fila in matriz_dh_params:
        a, alpha, d, theta = fila
        T_i = transformaciones.matriz_dh(a, alpha, d, theta)
        T = np.dot(T, T_i)

        # Agregar la posición del extremo (x, y, z, 1) para mantener la dimensión consistente
        nueva_posicion = np.append(T[:3, 3], 1)
        # Agrego a matriz la nueva posición
        puntos_homogeneos = np.vstack((puntos_homogeneos, nueva_posicion))

    # Obtengo la matriz de rotación de la ultima matriz homogenea (Orientación final del TCP)
    R_tcp = T[:3, :3]

    return puntos_homogeneos[:, :3], R_tcp

def calcular_angulos_euler(R):
    """
    Calcula los ángulos de euler (A, B, C) a partir de una matriz de rotación 3x3

    Parámetros:
        R (numpy.ndarray): Matriz de rotación 3x3

    Retorna:
        list: [A, B, C] en radianes
    """

    B = np.arctan2(-R[2, 0], np.sqrt(1 - R[2, 0]**2))
    A = np.arctan2(R[2, 1], R[2, 2])
    C = np.arctan2(R[1, 0], R[0, 0])

    return [A, B, C]

#

if __name__ == "__main__":
    # Prueba con una matriz DH de ejemplo (2 articulaciones)
    matriz_dh_params = np.array([
        [0, np.pi/2, 1, np.radians(0)],  # a, alpha, d, theta
        [1, 0, 0, np.radians(0)],
        [1, 0, 0, np.radians(0)]
    ])

    posiciones, orientacion_tcp = cinematica_directa(matriz_dh_params)
    print("Cada fila representa la posicion de una articulacion")
    print(posiciones)
    print("Matriz de orientacion final: ")
    print(orientacion_tcp)
    angulos_euler = calcular_angulos_euler(orientacion_tcp)
    print(f"Los angulos de Euler (en radianes) son: {angulos_euler}")
    
    #Convierto angulos de euler de rad2deg
    angulos_euler_deg = []
    for angulo in angulos_euler:
        angulos_euler_deg.append(np.degrees(angulo))

    print(f"Los angulos de Euler (en grados) son: {angulos_euler_deg}")