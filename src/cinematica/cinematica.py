import numpy as np

import sys
from pathlib import Path

# Obtener el directorio raíz del proyecto
project_root = Path(__file__).resolve().parent.parent.parent
print(f"Project Root: {project_root}")

# Agregar la ruta relativa al sys.path
algebra_path = project_root / "src" / "algebra_lineal"
sys.path.append(str(algebra_path))
#print(f"Ruta Agregada:", algebra_path)
#print("Contenido de la carpeta:", list(algebra_path.glob("*.py")))

# Intentar importar el módulo
try:
    import transformaciones
    print("Modulo transformaciones importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudo importar el modulo 'transformaciones'")

# Función principal para calcular las posiciones de los puntos a partir de la matriz DH
def calcular_posiciones(matriz_dh_params):
    """
    Calcula las posiciones de las articulaciones utilizando parámetros DH.
    Parámetros:
        matriz_dh_params: Matriz numpy de tamaño (n, 4) con los parámetros DH (a, alpha, d, theta)
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
        T_i = transformaciones.matriz_dh(a, alpha, d, theta)  # Obtener matriz homogénea para la articulación actual
        T = np.dot(T, T_i)  # Acumulación de la transformación

        # Agregar la posición del extremo (x, y, z, 1) para mantener la dimensión consistente
        nueva_posicion = np.append(T[:3, 3], 1)
        # Stackeo matriz en forma vertical, agregando la nueva posición
        puntos_homogeneos = np.vstack((puntos_homogeneos, nueva_posicion))

    # Obtengo la matriz de rotación de la ultima matriz homogenea (Orientación final del TCP)
    orientacion_tcp = T[:3, :3]

    # Transponer la matriz de puntos para devolver las coordenadas en columnas [x, y, z]
    return puntos_homogeneos[:, :3].T, orientacion_tcp # Devuelve las posiciones y la matriz de orientación final


if __name__ == "__main__":
    # Prueba con una matriz DH de ejemplo (2 articulaciones)
    matriz_dh_params = np.array([
        [0, np.pi/2, 1, np.radians(0)],  # a, alpha, d, theta
        [1, 0, 0, np.radians(0)],
        [1, 0, 0, np.radians(0)]
    ])

    posiciones, orientacion_tcp = calcular_posiciones(matriz_dh_params)
    print("Coordenadas: X en fila 0, Y en fila 1, Z en fila 2")
    print(posiciones)
    print("Matriz de orientacion final: ")
    print(orientacion_tcp)