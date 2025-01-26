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
    import orientacion
    print("Modulos importados con exito")
except ModuleNotFoundError:
    print("Error: No se pudieron importar los modulos")

def cinematica_directa_dh(matriz_dh_params):
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
        T_i = transformaciones.TransformacionHomogenea.matriz_dh(a, alpha, d, theta)
        T = np.dot(T, T_i)

        # Agregar la posición del extremo (x, y, z, 1) para mantener la dimensión consistente
        nueva_posicion = np.append(T[:3, 3], 1)
        # Agrego a matriz la nueva posición
        puntos_homogeneos = np.vstack((puntos_homogeneos, nueva_posicion))

    # Obtengo la matriz de rotación de la ultima matriz homogenea (Orientación final del TCP)
    R_tcp = T[:3, :3]

    return puntos_homogeneos[:, :3], R_tcp

def cinematica_directa(angulos_theta):
    """
    Calcula la posición y orientación del TCP dado los ángulos de las ariticulaciones.

    Parámetros:
        angulos_theta (list): Lista de ángulos de las articulaciones en radianes [theta1, theta2, theta3, theta4]

    Retorna:
        tuple: (posicion, orientacion)
            * posicion (np.array): Coordenadas (x, y, z) del TCP
            * orientacion (np.array): Matriz de orientación 3x3 del TCP
    """

    # Longitudes de los Eslabones (Ajustar según robot)
    a1x=0
    a1y=0
    a1z=0.3

    a2x=0
    a2y=0
    a2z=1

    a3x=1
    a3y=0
    a3z=0

    a4x=0.4
    a4y=0
    a4z=0

    # Obtengo los ángulos
    theta1, theta2, theta3, theta4 = angulos_theta

    # Matrices de transformación homogénea para cada articulación
    # Rotación sobre eje Z
    T1 = transformaciones.TransformacionHomogenea.rotacion_z(theta1, a1x, a1y, a1z)
    T2 = transformaciones.TransformacionHomogenea.rotacion_y(theta2, a2x, a2y, a2z)
    T3 = transformaciones.TransformacionHomogenea.rotacion_y(theta3, a3x, a3y, a3z)
    T4 = transformaciones.TransformacionHomogenea.rotacion_y(theta4, a4x, a4y, a4z)

    # Matriz de transformación homogenea para robot (from MCS to WP)
    T04 = T1 @ T2 @ T3 @ T4

    # Matriz para ubicar robot en coordenadas globales (from GCS to MCS)
    T_base = transformaciones.TransformacionHomogenea.traslacion(2, 0, 0)
    
    # Matriz para herramienta (from WP to TCP)
    T_tool = transformaciones.TransformacionHomogenea.traslacion(0.2, 0, 0)
    #R_tool = T_tool[:3, :3]

    # posicion final del TCP, teniendo en cuenta herramienta y GCS
    T_tcp = T_base @ T04 @ T_tool

    # Extraer la posición del TCP (última columna de T_total)
    posicion = T_tcp[:3, 3]
    orientacion = T04[:3, :3]
    
    return posicion, orientacion


if __name__ == "__main__":
    print("\n --- TEST cinematica_directa_dh() --- \n")
    # matriz de parámetros dh de prueba
    matriz_dh_params = np.array([
        [0, np.pi/2, 1, np.radians(0)],  # a, alpha, d, theta
        [1, 0, 0, np.radians(0)],
        [1, 0, 0, np.radians(0)]
    ])
    posiciones, orientacion_tcp = cinematica_directa_dh(matriz_dh_params)
    print("Cada fila representa la posicion de una articulacion:")
    print("Posiciones:", posiciones)
    print("Matriz de orientacion final: ", orientacion_tcp)

    angulos_euler = orientacion.Euler.matriz_a_euler(orientacion_tcp)
    print(f"Los angulos de Euler (en radianes) son: {angulos_euler}")
    
    #Convierto angulos de euler de rad2deg
    angulos_euler_deg = []
    for angulo in angulos_euler:
        angulos_euler_deg.append(np.degrees(angulo))
    print(f"Los angulos de Euler (en grados) son: {angulos_euler_deg}")


    print("\n --- TEST cinematica_directa() --- \n")
    angulos = [0, 0, 0, 0]
    posicion, orientacion_tcp = cinematica_directa(angulos)
    print("Posicion TCP:", posicion)
    print("Matriz de Orientacion TCP: \n", orientacion_tcp)

    angulos_euler = orientacion.Euler.matriz_a_euler(orientacion_tcp)
    print(f"Los angulos de Euler (en radianes) son: {angulos_euler}")
    
    #cinematica_inversa(posicion)