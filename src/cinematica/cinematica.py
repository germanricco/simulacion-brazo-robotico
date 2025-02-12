"""
Modulo para cinematica de un Brazo Robotico

Bibliografia: "Industrial Robotic Control - Fabrizio Frigeni"
"""

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
    import Euler
    import matrices_rotacion
    print("Modulos importados con exito")
except ModuleNotFoundError:
    print("Error: No se pudieron importar los modulos")

def cinematica_directa_dh(matriz_dh_params):
    """
    Calcula las posiciones de las articulaciones utilizando parámetros DH.

    Parámetros:
        * matriz_dh_params: Matriz numpy de tamaño (n, 4) con los parámetros DH (a, alpha, d, theta)

    Retorna:
        * Matriz numpy con las posiciones x, y, z de las articulaciones y la matriz de orientación final
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

def cinematica_directa(angulos_theta, parametros_geometricos):
    """ 
    Calcula la posición y orientación del TCP dado los ángulos de las ariticulaciones.

    Parámetros:
        * angulos_theta (list): Lista de ángulos de las articulaciones en radianes [theta1, theta2, theta3, theta4]
        * parametros_geometricos (np.array): Matriz con parámetros geométricos del robot [a1x, a1y, a1z]

    Retorna:
        * posicion (np.array): Coordenadas (x, y, z) del TCP
        * orientacion (np.array): Matriz de orientación 3x3 del TCP
    """

    num_articulaciones = len(angulos_theta)

    if (num_articulaciones != 4 and num_articulaciones != 6):
        raise ValueError("La cantidad de ángulos debe ser 4 o 6.")
    
    T_total = np.eye(4)

    for i in range(num_articulaciones):
        theta = angulos_theta[i]
        parametros = parametros_geometricos[i,:]
        #print(f"Fila: {i}, Parametros: {parametros}")

        if i == 0:
            T = transformaciones.TransformacionHomogenea.rotacion_z(theta, parametros[0], parametros[1], parametros[2])
        elif (i==1 or i==2 or i==4):
            T = transformaciones.TransformacionHomogenea.rotacion_y(theta, parametros[0], parametros[1], parametros[2])
        elif (i==3 or i==5):
            T = transformaciones.TransformacionHomogenea.rotacion_x(theta, parametros[0], parametros[1], parametros[2])

        T_total = T_total @ T  # Multiplica las matrices de transformación

    """
    # Estoy tengo que agregarlo en otro lado

    # Matriz para ubicar robot en coordenadas globales (from GCS to MCS)
    T_base = transformaciones.TransformacionHomogenea.traslacion(2, 0, 0)
    
    # Matriz para herramienta (from WP to TCP)
    T_tool = transformaciones.TransformacionHomogenea.traslacion(0.2, 0, 0)
    #R_tool = T_tool[:3, :3]

    # posicion final del TCP, teniendo en cuenta herramienta y GCS
    T_tcp = T_base @ T_total @ T_tool
    """

    # Extraer la posición del TCP (última columna de T_total)
    posicion = np.round(T_total[:3, 3], decimals=1)
    orientacion = np.round(T_total[:3, :3], decimals=6)
    
    return posicion, orientacion

def cinematica_inversa(posicion_deseada_tcp, orientacion_deseada_tcp, parametros_geometricos):
    """
    Calcula los ángulos de las articulaciones para alcanzar una posición y orientación deseada.
    
    Parámetros:
        * posicion_deseada: np.array([x, y, z]), posición deseada de TCP en coordenadas cartesianas.
        * orientacion_deseada: np.array([roll, pitch, yaw]), orientación deseada del TCP en ángulos de Euler.
        * parametros_geometricos (np.array): Matriz con los parámetros geométricos del robot (a1x, a1y, a1z, ..., a4x, a4y, a4z).
    
    Retorna:
        * (np.array): Ángulos de las articulaciones en radianes.
    """

    a1 = np.array([parametros_geometricos[0,0], parametros_geometricos[0,1], parametros_geometricos[0,2]])
    a2 = np.array([parametros_geometricos[1,0], parametros_geometricos[1,1], parametros_geometricos[1,2]])
    a3 = np.array([parametros_geometricos[2,0], parametros_geometricos[2,1], parametros_geometricos[2,2]])
    a4 = np.array([parametros_geometricos[3,0], parametros_geometricos[3,1], parametros_geometricos[3,2]])

    theta1, theta2, theta3, theta4 = 0, 0, 0, 0

    #! Parte 1. Dividir el robot en 2 partes (inferior y superior). Decoupling 
    # Las articulaciones superiores (4, 5 y 6) no afectan la posicion del WP

    # Obtengo matriz de rotación desde la base hasta el TCP
    R_tcp = Euler.Euler.euler_a_matriz(orientacion_deseada_tcp[0], orientacion_deseada_tcp[1], orientacion_deseada_tcp[2])
    print("R_tcp: ", R_tcp)

    # Extraigo x
    x_vec = np.array([
        [R_tcp[0,0]],
        [R_tcp[1,0]],
        [R_tcp[2,0]]
    ]).T    #Transpuesta para hacer el producto despues

    # Encuentro WP
    a6x = parametros_geometricos[5,0]
    posicion_wp = posicion_deseada_tcp - a6x*x_vec

    print(f"posicion_wp: {posicion_wp}")
    
    #! Parte 2. Dada la posicion del wp, obtener J1, J2 y J3
    
    # Obtengo J1 simplificando el problema en plano 2D (j1 solo afecta posicion x e y)
    J1 = np.arctan2(posicion_wp[0,1], posicion_wp[0,0])
    print(f"J1: {J1}")
    # 2 Problemas: hay 2 soluciones válidas (FRONT, BACK) y si WPx=WPy=0 hay infinitas soluciones

    #Fila 0, Columna 0
    print(f"posicion WPx: {posicion_wp[0,0]}")

    WPxy = np.sqrt(np.power(posicion_wp[0,0],2) + np.power(posicion_wp[0,1],2))
    #Eq. 4-3
    l = WPxy - parametros_geometricos[1,0]
    #Eq. 4-5
    h = posicion_wp[0,2] - parametros_geometricos[0,2] - parametros_geometricos[1,2]
    #Eq. 4-6
    rho = np.sqrt(np.power(np.linalg.norm(h),2) + np.power(np.linalg.norm(l),2))
    #Eq. 4-7
    b4x = np.sqrt(np.power(parametros_geometricos[3,2],2) + np.power((parametros_geometricos[3,0]+parametros_geometricos[4,0]),2))
    print(f"WPxy: {WPxy} | l: {l} | h: {h} | rho: {rho} | b4x: {b4x}")

    #! Restricciones Eq. 4-8 y 4-9
    if (rho <= (parametros_geometricos[2,2] + b4x) and rho >= abs(parametros_geometricos[2,2] - b4x) ):
        #Eq. 4-10
        cos_beta = (np.power(rho,2) + np.power(parametros_geometricos[2,2],2) - np.power(b4x,2)) / (2*rho*parametros_geometricos[2,2])
        beta = np.arctan2(np.sqrt(1- np.power(cos_beta,2)), cos_beta)
        alpha = np.arctan2(h,l)
        #Eq. 4-11
        J2 = np.pi/2-alpha-beta

        # Obtengo J3
        #Eq. 4-12
        cos_gamma = (np.power(parametros_geometricos[2,2],2) + np.power(b4x,2) - np.power(rho,2)) / (2*parametros_geometricos[2,2]*b4x)
        #Eq. 4-13
        delta = np.arctan2((parametros_geometricos[3,0]+parametros_geometricos[4,0]),parametros_geometricos[3,2])

        #Eq. 4-14
        J3 = np.pi - np.arccos(cos_gamma) - delta

        #! Parte 3. Solve the Wrist (J4, J5 y J6)
        #Eq. 4-15 (R_tcp ya lo obvtuve anteriormente)

        # Busco R_arm (orientación del brazo, antes de la muñeca)
        #Eq. 4-16
        R_arm = matrices_rotacion.rot_z(J1) @ matrices_rotacion.rot_y(J2+J3)
        
        # Propiedad. para matrices de rotación la inversa es igual a la transpuesta
        R_wrist = R_arm.T @ R_tcp

        # +-sqrt dependiendo de la configuración deseada
        J5 = np.arctan2(np.sqrt(1-np.power(R_wrist[0,0],2)), R_wrist[0,0])

        if (R_wrist[0,0] != 1):
            # +- dependiendo configuración deseada
            J4 = np.arctan2(R_wrist[1,0],-R_wrist[2,0])
            J6 = np.arctan2(R_wrist[0,1],R_wrist[0,2])
        else:
            # Singularidad. Debemos fijar J4, por ejemplo manteniendo el valor anterior, en este caso puesto en 0
            J4 = 0
            J6 = np.arctan2(R_wrist[2,1],R_wrist[2,2])
        
        joint_angles = np.array([J1, J2, J3, J4, J5, J6])

        return joint_angles

    else:
        print(f"Posición imposible de alcanzar. ")
        return None


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

    angulos_euler = Euler.Euler.matriz_a_euler(orientacion_tcp)
    print(f"Los angulos de Euler (en radianes) son: {angulos_euler}")
    
    #Convierto angulos de euler de rad2deg
    angulos_euler_deg = []
    for angulo in angulos_euler:
        angulos_euler_deg.append(np.degrees(angulo))
    print(f"Los angulos de Euler (en grados) son: {angulos_euler_deg}")


    print("\n --- TEST cinematica_directa() --- \n")

    print("Para un robot de 6GL")
    # Longitudes de los Eslabones (Ajustar según robot)
    parametros_geometricos = np.array([
        [0, 0, 650],
        [400, 0, 680],
        [0, 0, 1100],
        [766, 0, 230],
        [345, 0, 0],
        [244, 0, 0]
    ])

    angulos = [np.deg2rad(-46), np.deg2rad(46), np.deg2rad(46), np.deg2rad(46), np.deg2rad(46), np.deg2rad(46)]
    posicion, orientacion_tcp = cinematica_directa(angulos, parametros_geometricos)
    print("Posicion TCP:", posicion)
    #print("Matriz de Orientacion TCP: \n", orientacion_tcp)

    angulos_euler = Euler.Euler.matriz_a_euler(orientacion_tcp)
    #print(f"Los angulos de Euler (en radianes) son: {angulos_euler}")
    angulos_euler_deg = np.rad2deg(angulos_euler)
    print(f"Los angulos de Euler (en grados) son: {angulos_euler_deg}")

    posicion_deseada_tcp = np.array([600, -1000, 3300])
    orientacion_deseada_tcp = np.array([np.deg2rad(250), np.deg2rad(0), np.deg2rad(-90)])
    
    joint_angles = cinematica_inversa(posicion_deseada_tcp, orientacion_deseada_tcp, parametros_geometricos)

    joint_angles_deg = np.round(np.rad2deg(joint_angles),decimals=1)
    print(f"Joint Angles: \n {joint_angles_deg}")

