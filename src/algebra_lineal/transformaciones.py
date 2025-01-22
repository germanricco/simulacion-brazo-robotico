import numpy as np

def traslacion_3d(dx, dy, dz):
    """
    Devuelve una matriz de traslación 3D.
    """
    return np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])

def rot_x(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def rot_y(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

def matriz_dh(a, alpha, d, theta):
    """
    Calcula la matriz de transformación homogénea usando los parámetros de Denavit-Hartenberg (DH).

    a: Longitud del enlace
    alpha: Ángulo de torsion (rad)
    d: Desplazamiento a lo largo del eje z
    theta: Ángulo de rotación de la articulación (rad)
    
    """
    matriz_homogenea = np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return np.round(matriz_homogenea, decimals=12)

# Prueba de la funcion al ejecutar script
if __name__ == "__main__":

    angulo = np.pi/4 #45 grados
    vector1 = np.array([
        [1],
        [0],
        [0]
        ])
    
    print("Matriz de traslacion 3D:")
    print(traslacion_3d(0, 1, 2))

    # Prueba de las matrices de rotacion en 3D
    print("Matriz de rotacion sobre eje X (45 grados):")
    print(rot_x(angulo))

    print("Matriz de rotacion sobre eje Y (45 grados):")
    print(rot_y(angulo))

    print("Matriz de rotacion sobre eje Z (45 grados):")
    print(rot_z(angulo))

    print("Rotacion vector (1,0,0) 45 grados antihorario (+): ")
    vector2 = np.dot(rot_z(angulo), vector1)
    print(vector2)

    print("Rotacion vector (1,0,0) 45 grados horario (-): ")
    vector2 = np.dot(rot_z(angulo).T, vector1)
    print(vector2)