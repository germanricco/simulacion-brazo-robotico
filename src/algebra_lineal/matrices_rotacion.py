import numpy as np
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

if __name__ == "__main__":
    angulo = np.pi/4

    # Prueba de las matrices de rotacion en 3D
    print("Matriz de rotacion sobre eje X (45 grados):")
    print(rot_x(angulo))

    print("Matriz de rotacion sobre eje Y (45 grados):")
    print(rot_y(angulo))

    print("Matriz de rotacion sobre eje Z (45 grados):")
    print(rot_z(angulo))

    vector1 = np.array([
        [1],
        [0],
        [0]
        ])

    print("Rotacion vector (1,0,0) 45 grados antihorario (+): ")
    vector2 = np.dot(rot_z(angulo), vector1)
    print(vector2)

    print("Rotacion vector (1,0,0) 45 grados horario (-): ")
    vector2 = np.dot(rot_z(angulo).T, vector1)
    print(vector2)