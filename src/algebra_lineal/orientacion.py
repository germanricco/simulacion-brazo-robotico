import numpy as np

class Euler:
    """
    Clase para trabajar con ángulos de Euler
    Incluye métodos para convertir entre ángulos de Euler y matrices de rotacion
    """

    @staticmethod
    def matriz_a_euler(R):
        """
        Convierte una matriz de rotación 3x3 a ángulos de Euler (A,B,C)

        Parámetros:
            R (np.array): Matriz de rotación 3x3

        Retorna:
            tuple: ( roll(A), pitch(B), yaw(C) ) en radianes.
        """

        B = np.arctan2(-R[2, 0], np.sqrt(1 - R[2, 0]**2))
        A = np.arctan2(R[2, 1], R[2, 2])
        C = np.arctan2(R[1, 0], R[0, 0])

        return A, B, C
    
    @staticmethod
    def euler_a_matriz(roll, pitch, yaw):
        """
        Convierte ángulos de Euler (A,B,C) a una matriz de rotación 3x3.

        Parámetros:
            roll (float): Ángulo de rotación sobre el eje X en radianes.
            pitch (float): Ángulo de rotación sobre el eje Y en radianes.
            yaw (float): Ángulo de rotación sobre el eje Z en radianes.

        Retorna:
            np.array: Matriz de rotación 3x3.
        """
        # Matrices de rotación individuales
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Matriz de rotación compuesta (ZYX)
        R = Rz @ Ry @ Rx
        return R
    
    @staticmethod
    def normalizar_angulos(roll, pitch, yaw):
        """
        Normaliza los ángulos de Euler al rango [-π, π].

        Parámetros:
        roll, pitch, yaw (float): Ángulos de Euler en radianes.

        Retorna:
        tuple: (roll, pitch, yaw) normalizados.
        """
        roll = np.arctan2(np.sin(roll), np.cos(roll))
        pitch = np.arctan2(np.sin(pitch), np.cos(pitch))
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        return roll, pitch, yaw
    

if __name__ == "__main__":
    #Definir ángulos de Euler (en radianes)
    roll, pitch, yaw = 3*np.pi/4, np.pi/6, np.pi/3

    # Convertir a matriz de orientación
    R = Euler.euler_a_matriz(roll, pitch, yaw)
    print("Matriz de orientación:")
    print(R)

    # Convertir de vuelta a ángulos de Euler
    roll_calc, pitch_calc, yaw_calc = Euler.matriz_a_euler(R)
    print("\nAngulos de Euler calculados:")
    print(f"Roll: {roll_calc}, Pitch: {pitch_calc}, Yaw: {yaw_calc}")

    # Normalizar ángulos de Euler
    roll_norm, pitch_norm, yaw_norm = Euler.normalizar_angulos(roll_calc, pitch_calc, yaw_calc)
    print("\nAngulos de Euler normalizados:")
    print(f"Roll: {roll_norm}, Pitch: {pitch_norm}, Yaw: {yaw_norm}")
