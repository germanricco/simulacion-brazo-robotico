import numpy as np

class TransformacionHomogenea:
    """
    Clase para generar matrices de transformación homogénea 4x4.
    Incluye métodos para rotaciones sobre los ejes X, Y, Z y traslaciones.
    """

    # Métodos estáticos para reducir el consumo de recursos
    @staticmethod
    def rotacion_x(angulo, dx=0, dy=0, dz=0):
        """
        Genera una matriz de transformación homogénea para una rotación sobre el eje X.

        Parámetros:
        angulo (float): Ángulo de rotación en radianes.
        dx, dy, dz (float): Desplazamientos en los ejes X, Y, Z (opcional).

        Retorna:
        np.array: Matriz de transformación homogénea 4x4.
        """
        cos_theta = np.cos(angulo)
        sin_theta = np.sin(angulo)

        R = np.array([
            [1, 0, 0, dx],
            [0, cos_theta, -sin_theta, dy],
            [0, sin_theta, cos_theta, dz],
            [0, 0, 0, 1]
        ])
        return R

    @staticmethod
    def rotacion_y(angulo, dx=0, dy=0, dz=0):
        """
        Genera una matriz de transformación homogénea para una rotación sobre el eje Y.

        Parámetros:
        angulo (float): Ángulo de rotación en radianes.
        dx, dy, dz (float): Desplazamientos en los ejes X, Y, Z (opcional).

        Retorna:
        np.array: Matriz de transformación homogénea 4x4.
        """
        cos_theta = np.cos(angulo)
        sin_theta = np.sin(angulo)

        R = np.array([
            [cos_theta, 0, sin_theta, dx],
            [0, 1, 0, dy],
            [-sin_theta, 0, cos_theta, dz],
            [0, 0, 0, 1]
        ])
        return R

    @staticmethod
    def rotacion_z(angulo, dx=0, dy=0, dz=0):
        """
        Genera una matriz de transformación homogénea para una rotación sobre el eje Z.

        Parámetros:
        angulo (float): Ángulo de rotación en radianes.
        dx, dy, dz (float): Desplazamientos en los ejes X, Y, Z (opcional).

        Retorna:
        np.array: Matriz de transformación homogénea 4x4.
        """
        cos_theta = np.cos(angulo)
        sin_theta = np.sin(angulo)

        R = np.array([
            [cos_theta, -sin_theta, 0, dx],
            [sin_theta, cos_theta, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]
        ])
        return R

    @staticmethod
    def traslacion(dx, dy, dz):
        """
        Genera una matriz de transformación homogénea para una traslación.

        Parámetros:
        dx, dy, dz (float): Desplazamientos en los ejes X, Y, Z.

        Retorna:
        np.array: Matriz de transformación homogénea 4x4.
        """
        T = np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]
        ])
        return T
    
    def matriz_dh(a, alpha, d, theta):
        """
        Calcula la matriz de transformación homogénea usando los parámetros de Denavit-Hartenberg (DH).

        Parámetros:
            a (float): Longitud del enlace
            alpha (float): Ángulo de torsion en radianes
            d (float): Desplazamiento a lo largo del eje z
            theta (float): Ángulo de rotación de la articulación en radianes

        Retorna:
            np.array: Matriz de transformación homogénea 4x4
        
        """
        matriz_homogenea = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return np.round(matriz_homogenea, decimals=12)



# Prueba de la funcion al ejecutar script
#if __name__ == "__main__":


