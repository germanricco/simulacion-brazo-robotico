import numpy as np

class Cuboid:
    def __init__(self, min_point, max_point):
        """
        Inicializa un objeto Cuboid definido por dos puntos vertices opuestos,
        con validacion de los puntos de entrada.

        Parametros:
            * min_point (array): Punto con coords. minimas (x, y, z)
            * max_poing (array): Punto con coords. maximas (x, y, z)

        Retorna:
            * TypeError: Si alguno de los puntos no es un array
            * ValueError: Si alguno de los puntos no tienen 3 dim, o si alguna coordenada
                de min_point no es menor que la coordenada correspondiente de max_point
        """
        # Validacion de tipo array
        if not isinstance(min_point, (list, tuple, np.ndarray)) or not isinstance(max_point, (list, tuple, np.ndarray)):
            raise TypeError("Los puntos deben ser un array-like (lista, tupla o np.array).")

        min_point_np = np.array(min_point)
        max_point_np = np.array(max_point)

        # Validación de dimensión R3
        if min_point_np.shape != (3,) or max_point_np.shape != (3,):
            raise ValueError("Los puntos deben ser de dimensión 3.")

        # Validación de tipo de datos numérico
        if not np.issubdtype(min_point_np.dtype, np.number) or not np.issubdtype(max_point_np.dtype, np.number):
            raise ValueError("Las coordenadas de los puntos deben ser numericas.")

        # Validación: min_point coordenadas <= max_point coordenadas
        if not np.all(min_point_np < max_point_np):
            raise ValueError("Cada coordenada de min_point debe ser estrictamente menor que su coordenada correspondiente en max_point.")

        self.min_point = np.array(min_point)
        self.max_point = np.array(max_point)

    def get_vertices_for_plotting(self):
        p1 = self.min_point
        p2 = self.max_point
        vertices = np.array([
            [p1[0], p1[1], p1[2]],
            [p2[0], p1[1], p1[2]],
            [p2[0], p2[1], p1[2]],
            [p1[0], p2[1], p1[2]],
            [p1[0], p1[1], p2[2]],
            [p2[0], p1[1], p2[2]],
            [p2[0], p2[1], p2[2]],
            [p1[0], p2[1], p2[2]]
            ])
        return vertices
    
    def contains(self, punto, epsilon=1e-6):
        """
        Verifica si un punto está dentro del Cuboid.

        Parámetros:
            * punto (array-like): Punto 3D a verificar.
            * epsilon (float): Tolerancia para comparación en punto flotante.

        Retorna:
            * bool: True si el punto está dentro, False si no.
        """
        punto_np = np.array(punto)
        if punto_np.shape != (3,):
            raise ValueError(f"El punto debe ser en R3, pero tiene dimensión: {punto_np.shape}")

        # Verifica si cada coordenada es mayor al min. y menor al max
        return (self.min_point[0] - epsilon <= punto_np[0] <= self.max_point[0] + epsilon and
                self.min_point[1] - epsilon <= punto_np[1] <= self.max_point[1] + epsilon and
                self.min_point[2] - epsilon <= punto_np[2] <= self.max_point[2] + epsilon)
    

    def segmento_intersecta_cuboide(self, inicio_segmento, fin_segmento, epsilon=1e-6):
        """
        Verifica si un segmento de línea interseca el Cuboid usando el algoritmo de Liang-Barsky (adaptado a 3D).
        Complejidad: O(1) (tiempo constante).
        
        
        Parámetros:
            * inicio_segmento (array-like): Punto inicial del segmento en R3.
            * fin_segmento (array-like): Punto final del segmento en R3.
            * epsilon (float): Tolerancia para comparaciones de punto flotante.

        Retorna:
            * bool: True si el segmento interseca el Cuboid, False en caso contrario.
        """
        # Convertir a arrays de numpy
        p0 = np.asarray(inicio_segmento)
        p1 = np.asarray(fin_segmento)
        
        # Validación de dimensiones
        if p0.shape != (3,) or p1.shape != (3,):
            raise ValueError("Los puntos deben ser de dimensión 3.")
        
        # 1. Chequear si alguno de los extremos está dentro del cuboide
        if (self.contains(p0, epsilon) or self.contains(p1, epsilon)):
            return True
        
        # 2. Algoritmo Liang-Barsky 3D para intersección segmento-cuboide
        dir_vec = p1 - p0  # Vector dirección del segmento
        t_min, t_max = 0.0, 1.0  # Parámetros t iniciales
        
        # Límites del cuboide (expandidos por epsilon para evitar errores numéricos)
        min_bounds = self.min_point - epsilon
        max_bounds = self.max_point + epsilon
        
        # Para cada dimensión (x, y, z), calcular t de entrada/salida
        for i in range(3):
            if np.abs(dir_vec[i]) < epsilon:
                # Segmento paralelo a los planos de esta dimensión
                if p0[i] < min_bounds[i] or p0[i] > max_bounds[i]:
                    return False  # Segmento fuera del cuboide en esta dimensión
            else:
                # Calcular t para los planos min y max de la dimensión i
                t1 = (min_bounds[i] - p0[i]) / dir_vec[i]
                t2 = (max_bounds[i] - p0[i]) / dir_vec[i]
                
                # Actualizar t_min y t_max
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))
                
                # Salir temprano si no hay solapamiento
                if t_min > t_max:
                    return False
        
        # 3. Verificar si hay solapamiento en el rango [0, 1]
        return t_max >= t_min and t_min <= 1.0 and t_max >= 0.0
        
if __name__ == "__main__":
    # Cuboide de ejemplo: [0,0,0] a [5,5,5]
    mi_cuboid = Cuboid(min_point=np.array([0,0,0]), max_point=np.array([5,5,5]))
    centro_cubo = [2, 2, 2]
    punto_fuera = [6, 6, 0]
    print(f"Un punto interior: {centro_cubo} esta contenido: {mi_cuboid.contains(centro_cubo)}")
    print(f"Un punto exterior: {punto_fuera} esta contenido: {mi_cuboid.contains(punto_fuera)}")

    # Pruebas de validaciones
    # Caso 1: Segmento totalmente dentro
    assert mi_cuboid.segmento_interseca_cuboide([1,1,1], [4,4,4]) == True

    # Caso 2: Segmento que intersecta una cara
    assert mi_cuboid.segmento_interseca_cuboide([-1,-1,2], [6,6,2]) == True

    # Caso 3: Segmento paralelo al eje Z pero fuera en X
    assert mi_cuboid.segmento_interseca_cuboide([-1,2,2], [-1,2,5]) == False

    # Caso 4: Segmento tangente a una esquina
    assert mi_cuboid.segmento_interseca_cuboide([5,5,5], [6,6,6]) == True  # Punto [5,5,5] está en el límite

    assert mi_cuboid.segmento_interseca_cuboide([-2, -2, 0], [2, 2, 0]) == True #Intersecta en ODC
