import numpy as np

# Crear un array del 1 al 20

array = np.arange(1,21)
print(array)

# Convertir en matriz 4x5
matriz = array.reshape(4, 5)
print(matriz)

#Nota: Si hubiera usado reshape(3,5) me daría error porque no se utilizan los 20 elementos

# Mostrar Propiedades
print("Forma:", matriz.shape)
print("Tamanio:", matriz.size)
print("Tipo de dato", matriz.dtype)

# Extraer segunda fila completa
segunda_fila = matriz[1,:]
# Extraer hasta la fila 2 (0, 1)
filas = matriz[:2]

# Extraer una columna completa
segunda_columna = matriz[:,1]

# Diagonal Principal
diagonal = matriz.diagonal()

# Imprimo en pantalla
print("Segunda Fila: ", segunda_fila)
print("Primeras Dos Filas: ", filas)
print("Segunda Columna: ", segunda_columna)
print("Diagonal Principal: ", diagonal)

# OPERACIONES MATEMATICAS ELEMENTALES
A = np.array([[1, 2], [3, 4]])
B = np.array([[5, 6], [7, 8]])

suma = A + B
resta = A - B
multiplicacion = A * B #elemento a elemento

print("Suma:\n",suma)
print("Resta:\n", resta)
print("Multiplicacion elemento a elemento:\n", multiplicacion)

suma_filas = A.sum(axis=1)
suma_columnas = B.sum(axis=0)

print("Suma de filas de matriz A:", suma_filas)
print("Suma de columnas de matriz B:", suma_columnas)