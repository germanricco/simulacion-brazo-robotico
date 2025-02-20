import numpy as np

class Pose:
    """
    Clase auxiliar para representar posicion (x,y,z) y orientacion (A,B,C)
    """
    def __init__(self, position: np.ndarray, orientation: np.ndarray):
        self.position = position
        self.orientation = orientation

def verificar_pose(pose):
    """
    Verifica si el argumento de entrada es una pose valida.

    Parametros:
        * pose: objeto Pose o np.array que describe posicion y orientacion

    Retorna:
        * (bool): True si la pose es valida, False y mensaje de error si no lo es
    """

    # Verifica si es un array
    if isinstance(pose, np.ndarray):
        # Verifica que sea array unidimensional
        if pose.ndim == 1:
            # Verifica longitud de array
            longitud_array = len(pose)
            if longitud_array not in [6,7]:
                print(f"Error: pose(np.array) debe tener longitud 6 o 7, pero tiene longitud {longitud_array}")
                return False
            else:
                return True
        else:
            print(f"Error: pose(np.array) debe ser unidimensional, pero tiene dimension {pose.ndim} ")
            return False
    elif isinstance(pose, Pose):
        return True

    else:
        print(f"Error: pose debe ser un objeto Pose o np.array(), pero es {type(pose)}")
        return False

if __name__ == "__main__":

    # -- Validacion de funcion verificar_pose --

    # Caso 1: pose no es un objeto valido
    assert verificar_pose(pose="np.array()") == False

    # Caso 2: pose es un array pero de dimension incorrecta
    assert verificar_pose(pose=np.array([[1,1,2,1,1,1], [1,3,2,2,2,2]])) == False

    # Caso 3: pose tiene dimension correcta pero longitud incorrecta
    assert verificar_pose(pose=np.array([4,1,1,1])) == False

    # Caso 4: pose es valido en quaterniones
    assert verificar_pose(pose=np.array([0,0,0,0,0,0,1])) == True

    # Caso 5: pose es valido con Euler
    assert verificar_pose(pose=np.array([0,0,0,np.pi,0,0,])) == True

    # Caso 6: pose es objeto Pose valido
    pose = Pose(position=np.array([0,0,0]), orientation=np.array([0,0,0]))
    assert verificar_pose(pose=pose) == True
