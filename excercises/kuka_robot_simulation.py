import pybullet as p
import numpy as np
import time

# Conectar la simulación
p.connect(p.GUI)

# Especificar una nueva ruta sin caracteres especiales
custom_path = "C:/ProgramData/pybullet_models/"
p.setAdditionalSearchPath(custom_path)

# Cargar el archivo URDF del plano y robot
plane_id = p.loadURDF("plane.urdf")
# Cargar el robot
robot_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])

def move_home(robot_id):
    """
    Coloca todas las articulaciones del robot en 0 radianes.
    """
    num_joints = p.getNumJoints(robot_id)
    for joint in range(num_joints):
        p.setJointMotorControl2(robot_id, joint, p.POSITION_CONTROL, targetPosition=0)
    print("Robot movido a posición home.")

# Agregar un boton a UI para llamar a move_home
home_button_id = p.addUserDebugParameter("Move Home", 1, 0, 0)

# Agregar control deslizante
slider_0 = p.addUserDebugParameter("Articulación 0", -np.pi, np.pi, 0)

slider_1 = p.addUserDebugParameter("Articulación 1", -np.pi, np.pi, 0)

slider_2 = p.addUserDebugParameter("Articulación 2", -np.pi, np.pi, 0)

slider_3 = p.addUserDebugParameter("Articulación 3", -np.pi, np.pi, 0)

slider_4 = p.addUserDebugParameter("Articulación 4", -np.pi, np.pi, 0)

slider_5 = p.addUserDebugParameter("Articulación 5", -np.pi, np.pi, 0)


# Obtener el número de articulaciones
num_joints = p.getNumJoints(robot_id)
print(f"El Robot Husky tiene {num_joints} articulaciones")

# Imprimir información de las articulaciones
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Indice {i}: {joint_info[1].decode()}")

# Establecer gravedad
p.setGravity(0, 0, -9.8)

# Establecer objetivos de posición para varias articulaciones
target_positions = [0.5, -0.5, 1.0, -1.0, 0.5, -0.5, 0.0]
for i in range(len(target_positions)):
    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=target_positions[i])


contador = 0
# Mantener la simulación en ejecución
try:
    while True:
        p.stepSimulation()
        #Leo slider y actualizo posicion
        target_pos_0 = p.readUserDebugParameter(slider_0)
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, targetPosition=target_pos_0)

        target_pos_1 = p.readUserDebugParameter(slider_1)
        p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition=target_pos_1)

        target_pos_2 = p.readUserDebugParameter(slider_2)
        p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition=target_pos_2)

        target_pos_3 = p.readUserDebugParameter(slider_3)
        p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition=target_pos_3)

        target_pos_4 = p.readUserDebugParameter(slider_4)
        p.setJointMotorControl2(robot_id, 4, p.POSITION_CONTROL, targetPosition=target_pos_4)

        target_pos_5 = p.readUserDebugParameter(slider_5)
        p.setJointMotorControl2(robot_id, 5, p.POSITION_CONTROL, targetPosition=target_pos_5)

        if (contador >= 1000):
            base_pos = p.getBasePositionAndOrientation(robot_id)
            print(base_pos)
            end_joint_state = p.getJointState(robot_id, 6)
            print(end_joint_state)
            contador=0
        else:
            contador = contador+1

        # Verificar si se presionó el boton "home"
        if p.readUserDebugParameter(home_button_id) == 1:
            move_home(robot_id)
            # Reiniciar el boton a su estado original
            p.resetUserDebugParameter(home_button_id)

        time.sleep(1/240)   # Simular a 240 Hz
except KeyboardInterrupt:
    print("Simulación terminada por el usuario.")

# Desconectar la simulación antes de salir
p.disconnect()
