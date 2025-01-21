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
husky_id = p.loadURDF("husky/husky.urdf", basePosition=[-2,0,0])
#r2d2_id = p.loadURDF("r2d2.urdf", basePosition=[0, 1, 2])
print("URDFs cargados correctamente")

# Obtener el número de articulaciones
num_joints = p.getNumJoints(husky_id)
print(f"El Robot Husky tiene {num_joints} articulaciones")

# Imprimir información de las articulaciones
for i in range(num_joints):
    joint_info = p.getJointInfo(husky_id, i)
    print(f"Indice {i}: {joint_info[1].decode()}")

# Establecer gravedad
p.setGravity(0, 0, -9.8)

# Mover la articulacion 2 (rueda delatera izquierda) a un ángulo de 2*pi radiantes
#p.setJointMotorControl2(husky_id, 2, p.POSITION_CONTROL, targetPosition=4*np.pi)

"""
p.setJointMotorControl2(robot_id, indice, modo_control, targetPosition)
Hay 3 modos de control disponibles:
1. p.POSITION_CONTROL
2. p.VELOCITY_CONTROL
3. p.TORQUE_CONTROL
"""

# Establecer objetivos de posición para varias articulaciones
#target_positions = [0, 0, 2*np.pi, 2*np.pi, 2*np.pi, 2*np.pi, 0.0]
#for i in range(len(target_positions)):
#    p.setJointMotorControl2(husky_id, i, p.POSITION_CONTROL, targetPosition=target_positions[i])


# Aplicar velocidad a una articulación específica
p.setJointMotorControl2(husky_id, 2, p.VELOCITY_CONTROL, targetVelocity=4, force=50)
p.setJointMotorControl2(husky_id, 3, p.VELOCITY_CONTROL, targetVelocity=4, force=50)
p.setJointMotorControl2(husky_id, 4, p.VELOCITY_CONTROL, targetVelocity=4, force=50)
p.setJointMotorControl2(husky_id, 5, p.VELOCITY_CONTROL, targetVelocity=4, force=50)

# Estado actual de articulación
joint_state = p.getJointState(husky_id, 2)
print(f"Posición actual de la articulación 2: {joint_state[0]}")
print(f"Velocidad actual de la articulación 2: {joint_state[1]}")

"""
# Resetear la posición inicial
p.resetBasePositionAndOrientation(husky_id, [2, 0, 1], [0, 0, 0, 1])
print("Reseteando la simulación...")
"""

# Mantener la simulación en ejecución
try:
    while True:
        p.stepSimulation()  # Mantener la simulación en ejecución
        time.sleep(1/240)   # Simular a 240 Hz
except KeyboardInterrupt:
    print("Simulación terminada por el usuario.")



# Desconectar la simulación antes de salir
p.disconnect()