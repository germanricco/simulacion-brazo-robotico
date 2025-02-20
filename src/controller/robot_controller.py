import numpy as np
from scipy.spatial.transform import Rotation as R

import sys
from pathlib import Path

# Importar Modulos
project_root = Path(__file__).resolve().parent.parent.parent
src_path = project_root / "src" 
sys.path.append(str(src_path))

from algebra_lineal.euler import Euler
from algebra_lineal import matrices_rotacion
from algebra_lineal.transformaciones import TransformacionHomogenea

class RobotController:
    def __init__(self, initial_joint_angles):
        """
        Controlador de robot

        Argumentos:
            * initial_joint_angles: Angulos iniciales de las articulaciones en radianes
        """
        self.joint_angles = np.array(initial_joint_angles)
        self.geometrical_parameters =np.array([
            [0, 0, 650],
            [400, 0, 680],
            [0, 0, 1100],
            [766, 0, 230],
            [345, 0, 0],
            [244, 0, 0]
            ])
        self.current_tcp_pose, self.joints_positions = self.forward_kinematics(self.joint_angles)
        self.urdf_file = []
        self.moving = False

    def forward_kinematics(self, theta_angles):
        """ 
        Calcula la posición y orientación del TCP dado los ángulos de las ariticulaciones.

        Parámetros:
            * theta_angles (np.array): Lista de ángulos de las articulaciones en radianes

        Retorna:
            * tcp_pose (np.array): Lista con posicion y orientacion de TCP [x,y,z,qx,qy,qz,qw]
            * joint_positions (np.array): matriz con posiciones XYZ de todas las articulaciones
        """
        num_articulaciones = len(theta_angles)

        if num_articulaciones not in [4, 6]:
            raise ValueError("La cantidad de ángulos debe ser 4 o 6.")
        
        T_total = np.eye(4)
        joint_positions = [np.zeros(3)] #Posicion inicial (MCS)

        for i in range(num_articulaciones):
            theta = theta_angles[i]
            parametros = self.geometrical_parameters[i,:]

            if i == 0:
                T = TransformacionHomogenea.rotacion_z(theta, parametros[0], parametros[1], parametros[2])
            elif i in [1, 2, 4]:
                T = TransformacionHomogenea.rotacion_y(theta, parametros[0], parametros[1], parametros[2])
            elif i in [3, 5]:
                T = TransformacionHomogenea.rotacion_x(theta, parametros[0], parametros[1], parametros[2])

            T_total = T_total @ T  # Multiplica las matrices de transformación

            # Guardar la posicion de la articulacion actual
            joint_positions.append(T_total[:3, 3].copy())

        """
        # Matriz para ubicar robot en coordenadas globales (from GCS to MCS)
        T_base = transformaciones.TransformacionHomogenea.traslacion(2, 0, 0)
        
        # Matriz para herramienta (from WP to TCP)
        T_tool = transformaciones.TransformacionHomogenea.traslacion(0.2, 0, 0)
        #R_tool = T_tool[:3, :3]

        # posicion final del TCP, teniendo en cuenta herramienta y GCS
        T_tcp = T_base @ T_total @ T_tool
        """

        #! Procesar pose final del TCP
        posicion_tcp = np.round(T_total[:3, 3], decimals=1)
        R_tcp = np.round(T_total[:3, :3], decimals=6)

        # Para obtener orientacion con quaterniones
        rotacion = R.from_matrix(R_tcp)
        quaternion = rotacion.as_quat()

        # Para obtener orientacion en Euler
        #angulos_euler = Euler.matriz_a_euler(R_tcp)
        #angulos_euler_deg = np.rad2deg(angulos_euler)

        # Creo objeto Pose para TCP
        #tcp_pose = Pose(position=posicion_tcp, orientation=angulos_euler_deg)

        tcp_pose = np.concatenate((posicion_tcp, quaternion), axis=0)
        joint_positions = np.round(joint_positions, decimals=1)

        return tcp_pose, joint_positions
    

    def inverse_kinematics(self, goal_tcp_pose):
        """
        Calcula los ángulos de las articulaciones para alcanzar una posición y orientación deseada.
        
        Parámetros:
            * goal_tcp_pose (np.array): pose deseada de TCP. Puede ser: 
                [x,y,z,A,B,C] siendo A,B,C angulos de euler en radianes
                [z,y,z,qx,qy,qz,qw] siendo [qx,qy,qz,qw] un quaternion

        Retorna:
            * (np.array): Ángulos de las articulaciones en radianes.
        """

        posicion_deseada_tcp = goal_tcp_pose[:3]
        orientacion_deseada_tcp = goal_tcp_pose[3:]

        #! Parte 1. Dividir el robot en 2 partes (inferior y superior). Decoupling 
        # Las articulaciones superiores (4, 5 y 6) no afectan la posicion del WP

        # Verifico si la orientacion se describe con euler o quaternions
        if len(orientacion_deseada_tcp) == 3:
            print(f"orientacion en Euler: {orientacion_deseada_tcp}")
            # Obtengo matriz de rotación desde la base hasta el TCP
            R_tcp = Euler.euler_a_matriz(orientacion_deseada_tcp[0], orientacion_deseada_tcp[1], orientacion_deseada_tcp[2])
        elif len(orientacion_deseada_tcp) == 4:
            print(f"orientacion en quaterniones: {orientacion_deseada_tcp}")
            rotacion = R.from_quat(orientacion_deseada_tcp)
            R_tcp = rotacion.as_matrix()
        else:
            raise TypeError("Orientacion debe quedar expresada en Euler o Quaterniones")

        # Extraigo x
        x_vec = np.array([
            [R_tcp[0,0]],
            [R_tcp[1,0]],
            [R_tcp[2,0]]
        ]).T    #Transpuesta para hacer el producto despues

        # Encuentro WP
        a6x = self.geometrical_parameters[5,0]
        posicion_wp = posicion_deseada_tcp - a6x*x_vec

        #! Parte 2. Dada la posicion del wp, obtener J1, J2 y J3
        
        # Obtengo J1 simplificando el problema en plano 2D (j1 solo afecta posicion x e y)
        J1 = np.arctan2(posicion_wp[0,1], posicion_wp[0,0])
        # 2 Problemas: hay 2 soluciones válidas (FRONT, BACK) y si WPx=WPy=0 hay infinitas soluciones

        WPxy = np.sqrt(np.power(posicion_wp[0,0],2) + np.power(posicion_wp[0,1],2))
        #Eq. 4-3
        l = WPxy - self.geometrical_parameters[1,0]
        #Eq. 4-5
        h = posicion_wp[0,2] - self.geometrical_parameters[0,2] - self.geometrical_parameters[1,2]
        #Eq. 4-6
        rho = np.sqrt(np.power(np.linalg.norm(h),2) + np.power(np.linalg.norm(l),2))
        #Eq. 4-7
        b4x = np.sqrt(np.power(self.geometrical_parameters[3,2],2) + np.power((self.geometrical_parameters[3,0]+self.geometrical_parameters[4,0]),2))

        #! Restricciones Eq. 4-8 y 4-9
        if (rho <= (self.geometrical_parameters[2,2] + b4x) and rho >= abs(self.geometrical_parameters[2,2] - b4x) ):
            #Eq. 4-10
            cos_beta = (np.power(rho,2) + np.power(self.geometrical_parameters[2,2],2) - np.power(b4x,2)) / (2*rho*self.geometrical_parameters[2,2])
            beta = np.arctan2(np.sqrt(1- np.power(cos_beta,2)), cos_beta)
            alpha = np.arctan2(h,l)
            #Eq. 4-11
            J2 = np.pi/2-alpha-beta

            # Obtengo J3
            #Eq. 4-12
            cos_gamma = (np.power(self.geometrical_parameters[2,2],2) + np.power(b4x,2) - np.power(rho,2)) / (2*self.geometrical_parameters[2,2]*b4x)
            #Eq. 4-13
            delta = np.arctan2((self.geometrical_parameters[3,0]+self.geometrical_parameters[4,0]),self.geometrical_parameters[3,2])

            #Eq. 4-14
            J3 = np.pi - np.arccos(cos_gamma) - delta

            #! Parte 3. Solve the Wrist (J4, J5 y J6)
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
        
    def get_joints_for_plotting(self):
        return self.joints_positions
        
    def get_status(self):
        pass

    def move_to_goal(self):
        pass

    def execute_move_step(self):
        pass

    def ptp_move(self):
        pass

    def validate_joint_angles(self):
        pass

    def send_to_actuators(self):
        pass

    def emergency_stop(self):
        pass
        

if __name__ == "__main__":
    print(f" \n-- Test Cinematica Directa --\n")
    initial_angles = np.array([0, 0, 0, 0, 0, 0])
    robot = RobotController(initial_joint_angles=initial_angles)
    print(f"Current TCP_Pose: {robot.current_tcp_pose}")
    print(f"Joint Positions: {robot.get_joints_for_plotting()}")

    print(f" \n-- Test Cinematica Inversa --\n")
    print(f"Utilizando angulos de Euler: ")
    goal_tcp_position = np.array([600, -1000, 3300])
    goal_tcp_orientation = np.array([np.deg2rad(250), np.deg2rad(0), np.deg2rad(-90)])

    goal_tcp_pose = np.concatenate((goal_tcp_position, goal_tcp_orientation), axis=0)

    joint_angles = robot.inverse_kinematics(goal_tcp_pose=goal_tcp_pose)
    joint_angles_deg = np.round(np.rad2deg(joint_angles),decimals=1)
    print(f"Joint Angles: {joint_angles_deg}")

    print(f"Utilizando Quaterniones: ")

    rotacion = R.from_euler('xyz', goal_tcp_orientation)
    goal_quat = rotacion.as_quat()

    goal_tcp_pose_quat = np.concatenate((goal_tcp_position, goal_quat), axis=0)

    joint_angles = robot.inverse_kinematics(goal_tcp_pose=goal_tcp_pose_quat)
    print(joint_angles)
    joint_angles_deg = np.round(np.rad2deg(joint_angles),decimals=1)
    print(f"Joint Angles usando Quaterniones: {joint_angles_deg}")
