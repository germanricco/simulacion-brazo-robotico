import numpy as np

class TrajectoryPlanner():
    def __init__(self, max_jerk, max_accel, max_vel):
        """
        Args:
            max_jerk (float): Jerk máximo permitido [rad/s³]
            max_accel (float): Aceleración máxima permitida [rad/s²]
            max_vel (float): Velocidad máxima permitida [rad/s]
        """
        self.max_jerk = abs(max_jerk)
        self.max_accel = abs(max_accel)
        self.max_vel = abs(max_vel)
    
    def plan_global_trajectory_for_one_joint(self, initial_pos, end_pos, total_time):
        """
        Genera un perfil S-curve entre dos puntos con tiempo total especificado.
        
        Args:
            initial_pos (float): Posición inicial [rad]
            end_pos (float): Posición final [rad]
            total_time (float): Tiempo total del movimiento [s]
            
        Returns:
        """
