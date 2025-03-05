import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional

import sys
from pathlib import Path

# Importo modulos
src_root = Path(__file__).resolve().parent.parent
print(src_root)
sys.path.append(str(src_root))

from trajectory_planning.velocity_profiles import VelocityProfile

@dataclass
class JointState:
    """Estado completo de una articulación en un instante t"""
    time: float
    position: float
    velocity: float
    acceleration: float

@dataclass
class RobotPose:
    """Estado completo del robot (todas articulaciones) en un instante t"""
    time: float
    joints: dict[int, JointState]  # {joint_id: JointState}

# Structured dtype para trayectorias individuales
trajectory_dtype = np.dtype([
    ('time', 'f8'),
    ('position', 'f8'),
    ('velocity', 'f8'),
    ('acceleration', 'f8')
])



class TrajectoryPlanner():
    """
    Coordina multiples articulaciones y puntos de paso.
    Utiliza perfiles de velocidad para cada articulacion
    """
    def __init__(self, joints_path: np.ndarray, profiles: list[VelocityProfile]):
        self.joints_path = joints_path
        self.profiles = profiles
        self.trajectories: dict[int, np.ndarray] = {}
        self.via_points: list[RobotPose]
        
    def compute_trajectories(self):
        """Calcula trayectorias para todas las articulaciones"""
        for joint_id, profile in enumerate(self.profiles):
            self._compute_joint_trajectory(joint_id, profile)

    def _compute_joint_trajectory(self, joint_id: int, profile: VelocityProfile):
        """Calcula y almacena la trayectoria para una articulación"""
        joint_data = []
        
        # Calcular entre cada par de via points
        for i in range(len(self.joints_path)-1):
            q0, q1 = self.joints_path[i, joint_id], self.joints_path[i+1, joint_id]
            profile.plan_trajectory(q0, q1, v0=0, v1=0)
            
            # Generar serie temporal
            time_samples = np.linspace(0, profile.characteristics.total_time, 100)
            joint_segment = np.zeros(len(time_samples), dtype=trajectory_dtype)
            
            for idx, t in enumerate(time_samples):
                state = profile.get_state(t)
                #joint_segment[idx] = (t, state[POSITION_ID], state[SPEED_ID], state[ACCELERATION_ID])
            
            joint_data.append(joint_segment)
        
        self.trajectories[joint_id] = np.concatenate(joint_data)

    def get_combined_trajectory(self) -> np.ndarray:
        """Combina todas las articulaciones en estructura unificada"""
        times = self._get_synchronized_times()
        n_samples = len(times)
        n_joints = len(self.profiles)
        
        #combined = np.zeros(n_samples)
        
        for i, t in enumerate(times):
            positions = np.zeros(n_joints)
            velocities = np.zeros(n_joints)
            accelerations = np.zeros(n_joints)
            
            for joint_id in range(n_joints):
                state = self._get_interpolated_state(joint_id, t)
                positions[joint_id] = state['position']
                velocities[joint_id] = state['velocity']
                accelerations[joint_id] = state['acceleration']
                
            #combined[i] = (t, positions, velocities, accelerations)
        
        #return combined

    
    def add_via_point(self, point: RobotPose):
        """Añade un punto de paso con restricciones"""
        self.via_points.append(point)
        self._optimize_around_via_points()

    def _optimize_around_via_points(self):
        """Ajusta trayectorias para cumplir con via points"""
        # 1. Alinear marcos temporales
        # 2. Aplicar filtros de suavizado
        # 3. Ajustar perfiles de velocidad
        # 4. Validar restricciones cinemáticas
        
    def _apply_singularity_filters(self):
        """Filtrado especial para zonas de singularidad"""
        # Implementar filtros de:
        # - Suavizado de aceleración
        # - Limitación de jerk
        # - Ajuste de velocidad

    def _get_synchronized_times(self) -> np.ndarray:
        """Obtiene vector temporal común para todas las articulaciones"""
        all_times = [t['time'] for traj in self.trajectories.values() for t in traj]
        return np.unique(np.sort(all_times))
    
    def _get_interpolated_state(self, joint_id: int, t: float) -> np.ndarray:
        """Obtiene estado interpolado para tiempo t"""
        traj = self.trajectories[joint_id]
        idx = np.searchsorted(traj['time'], t, side='right') - 1
        return traj[idx]