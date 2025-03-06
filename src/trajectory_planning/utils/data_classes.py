import numpy as np
from typing import Callable
from dataclasses import dataclass

@dataclass
class TrajectoryCharacteristics:
    """Caracteristicas claves de la trayectoria calculada"""
    total_time: float
    achieved_max_acceleration: float
    achieved_max_deceleration: float
    achieved_max_velocity: float

@dataclass
class JointConstraints:
    """Clase contenedora de restricciones cinemáticas de una articulación"""
    max_velocity: float
    max_acceleration: float
    max_jerk: float

@dataclass
class JointState:
    """Estado completo de una articulación en un instante t"""
    time: float
    position: float
    velocity: float
    acceleration: float

@dataclass
class SegmentConstraints:
    """Restricciones inmutables del segmento"""
    q_start: float
    q_end: float
    v_start: float
    v_end: float
    # Debidas a la articulaciones
    jerk_max: float
    accel_max: float
    vel_max: float

@dataclass
class SegmentProfile:
    """Perfil de un segmento de trayectoria entre dos puntos"""
    # Metadata
    segment_id: int
    joint_id: int

    # Parametros de S-Curve
    constraints: SegmentConstraints
    Tj1: float
    Ta: float
    Tj2: float
    Td: float
    Tv: float

    # Funcion de trayectoria precomputada
    trajectory_func: Callable[[float], np.ndarray]

    #? Cache de valores criticos
    _time_vector: np.ndarray = None
    _position_profile: np.ndarray = None
    _velocity_profile: np.ndarray = None
    _acceleration_profile: np.ndarray = None

    @property
    def total_time(self) -> float:
        """Tiempo total del segmento"""
        return self.Ta + self.Td + self.Tv
    
    @property
    def max_velocity(self) -> float:
        """Velocidad máxima alcanzada"""
        return self.constraints.v_start + self.Ta*self.constraints.accel_max - self.constraints.jerk_max*(self.Tj1**2)/2

    def sample_trajectory(self, sample_rate: float = 0.01) -> None:
        """Pre-muestrea la trayectoria para acceso rápido"""
        self._time_vector = np.arange(0, self.total_time, sample_rate)
        self._position_profile = np.array([self.trajectory_func(t)[0] for t in self._time_vector])
        self._velocity_profile = np.array([self.trajectory_func(t)[1] for t in self._time_vector])
        self._acceleration_profile = np.array([self.trajectory_func(t)[2] for t in self._time_vector])