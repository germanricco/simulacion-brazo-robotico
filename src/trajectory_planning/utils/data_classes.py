import numpy as np
from typing import Callable
from dataclasses import dataclass

ACCELERATION_ID = 0
SPEED_ID = 1
POSITION_ID = 2

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
    profile_type: str

    # Parametros de S-Curve inmutables
    constraints: SegmentConstraints
    Tj1: float
    Ta: float
    Tj2: float
    Td: float
    Tv: float

    # Funcion de trayectoria precomputada
    trajectory_func: Callable[[float], np.ndarray]

    # Atributos internos
    def __post_init__(self):
        self._sampled = False
        self._time_vector: np.ndarray = None
        self._position_profile: np.ndarray = None
        self._velocity_profile: np.ndarray = None
        self._acceleration_profile: np.ndarray = None

    # Caracteristicas dinamicas
    #duration: float
    #synchronized: bool = False

    # En ScurveProfile y perfiles dinamicos
    @property
    def total_time(self) -> float:
        """Tiempo total del segmento"""
        if self.profile_type == "scurve":
            return max(self.Ta + self.Td + self.Tv, 1e-9)  # Evitar division por cero
        elif self.profile_type == "dwell":
            return self.Tv
        elif self.profile_type == "constant_velocity":
            return self.Tv
        else:
            raise ValueError("Tipo de perfil no soportado")
    @property
    def limit_velocity(self) -> float:
        """Velocidad máxima alcanzada en el segmento"""
        return self.constraints.v_start + (self.Ta-self.Tj1)*self.max_acceleration

    @property
    def max_acceleration(self) -> float:
        return self.constraints.jerk_max*self.Tj1
    
    @property
    def max_deceleration(self) -> float:
        return self.constraints.jerk_max*-self.Tj2
    
    @property
    def position_profile(self) -> np.ndarray:
        if not self.is_sampled:
            self.sample_trajectory()
        return self._position_profile
    
    @property
    def velocity_profile(self) -> np.ndarray:
        if not self.is_sampled:
            self.sample_trajectory()
        return self._velocity_profile
    
    @property
    def acceleration_profile(self) -> np.ndarray:
        if not self.is_sampled:
            self.sample_trajectory()
        return self._acceleration_profile

    @property
    def is_sampled(self) -> bool:
        return self._sampled

    def sample_trajectory(self, sample_rate: float = 0.01) -> None:
        """Muestrea la trayectoria para acceso rápido"""
        # Validacion
        if sample_rate <= 0:
            raise ValueError("La tasa de muestreo debe ser mayor a cero")
        
        if np.isnan(self.total_time) or self.total_time <= 0:
            raise ValueError("El tiempo total del segmento debe ser mayor a cero")
        
       # Generar vector temporal con np.linspace (mejor precisión)
        num_samples = int(np.ceil(self.total_time / sample_rate)) + 1
        self._time_vector = np.linspace(0, self.total_time, num=num_samples)

        self._position_profile = np.array([self.trajectory_func(t)[POSITION_ID] for t in self._time_vector])
        self._velocity_profile = np.array([self.trajectory_func(t)[SPEED_ID] for t in self._time_vector])
        self._acceleration_profile = np.array([self.trajectory_func(t)[ACCELERATION_ID] for t in self._time_vector])

        self._sampled = True