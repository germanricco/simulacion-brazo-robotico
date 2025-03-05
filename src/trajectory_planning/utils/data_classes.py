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
class SegmentProfile:
    """Perfil de un segmento de trayectoria entre dos puntos"""
    start_state: JointState
    end_state: JointState
    duration: float
    profile_type: type
    constraints: JointConstraints

