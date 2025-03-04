import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional




class TrajectoryPlanner():
    """
    Coordina multiples articulaciones y puntos de paso.
    Utiliza perfiles de velocidad para cada articulacion
    """
    def __init__(self, joints_path, profiles):
        self.profiles = profiles
        self.joints_path = joints_path

    
