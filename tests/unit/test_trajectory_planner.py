"""
Test unitarios para planificador de trayectoria
"""

import unittest
import numpy as np
import os
import sys
from pathlib import Path
from math import isclose
import matplotlib.pyplot as plt
# Importo modulos
src_root = Path(__file__).resolve().parent.parent.parent
print(src_root)
sys.path.append(str(src_root))

from src.trajectory_planning.trajectory_planner import TrajectoryPlanner
from src.trajectory_planning.utils.data_classes import JointConstraints

class TestTrajectoryPlanner(unittest.TestCase):
    """
    Valida implementacion contra ejemplos de referencia.
    "Trajectory Planning for Automatic Machines and Robots - Biagiotti, Melchiorri"
    (Pagina 79 - Trajectory with Double S Velocity Profile)
    """
    def setUp(self):
        """
        Configura el entorno de pruebas
        """
        self.default_constraints = JointConstraints(
            max_velocity=5.0,
            max_acceleration=10.0,
            max_jerk=30.0
        )

        # Instancia de TrajectoryPlanner
        self.trajectory_planner = TrajectoryPlanner(joints_constraints=self.default_constraints)

        # Tolerancia para comparaciones
        self.float_tolerance = 1e-6

    def test_smooth_path(self):
        # Test con array 1D
        path_1d = np.array([-0.78, -0.73, -0.66, -0.6, -0.52, -0.44, -0.35, -0.26, 
                            -0.16, -0.05, 0.05, 0.16, 0.26, 0.35, 0.44, 0.52, 0.6, 
                            0.66, 0.73, 0.78])
        
        # Test con array 2D (una articulación)
        path_2d_single = path_1d.reshape(-1, 1)
        
        print("\nTest con array 2D (una articulación):")
        print(f"Original Path shape: {path_2d_single.shape}")
        smooth_path_2d_single = self.trajectory_planner._smooth_path(positions=path_2d_single, window_size=5)
        print(f"Smooth Path shape: {smooth_path_2d_single.shape}")
        
        # Test con array 2D (múltiples articulaciones)
        # Crear un ejemplo con 3 articulaciones
        path_2d_multi = np.column_stack([path_1d, path_1d * 0.5, path_1d * -0.75])
        
        print("\nTest con array 2D (múltiples articulaciones):")
        print(f"Original Path shape: {path_2d_multi.shape}")
        smooth_path_2d_multi = self.trajectory_planner._smooth_path(positions=path_2d_multi, window_size=5)
        print(f"Smooth Path shape: {smooth_path_2d_multi.shape}")
        
        # Visualizar resultados para la primera articulación (opcional)
        
        plt.figure(figsize=(10, 6))
        plt.plot(path_1d, 'b-', label='Original')
        plt.plot(smooth_path_2d_single, 'r-', label='Suavizada')
        plt.legend()
        plt.title('Comparación de trayectoria original vs suavizada')
        plt.grid(True)
        plt.savefig('smooth_path_comparison.png')
        plt.close()
        
        # Comprobación: las formas de entrada y salida deben coincidir
        self.assertEqual(path_2d_single.shape, smooth_path_2d_single.shape)
        self.assertEqual(path_2d_multi.shape, smooth_path_2d_multi.shape)



if __name__ == '__main__':
    unittest.main()