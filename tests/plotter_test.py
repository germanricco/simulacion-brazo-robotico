import numpy as np

import sys
from pathlib import Path

# Importo modulos
project_root = Path(__file__).resolve().parent.parent
src_root = project_root / "src"
sys.path.append(str(src_root))

from collision_detection.geometric_objects import Cuboid
from collision_detection.zone import Zone
from collision_detection.safety_monitor import SafetyMonitor
from trajectory_planning.bezier_path import BezierPath
from visualization.plotter import Plotter


# Instancio un objeto Cuboide y lo establezco como zona segura
cuboide_1 = Cuboid(min_point=np.array([0,0,0]), max_point=np.array([5,5,5]))
safty_zone = Zone(cuboide_1, "Safe")

trajectory = BezierPath()
path, control_points = trajectory.calc_3points_bezier_path(np.array([-2,0,0]), np.array([2,2,3]), np.array([4,4,4]))

#! Deberia hacer que el path sea un parametro del objeto trayectoria instanciado
#print(path)

# Agrego la zona segura a la lista del safety_monitor
safety_monitor = SafetyMonitor()
safety_monitor.add_zone(safty_zone)

is_safe = safety_monitor.is_trajectory_safe(path)
print(f"La trayectoria es segura? {is_safe}")

plotter = Plotter("Titulo de Plot", 3)
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_trajectory(path)
plotter.customize_plot(title="Nueva Ploteo", legend=False)
plotter.show_plot()

