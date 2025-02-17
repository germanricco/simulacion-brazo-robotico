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
from trajectory_planning.simple_path import SimplePath
from visualization.plotter import Plotter


# Instancio un objeto Cuboide y lo establezco como zona segura
cuboide_1 = Cuboid(min_point=np.array([0,0,0]), max_point=np.array([10,10,10]))
safty_zone = Zone(cuboide_1, "Safe")

cuboide_2 = Cuboid(min_point=np.array([3,3,0]), max_point=np.array([7,7,4]))
forbidden_zone = Zone(cuboide_2, "Forbidden")

unsafe_trajectory = SimplePath()
unsafe_path = unsafe_trajectory.calc_linear_path(np.array([1,5,3]), np.array([9,5,3]), num_points=2)

safe_trajectory = BezierPath()
safe_path, control_points = safe_trajectory.calc_3points_bezier_path(np.array([1,5,3]), np.array([5,5,6]), np.array([9,5,3]))

# Agrego la zona segura a la lista del safety_monitor
safety_monitor = SafetyMonitor()
safety_monitor.add_zone(safty_zone)
safety_monitor.add_zone(forbidden_zone)

print(f"La trayectoria 1 es segura? {safety_monitor.is_trajectory_safe(unsafe_path)}")
print(f"La trayectoria 2 es segura? {safety_monitor.is_trajectory_safe(safe_path)}")

plotter = Plotter("Titulo de Plot", 3)
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_cuboid(cuboide_2, "Zona Prohibida", "red")
plotter.add_trajectory(unsafe_path, "Trayectoria 1", "orange")
plotter.add_trajectory(safe_path, "Trayectoria 2", "blue")
plotter.customize_plot(title="Test Zonas", legend=True, equal_aspect=False)
plotter.show_plot()

