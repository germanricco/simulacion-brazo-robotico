import numpy as np

import sys
from pathlib import Path

# Importo modulos de planificacion de trayectoria
project_root = Path(__file__).resolve().parent.parent.parent
print(f"Project Root: {project_root}")

colisiones_path = project_root / "src" / "deteccion_colisiones"
planificacion_path = project_root / "src" / "planificacion_trayectoria"
plot_path = project_root / "src" / "visualizacion_matplotlib"
sys.path.append(str(colisiones_path))
sys.path.append(str(planificacion_path))
sys.path.append(str(plot_path))

try:
    import objetos_geometricos
    import safety_monitor
    import Zone
    print("Modulo deteccion_colisiones importado con exito")
    import simple_path
    import bezier_path
    print("Modulo planificacion_trayectoria importado con exito")
    import Plotter
    print("Modulo Plotter importado con exito")
except ModuleNotFoundError:
    print("Error: No se pudieron importar los modulos")

# Instancio un objeto Cuboide y lo establezco como zona segura
cuboide_1 = objetos_geometricos.Cuboid(min_point=np.array([0,0,0]), max_point=np.array([5,5,5]))
safty_zone = Zone.Zone(cuboide_1, "Safe")

# Creo una trayecetoria lineal
#trajectory = SimplePath.SimplePath()
#path = trajectory.calc_linear_path([0,0,0], [2,2,2], 4)

trajectory = bezier_path.BezierPath()
path, control_points = trajectory.calc_3points_bezier_path(np.array([-2,0,0]), np.array([2,2,3]), np.array([4,4,4]))

#! Deberia hacer que el path sea un parametro del objeto trayectoria instanciado
#print(path)

# Agrego la zona segura a la lista del safety_monitor
safety_monitor = safety_monitor.SafetyMonitor()
safety_monitor.add_zone(safty_zone)

is_safe = safety_monitor.is_trajectory_safe(path)
print(f"La trayectoria es segura? {is_safe}")

plotter = Plotter.Plotter("Titulo de Plot", 3)
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_trajectory(path)
plotter.customize_plot(title="Nueva Ploteo", legend=False)
plotter.show_plot()

