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
from trajectory_planning.path_planner import PathPlanner
from trajectory_planning.path_generator import PathGenerator
from visualization.plotter import Plotter
from controller.robot_controller import RobotController

# Instancio un objeto Robot
initial_angles = np.array([0, 0, 0, 0, 0, 0])

robot = RobotController(initial_joint_angles=initial_angles)

current_pose = robot.current_tcp_pose
print(f"Posicion Inicial de Robot: {current_pose}")

# Instancio objetos Cuboide y los establezco como zonas segura y prohibida
cuboide_1 = Cuboid(min_point=np.array([-2000,-2000,0]), max_point=np.array([2000,2000,3000]))
safty_zone = Zone(cuboide_1, "Safe")

cuboide_2 = Cuboid(min_point=np.array([1200,1000,0]), max_point=np.array([2000,2000,1000]))
forbidden_zone = Zone(cuboide_2, "Forbidden")

cuboide_3 = Cuboid(min_point=np.array([-3000,-2000,0]), max_point=np.array([-2000,2000,2000]))
safety_zone_2 = Zone(cuboide_3, "Safe")

cuboide_4 = Cuboid(min_point=np.array([-3000,-2000,0]), max_point=np.array([-6,-6,1000]))
forbidden_zone_2 = Zone(cuboide_4, "Forbidden")

# Agrego las zonas a las listas del safety_monitor
safety_monitor = SafetyMonitor(robot=robot)
safety_monitor.add_zone(safty_zone)
safety_monitor.add_zone(safety_zone_2)
safety_monitor.add_zone(forbidden_zone)
safety_monitor.add_zone(forbidden_zone_2)

print(f"Safe limits:\n {safety_monitor._safe_limits}")
print(f"Forbidden limits:\n {safety_monitor._forbidden_limits}")

# Creo path con planner
planner = PathPlanner()

original_path = planner.generate_path(
    path_type='linear',
    start_pose=np.array([1755, -1500, 2660, 0, 0, 0, 1]),
    end_pose=np.array([1755, 1500, 2660, 0, 0, 0, 1]),
    num_poses=20,
    orientation_mode='slerp'
)

# Analizo retorno en un punto
print(f"Posicion 5 de original_path:\n {original_path[5,:]}")

if safety_monitor.verify_tcp_path(original_path[:,:3]):
    print(f"El path del tcp es seguro")

    # Creo el joint_path con path_generator
    path_generator = PathGenerator(robot=robot, path=original_path)
    current_joint_path = path_generator.current_joint_path
    print(f"Posicion 5 de current joint path:\n {current_joint_path[5]}")
    print(path_generator.original_joint_path)

    if safety_monitor.verify_full_body(current_joint_path):
        print(f"El cuerpo no colisiona con las zonas prohibidas")
    
    else:
        print(f"El cuerpo colisiona")

else:
    print(f"El path del tcp es inseguro")

plotter = Plotter("Titulo de Plot", 3)

plotter.add_robot(robot=robot)
plotter.add_pose(current_pose, "TCP")
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_cuboid(cuboide_2, "Zona Prohibida", "red")
plotter.add_cuboid(cuboide_3, "Zona Segura 2", "green")
plotter.add_cuboid(cuboide_4, "Zona Prohibida 2", "red")

plotter.add_trajectory(original_path, "Trayectoria", "black")

plotter.customize_plot(title="Test Plotter", legend=False, equal_aspect=False)
plotter.show_plot()


