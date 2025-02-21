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

# Agrego las zonas a las listas del safety_monitor
safety_monitor = SafetyMonitor(robot=robot)
safety_monitor.add_zone(safty_zone)
safety_monitor.add_zone(forbidden_zone)

planner = PathPlanner()

original_path = planner.generate_path(
    path_type='linear',
    start_pose=current_pose,
    end_pose=np.array([200, 1500, 500, 0, 0, 0, 1]),
    num_poses=10,
    orientation_mode='slerp'
)

print(f"Pose 5 de original_path:\n {original_path[5,:]}")
print(f"La trayectoria es segura? {safety_monitor.static_check(original_path)}")

path_generator = PathGenerator(robot=robot, path=original_path)
joint_original_path = path_generator.original_joint_path
print(f"Joint Path:\n {joint_original_path}")


plotter = Plotter("Titulo de Plot", 3)

plotter.add_robot(robot=robot)
plotter.add_pose(current_pose, "TCP")
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_cuboid(cuboide_2, "Zona Prohibida", "red")

plotter.add_trajectory(original_path, "Trayectoria", "black")

plotter.customize_plot(title="Test Plotter", legend=False, equal_aspect=False)
plotter.show_plot()


