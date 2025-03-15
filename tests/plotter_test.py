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
from trajectory_planning.trajectory_planner import TrajectoryPlanner
from trajectory_planning.utils.data_classes import JointConstraints
from visualization.plotter import Plotter
from visualization.trajectory_plotter import TrajectoryPlotter
from controller.robot_controller import RobotController

# Instancio un objeto Robot
initial_angles = np.array([0, 0, 0, 0, 0, 0])
robot = RobotController(initial_joint_angles=initial_angles)

current_pose = robot.current_tcp_pose
print(f"Posicion Inicial de Robot (x,y,z,jx,jy,jz,jw)\n{current_pose}")

# Instancio objetos Cuboide y los establezco como zonas segura y prohibida
cuboide_1 = Cuboid(min_point=np.array([-2000,-2000,0]), max_point=np.array([2000,2000,3000]))
safty_zone = Zone(cuboide_1, "Safe")

cuboide_2 = Cuboid(min_point=np.array([1200,1000,0]), max_point=np.array([2000,2000,1000]))
forbidden_zone = Zone(cuboide_2, "Forbidden")

cuboide_3 = Cuboid(min_point=np.array([-3000,-2000,0]), max_point=np.array([-2000,2000,2000]))
safety_zone_2 = Zone(cuboide_3, "Safe")

cuboide_4 = Cuboid(min_point=np.array([-3000,-2000,0]), max_point=np.array([-100,-100,1000]))
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
print(f"Posicion 5 de original_path:(x,y,z,jx,jy,jz,jw)\n {original_path[5,:]}")

if safety_monitor.verify_tcp_path(original_path[:,:3]):
    print(f"El path del tcp es seguro. Se genera path en joint-space")

    # Instancio PathGenerator y genero path
    path_generator = PathGenerator(robot=robot, path=original_path)
    current_joint_path = path_generator.current_joint_path

    print(f"Posicion 5 de current joint path:\n {current_joint_path[5]}")
    print(f"Path Completo: \n{path_generator.original_joint_path}")
    print(f"Longitud de Path = {path_generator.original_path_lenght}")

    if safety_monitor.verify_full_body(current_joint_path):
        print(f"El cuerpo no colisiona con las zonas prohibidas")

        joints_constraints = {
            0: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10),
            1: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10),
            2: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10),
            3: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10),
            4: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10),
            5: JointConstraints(max_velocity=10, max_acceleration=10, max_jerk=10)
        }

        # Creo instancia de TrajectoryPlanner
        trajectory_planner = TrajectoryPlanner(joints_constraints=joints_constraints)
        trajectory_planner.process_joints_path(current_joint_path)
        
    else:
        print(f"El cuerpo colisiona. Volver a planificar path")
else:
    print(f"El path del tcp es inseguro")

plotter = Plotter("Titulo de Plot", 3)
    
plotter.add_robot(robot=robot)
plotter.add_pose(current_pose, "TCP")
plotter.add_cuboid(cuboide_1, "Zona Segura", "green")
plotter.add_cuboid(cuboide_2, "Zona Prohibida", "red")
plotter.add_cuboid(cuboide_3, "Zona Segura 2", "green")
plotter.add_cuboid(cuboide_4, "Zona Prohibida 2", "red")
plotter.add_path(original_path, "Trayectoria", "black")

trajectory_plotter = TrajectoryPlotter()

num_joints = 5
for joint_id in range(num_joints):
    trajectory_plotter.add_joint_trajectory(joint_id, trajectory_planner.joint_profiles[joint_id])
    trajectory_plotter.plot(joint_id)


plotter.customize_plot(title="Test Plotter", legend=False, equal_aspect=False)
plotter.show_plot()


