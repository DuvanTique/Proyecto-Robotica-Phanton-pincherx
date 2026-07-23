from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
def generate_launch_description():
    # -------------------------------------------------------------------------
    #  Choose SIM vs REAL explicitly
    # -------------------------------------------------------------------------
    use_real_robot = LaunchConfiguration("use_real_robot")
    start_clasificador = LaunchConfiguration("start_clasificador")
    enable_rviz = LaunchConfiguration("enable_rviz")

    use_real_robot_arg = DeclareLaunchArgument(
        "use_real_robot",
        default_value="false",
        description=(
            "If 'true', connect MoveIt to the real PhantomX via "
            "pincher_control/follow_joint_trajectory. "
            "If 'false', use ros2_control simulation."
        ),
    )

    # Argumento opcional para iniciar el clasificador con máquina de estados.
    start_clasificador_arg = DeclareLaunchArgument(
        "start_clasificador",
        default_value="false",
        description=(
            "Si es 'true', arranca el nodo clasificador con una máquina de estados. "
            "Se recomienda activarlo cuando se ejecute en conjunto con el bringup de visión."
        ),
    )

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Si es 'false', no arranca RViz (útil para ejecución headless en Raspberry Pi).",
    )

    # -------------------------------------------------------------------------
    #  Paths (shared)
    # -------------------------------------------------------------------------
    urdf_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_description"),
        "urdf",
        "phantomx_pincher.urdf.xacro",
    ])

    move_group_launch_path = PathJoinSubstitution([
        FindPackageShare("phantomx_pincher_moveit_config"),
        "launch",
        "move_group.launch.py",
    ])

    # Define ros2_control_plugin based on use_real_robot
    from launch.substitutions import PythonExpression
    ros2_control_plugin = PythonExpression([
        "'real' if '", use_real_robot, "' == 'true' else 'fake'"
    ])
    
    # Disable ros2_control in URDF if using real robot (since we use pincher_control node)
    ros2_control_arg = PythonExpression([
        "'false' if '", use_real_robot, "' == 'true' else 'true'"
    ])

    # Robot description string (xacro → URDF)
    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_path,
            " ros2_control_plugin:=", ros2_control_plugin,
            " ros2_control:=", ros2_control_arg,
        ]),
        value_type=str,
    )

    # -------------------------------------------------------------------------
    #  Common nodes (both SIM and REAL)
    # -------------------------------------------------------------------------

    # Robot State Publisher (only for real robot mode; in sim, move_group.launch.py provides its own)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
        }],
        condition=IfCondition(use_real_robot),
    )

    # SRDF
    _robot_description_semantic_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("phantomx_pincher_moveit_config"),
                    "srdf",
                    "phantomx_pincher.srdf.xacro",
                ]
            ),
            " ",
            "prefix:=", "phantomx_pincher_",
            " ",
            "name:=", "phantomx_pincher",
            " ",
            "use_real_gripper:=", use_real_robot,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            _robot_description_semantic_xml,
            value_type=str,
        )
    }

    # Kinematics
    # We need to load the yaml file. Since we don't have the load_yaml helper here, 
    # we can import yaml and use get_package_share_directory directly or copy the helper.
    # For simplicity, let's just add the import and helper or do it inline.
    # But wait, 'load_yaml' is not defined in this file.
    # I will add the load_yaml function at the top or use a simpler approach if possible.
    # Actually, let's just add the parameters to the node and assume we can load the file.
    # I'll add the load_yaml helper function to this file first.
    
    kinematics = load_yaml(
        "phantomx_pincher_moveit_config", "config/kinematics.yaml"
    )

    # Commander (MoveGroupInterface wrapper) – always run
    commander_node = Node(
        package="phantomx_pincher_commander_cpp",
        executable="commander",
        name="commander",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            robot_description_semantic,
            kinematics,
        ],
    )

    # Clasificador / máquina de estados: se lanza opcionalmente. Este nodo
    # implementa la lógica de alto nivel para el pick & place utilizando una
    # máquina de estados y puede pausar la visión durante la ejecución.
    clasificador_node = Node(
        package="pincher_control",
        executable="clasificador_node",
        name="clasificador_node",
        output="screen",
        parameters=[{
            "fsm_enabled": True,
            "pause_vision_during_execution": True,
        }],
        condition=IfCondition(start_clasificador),
    )

    # Nodo que agrega objetos de colisión (mesa, canecas, bandeja) a la
    # planning scene de MoveIt para validación de colisiones.
    scene_objects_node = Node(
        package="pincher_control",
        executable="scene_objects",
        name="scene_objects",
        output="screen",
    )

    # -------------------------------------------------------------------------
    #  SIMULATION stack (ros2_control fake hardware)
    # -------------------------------------------------------------------------

    # -------------------------------------------------------------------------
    #  SIMULATION stack — delegated to move_group.launch.py
    #  (ros2_control_node + controller spawners are managed there)
    # -------------------------------------------------------------------------

    move_group_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path]),
        launch_arguments={
            # Let move_group.launch.py manage everything for simulation
            "ros2_control": "true",
            "ros2_control_plugin": "fake",
            # Let move_group.launch.py spawn controllers
            "manage_controllers": "true",
            # We want MoveIt RViz
            "enable_rviz": enable_rviz,
            # Disable servo (not needed for simulation testing)
            "enable_servo": "false",
        }.items(),
        condition=UnlessCondition(use_real_robot),
    )

    # -------------------------------------------------------------------------
    #  REAL HARDWARE stack (pincher_control follow_joint_trajectory)
    # -------------------------------------------------------------------------

    follow_joint_trajectory_node = Node(
        package="pincher_control",
        executable="follow_joint_trajectory",
        name="pincher_follow_joint_trajectory",
        output="screen",
        # Optional: override defaults if needed
        # parameters=[{
        #     "port": "/dev/ttyUSB0",
        #     "baudrate": 1000000,
        #     "joint_prefix": "phantomx_pincher_",
        #     "moving_speed": 200,
        #     "torque_limit": 400,
        #     "gripper_id": 5,
        # }],
        condition=IfCondition(use_real_robot),
    )

    move_group_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([move_group_launch_path]),
        launch_arguments={
            # Do NOT let MoveIt start ros2_control
            "ros2_control": "false",
            # Matches the mode you used manually with move_group.launch.py
            "ros2_control_plugin": "real",
            # pincher_control provides the action servers, so MoveIt won't spawn controllers
            "manage_controllers": "false",
            "enable_rviz": enable_rviz,
            # Disable servo (incompatible params in Jazzy)
            "enable_servo": "false",
        }.items(),
        condition=IfCondition(use_real_robot),
    )

    # -------------------------------------------------------------------------
    #  LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription([
        use_real_robot_arg,
        start_clasificador_arg,
        enable_rviz_arg,

        # Common
        robot_state_publisher_node,
        commander_node,
        # Opcional: clasificador con máquina de estados
        clasificador_node,
        # Collision objects para la planning scene
        scene_objects_node,

        # SIM-only (move_group.launch.py manages ros2_control + controllers)
        move_group_sim,

        # REAL-only
        follow_joint_trajectory_node,
        move_group_real,
    ])
