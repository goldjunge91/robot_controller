from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

from robot_utils.utils import find_device_port


def generate_launch_description():
    configuration = LaunchConfiguration("configuration")
    controller_config = LaunchConfiguration("controller_config")
    manipulator_serial_port = LaunchConfiguration("manipulator_serial_port")
    mecanum = LaunchConfiguration("mecanum")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    robot_description = LaunchConfiguration("robot_description")
    use_sim = LaunchConfiguration("use_sim", default="False")
    include_nerf_launcher = LaunchConfiguration("include_nerf_launcher", default="False")

    base_controller_prefix = PythonExpression(
        ["'mecanum_drive' if ", mecanum, " else 'diff_drive'"]
    )
    manipulator = PythonExpression(["'", configuration, "'.startswith('manipulation')"])
    manipulator_prefix = PythonExpression(["'manipulator_' if ", manipulator, " else ''"])
    controller_config_file = PythonExpression(
        ["'", base_controller_prefix, "' + '_' + '", manipulator_prefix, "' + 'controller.yaml'"]
    )
    default_controller_config = PathJoinSubstitution(
        [FindPackageShare("robot_controller"), "config", robot_model, controller_config_file]
    )

    declare_controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller_config,
        description="Path to controller configuration file.",
    )

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description=(
            "Specify configuration packages. Currently only robot XL has available packages."
        ),
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    default_manipulator_serial_port = find_device_port("0403", "6014", "/dev/ttyUSB0")
    declare_manipulator_serial_port_arg = DeclareLaunchArgument(
        "manipulator_serial_port",
        default_value=default_manipulator_serial_port,
        description="Port to connect to the manipulator.",
    )

    default_mecanum_value = PythonExpression(["'", robot_model, "' == 'robot_xl'"])
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value=default_mecanum_value,
        description="Whether to use mecanum drive controller, otherwise use diff drive",
        choices=["True", "False"],
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["robot", "robot_xl"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all launched nodes.",
    )

    declare_robot_description_arg = DeclareLaunchArgument(
        "robot_description",
        default_value="",
        description="Robot description XML passed to ros2_control_node.",
    )

    declare_include_nerf_arg = DeclareLaunchArgument(
        "include_nerf_launcher",
        default_value="False",
        description="Whether to include the Nerf launcher component in the URDF",
        choices=["True", "False"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config = ReplaceString(controller_config, {"<namespace>/": ns})

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "configuration": configuration,
            "controller_config": ns_controller_config,
            "manipulator_serial_port": manipulator_serial_port,
            "mock_joints": "False",
            "robot_model": robot_model,
            "use_sim": use_sim,
            "include_nerf_launcher": include_nerf_launcher,
        }.items(),
    )

    # SYNC_READ_FAIL occurs while controller_manager uses /robot_description topic
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            ns_controller_config,
        ],
        remappings=[
            ("drive_controller/cmd_vel_unstamped", "cmd_vel"),
            ("drive_controller/odom", "odometry/wheels"),
            ("drive_controller/transition_event", "_drive_controller/transition_event"),
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),
            (
                "joint_state_broadcaster/transition_event",
                "_joint_state_broadcaster/transition_event",
            ),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
        ],
        condition=UnlessCondition(use_sim),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drive_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    nerf_servo_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nerf_servo_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        condition=IfCondition(include_nerf_launcher),
    )

    nerf_flywheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nerf_flywheel_controller",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        condition=IfCondition(include_nerf_launcher),
    )

    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_controller"), "launch", "manipulator.launch.py"]
            )
        ),
        condition=IfCondition(manipulator),
    )

    controllers = [
        joint_state_broadcaster,
        imu_broadcaster,
        drive_controller,
        nerf_servo_controller,
        nerf_flywheel_controller,
    ]

    # spawners expect ros2_control_node to be running
    # Allow Gazebo to finish inserting the model and starting gz_ros2_control
    delayed_controllers = TimerAction(period=6.0, actions=controllers)

    # Delay start of manipulator
    delayed_manipulator_launch = TimerAction(period=8.0, actions=[manipulator_launch])

    def check_if_log_is_fatal(event):
        red_color = "\033[91m"
        reset_color = "\033[0m"
        msg = event.text.decode().lower()
        if ("fatal" in msg or "failed" in msg) and "attempt" not in msg:
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    controllers_monitor = [
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawner,
                on_stderr=check_if_log_is_fatal,
            )
        )
        for spawner in controllers
    ]

    controllers_monitor = GroupAction(controllers_monitor)

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_manipulator_serial_port_arg,
            declare_robot_model_arg,
            declare_namespace_arg,
            declare_mecanum_arg,  # mecanum base on robot_model arg
            declare_robot_description_arg,
            declare_include_nerf_arg,
            declare_controller_config_arg,  # controler_config base on mecanum and robot_model arg
            load_urdf,
            control_node,
            delayed_controllers,
            delayed_manipulator_launch,
            controllers_monitor,
        ]
    )
