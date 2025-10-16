# Importiert die Hauptklasse für Launch-Beschreibungen
from launch import LaunchDescription
# Importiert verschiedene Launch-Aktionen für Steuerung und Ereignisbehandlung
from launch.actions import (
    DeclareLaunchArgument,  # Deklariert Launch-Argumente
    EmitEvent,  # Sendet Ereignisse
    GroupAction,  # Gruppiert mehrere Aktionen
    IncludeLaunchDescription,  # Bindet andere Launch-Dateien ein
    RegisterEventHandler,  # Registriert Event-Handler
    TimerAction,  # Verzögert Aktionen zeitlich
)
# Importiert Bedingungen für bedingte Ausführung
from launch.conditions import IfCondition, UnlessCondition
# Importiert Event-Handler für Prozess-I/O
from launch.event_handlers import OnProcessIO
# Importiert Shutdown-Event
from launch.events import Shutdown
# Importiert Quelle für Python-Launch-Dateien
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Importiert verschiedene Substitutionen für dynamische Werte
from launch.substitutions import (
    EnvironmentVariable,  # Liest Umgebungsvariablen
    LaunchConfiguration,  # Liest Launch-Konfigurationswerte
    PathJoinSubstitution,  # Verbindet Pfade
    PythonExpression,  # Wertet Python-Ausdrücke aus
)
# Importiert ROS2-spezifische Launch-Aktionen
from launch_ros.actions import Node
# Importiert Parameter-Beschreibungen für Nodes
from launch_ros.parameter_descriptions import ParameterValue
# Importiert Substitution zum Finden von ROS2-Paketen
from launch_ros.substitutions import FindPackageShare
# Importiert String-Ersetzungsfunktion von Nav2
from nav2_common.launch import ReplaceString

# Importiert Hilfsfunktion zum Finden von Geräteporten
from robot_utils.utils import find_device_port


# Hauptfunktion zur Generierung der Launch-Beschreibung
def generate_launch_description():
    # Liest die Roboter-Konfiguration (basic/telepresence/autonomy/manipulation)
    configuration = LaunchConfiguration("configuration")
    # Liest den Pfad zur Controller-Konfigurationsdatei
    controller_config = LaunchConfiguration("controller_config")
    # Liest den seriellen Port für den Manipulator
    manipulator_serial_port = LaunchConfiguration("manipulator_serial_port")
    # Liest ob Mecanum-Antrieb verwendet wird
    mecanum = LaunchConfiguration("mecanum")
    # Liest den Namespace für alle Nodes
    namespace = LaunchConfiguration("namespace")
    # Liest das Robotermodell (robot_xl)
    robot_model = LaunchConfiguration("robot_model")
    # Liest die Roboterbeschreibung (URDF)
    robot_description = LaunchConfiguration("robot_description")
    # Liest ob Simulation verwendet wird (Standard: False)
    use_sim = LaunchConfiguration("use_sim", default="False")
    # Liest ob Nerf-Launcher eingebunden werden soll (Standard: False)
    include_nerf_launcher = LaunchConfiguration("include_nerf_launcher", default="False")

    # Bestimmt den Controller-Typ basierend auf Mecanum-Flag ('mecanum_drive' oder 'diff_drive')
    base_controller_prefix = PythonExpression(
        ["'mecanum_drive' if ", mecanum, " else 'diff_drive'"]
    )
    # Prüft ob die Konfiguration mit 'manipulation' beginnt
    manipulator = PythonExpression(["'", configuration, "'.startswith('manipulation')"])
    # Setzt Präfix 'manipulator_' wenn Manipulator aktiv ist, sonst leer
    manipulator_prefix = PythonExpression(["'manipulator_' if ", manipulator, " else ''"])
    # Baut den Dateinamen der Controller-Konfiguration zusammen
    controller_config_file = PythonExpression(
        ["'", base_controller_prefix, "' + '_' + '", manipulator_prefix, "' + 'controller.yaml'"]
    )
    # Erstellt den vollständigen Pfad zur Standard-Controller-Konfiguration
    default_controller_config = PathJoinSubstitution(
        [FindPackageShare("robot_controller"), "config", robot_model, controller_config_file]
    )

    # Deklariert das Launch-Argument für die Controller-Konfigurationsdatei
    declare_controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller_config,
        description="Path to controller configuration file.",
    )

    # Deklariert das Launch-Argument für die Roboter-Konfiguration
    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description=(
            "Specify configuration packages. Currently only robot XL has available packages."
        ),
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    # Findet automatisch den seriellen Port des Manipulators (USB-Gerät 0403:6014)
    default_manipulator_serial_port = find_device_port("0403", "6014", "/dev/ttyUSB0")
    # Deklariert das Launch-Argument für den Manipulator-Port
    declare_manipulator_serial_port_arg = DeclareLaunchArgument(
        "manipulator_serial_port",
        default_value=default_manipulator_serial_port,
        description="Port to connect to the manipulator.",
    )

    # Deklariert das Launch-Argument für Mecanum-Antrieb (immer True für robot_xl)
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
        choices=["True", "False"],
    )

    # Deklariert das Launch-Argument für das Robotermodell (liest aus Umgebungsvariable)
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value="robot_xl"),
        description="Specify robot model (only robot_xl supported)",
        choices=["robot_xl"],
    )

    # Deklariert das Launch-Argument für den Namespace (Standard: leer)
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all launched nodes.",
    )

    # Deklariert das Launch-Argument für die Roboterbeschreibung (URDF/XACRO)
    declare_robot_description_arg = DeclareLaunchArgument(
        "robot_description",
        default_value="",
        description="Robot description XML passed to ros2_control_node.",
    )

    # Deklariert das Launch-Argument für Nerf-Launcher-Einbindung
    declare_include_nerf_arg = DeclareLaunchArgument(
        "include_nerf_launcher",
        default_value="False",
        description="Whether to include the Nerf launcher component in the URDF",
        choices=["True", "False"],
    )

    # Erstellt Namespace-String mit Slash wenn nicht leer, sonst leer
    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    # Ersetzt Namespace-Platzhalter in der Controller-Konfiguration
    ns_controller_config = ReplaceString(controller_config, {"<namespace>/": ns})

    # Bindet die URDF-Launch-Datei ein, die die Roboterbeschreibung lädt
    load_urdf = IncludeLaunchDescription(
        # Findet und lädt die load_urdf.launch.py aus dem robot_description Paket
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_description"), "launch", "load_urdf.launch.py"]
            )
        ),
        # Übergibt alle notwendigen Argumente an die URDF-Launch-Datei
        launch_arguments={
            "configuration": configuration,  # Roboter-Konfiguration
            "controller_config": ns_controller_config,  # Controller-Konfiguration mit Namespace
            "manipulator_serial_port": manipulator_serial_port,  # Serieller Port des Manipulators
            "mock_joints": "False",  # Keine Mock-Joints (echte Hardware)
            "robot_model": robot_model,  # Robotermodell
            "use_sim": use_sim,  # Simulationsmodus
            "include_nerf_launcher": include_nerf_launcher,  # Nerf-Launcher einbinden
        }.items(),
    )

    # Startet den ros2_control Node (nur wenn nicht in Simulation)
    # Hinweis: SYNC_READ_FAIL kann auftreten wenn controller_manager /robot_description Topic nutzt
    control_node = Node(
        package="controller_manager",  # Controller-Manager Paket
        executable="ros2_control_node",  # Ausführbare Datei
        parameters=[
            # Übergibt die Roboterbeschreibung als String-Parameter
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            ns_controller_config,  # Controller-Konfiguration
        ],
        # Topic-Remappings für einheitliche Schnittstellen
        # Diese Remappings verbinden die ros2_control Hardware-Interfaces mit den standardisierten ROS2-Topics
        remappings=[
            # Data Flow: mecanum_drive_controller → RobotSystem HW Interface → /cmd_vel → micro-ROS Agent → Pico Firmware
            # Publisher: mecanum_drive_controller (publishes cmd_vel_unstamped)
            # Subscriber: RobotSystem hardware interface (subscribes to /cmd_vel, converts to Twist and publishes to firmware)
            # Purpose: Velocity commands from controller to robot base motors
            ("drive_controller/cmd_vel_unstamped", "cmd_vel"),

            # Data Flow: mecanum_drive_controller → /odometry/wheels
            # Publisher: mecanum_drive_controller (computes odometry from wheel velocities)
            # Subscriber: robot_localization EKF, navigation stack
            # Purpose: Wheel-based odometry estimation for sensor fusion and navigation
            ("drive_controller/odom", "odometry/wheels"),

            # Data Flow: drive_controller lifecycle events → hidden topic
            # Publisher: drive_controller (lifecycle state transitions)
            # Subscriber: controller_manager (monitors controller state)
            # Purpose: Internal controller lifecycle management (hidden with _ prefix)
            ("drive_controller/transition_event", "_drive_controller/transition_event"),

            # Data Flow: Pico Firmware → micro-ROS Agent → /imu/data_raw → RobotImuSensor HW Interface → imu_broadcaster
            # Publisher: Pico firmware (ICM20948 sensor via ImuAgent)
            # Subscriber: RobotImuSensor hardware interface (reads IMU data and exposes to imu_broadcaster)
            # Purpose: Raw IMU data (orientation, angular velocity, linear acceleration) for sensor fusion
            ("imu_sensor_node/imu", "/imu/data_raw"),

            # Data Flow: imu_broadcaster lifecycle events → hidden topic
            # Publisher: imu_broadcaster (lifecycle state transitions)
            # Subscriber: controller_manager (monitors broadcaster state)
            # Purpose: Internal broadcaster lifecycle management (hidden with _ prefix)
            ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),

            # Data Flow: joint_state_broadcaster lifecycle events → hidden topic
            # Publisher: joint_state_broadcaster (lifecycle state transitions)
            # Subscriber: controller_manager (monitors broadcaster state)
            # Purpose: Internal broadcaster lifecycle management (hidden with _ prefix)
            (
                "joint_state_broadcaster/transition_event",
                "_joint_state_broadcaster/transition_event",
            ),

            # Data Flow: Pico Firmware → micro-ROS Agent → /joint_states → RobotSystem HW Interface → joint_state_broadcaster
            # Publisher: Pico firmware (MotorsAgent publishes encoder positions and velocities)
            # Subscriber: RobotSystem hardware interface (reads joint states and exposes to joint_state_broadcaster)
            # Purpose: Real-time wheel encoder feedback for controller and robot state publisher
            # Note: This is the critical connection that enables real hardware mode (not mock mode)
            ("~/motors_response", "/joint_states"),
        ],
        condition=UnlessCondition(use_sim),  # Nur starten wenn NICHT in Simulation
    )

    # Spawnt den Joint-State-Broadcaster (veröffentlicht Gelenkzustände)
    joint_state_broadcaster = Node(
        package="controller_manager",  # Controller-Manager Paket
        executable="spawner",  # Spawner-Tool zum Laden von Controllern
        arguments=[
            "joint_state_broadcaster",  # Name des Controllers
            "-c",  # Controller-Manager Flag
            "controller_manager",  # Controller-Manager Name
            "--controller-manager-timeout",  # Timeout-Flag
            "20",  # Timeout in Sekunden
        ],
    )

    # Spawnt den Antriebs-Controller (Mecanum oder Differential)
    drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drive_controller",  # Name des Antriebs-Controllers
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    # Spawnt den IMU-Broadcaster (veröffentlicht IMU-Daten)
    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",  # Name des IMU-Broadcasters
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
    )

    # Spawnt den Nerf-Servo-Controller (steuert Pan/Tilt-Servos) - nur wenn Nerf-Launcher aktiv
    nerf_servo_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nerf_servo_controller",  # Name des Servo-Controllers
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        condition=IfCondition(include_nerf_launcher),  # Nur wenn Nerf-Launcher eingebunden
    )

    # Spawnt den Nerf-Flywheel-Controller (steuert Schwungräder) - nur wenn Nerf-Launcher aktiv
    nerf_flywheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "nerf_flywheel_controller",  # Name des Flywheel-Controllers
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "20",
        ],
        condition=IfCondition(include_nerf_launcher),  # Nur wenn Nerf-Launcher eingebunden
    )

    # Bindet die Manipulator-Launch-Datei ein - nur wenn Manipulator-Konfiguration aktiv
    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_controller"), "launch", "manipulator.launch.py"]
            )
        ),
        condition=IfCondition(manipulator),  # Nur bei manipulation/manipulation_pro Konfiguration
    )

    # Liste aller Controller die gestartet werden sollen
    controllers = [
        joint_state_broadcaster,  # Joint-State-Broadcaster
        imu_broadcaster,  # IMU-Broadcaster
        drive_controller,  # Antriebs-Controller
        nerf_servo_controller,  # Nerf-Servo-Controller (bedingt)
        nerf_flywheel_controller,  # Nerf-Flywheel-Controller (bedingt)
    ]

    # Verzögert den Start der Controller um 6 Sekunden
    # Spawner erwarten dass ros2_control_node läuft
    # Gibt Gazebo Zeit das Modell einzufügen und gz_ros2_control zu starten
    delayed_controllers = TimerAction(period=6.0, actions=controllers)

    # Verzögert den Start des Manipulators um 8 Sekunden
    delayed_manipulator_launch = TimerAction(period=8.0, actions=[manipulator_launch])

    # Callback-Funktion zur Überwachung von fatalen Fehlern in Controller-Logs
    def check_if_log_is_fatal(event):
        red_color = "\033[91m"  # ANSI-Code für rote Farbe
        reset_color = "\033[0m"  # ANSI-Code zum Zurücksetzen der Farbe
        msg = event.text.decode().lower()  # Dekodiert und konvertiert Log-Nachricht zu Kleinbuchstaben
        # Prüft auf "fatal" oder "failed" (aber nicht "attempt" für Wiederholungsversuche)
        if ("fatal" in msg or "failed" in msg) and "attempt" not in msg:
            # Gibt rote Fehlermeldung aus
            print(f"{red_color}Fatal error: {event.text}. Emitting shutdown...{reset_color}")
            # Sendet Shutdown-Event um System herunterzufahren
            return EmitEvent(event=Shutdown(reason="Spawner failed"))

    # Erstellt Event-Handler für jeden Controller zur Fehlerüberwachung
    controllers_monitor = [
        RegisterEventHandler(
            OnProcessIO(
                target_action=spawner,  # Überwacht diesen Spawner
                on_stderr=check_if_log_is_fatal,  # Ruft Callback bei stderr-Output auf
            )
        )
        for spawner in controllers  # Für jeden Controller in der Liste
    ]

    # Gruppiert alle Controller-Monitore zusammen
    controllers_monitor = GroupAction(controllers_monitor)

    # Gibt die vollständige Launch-Beschreibung zurück mit allen Komponenten
    return LaunchDescription(
        [
            declare_configuration_arg,  # Deklariert Konfigurations-Argument
            declare_manipulator_serial_port_arg,  # Deklariert Manipulator-Port-Argument
            declare_robot_model_arg,  # Deklariert Robotermodell-Argument
            declare_namespace_arg,  # Deklariert Namespace-Argument
            declare_mecanum_arg,  # Deklariert Mecanum-Argument (basiert auf robot_model)
            declare_robot_description_arg,  # Deklariert Roboterbeschreibungs-Argument
            declare_include_nerf_arg,  # Deklariert Nerf-Launcher-Argument
            declare_controller_config_arg,  # Deklariert Controller-Config-Argument (basiert auf mecanum und robot_model)
            load_urdf,  # Lädt URDF/Roboterbeschreibung
            control_node,  # Startet ros2_control Node
            delayed_controllers,  # Startet Controller nach 6 Sekunden
            delayed_manipulator_launch,  # Startet Manipulator nach 8 Sekunden
            controllers_monitor,  # Überwacht Controller auf Fehler
        ]
    )
