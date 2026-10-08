"""
Cryvex gercek donanim - alt seviye baslatma.

    ros2 launch cryvex_bringup hardware_bringup.launch.py

Baslatir: robot_state_publisher (urdf/cryvex_real.urdf.xacro -> TF agaci:
base_footprint->base_link->lidar_link/imu_link), YDLIDAR T-mini Plus surucusu,
STM32 seri koprusu (sonar/IMU/e-stop), robot_localization EKF (teker odom +
IMU -> odom->base_footprint TF, /odom'a remap edilir), cafe_ui_server
(index.html'i - goz animasyonlari, sanal joystick, "Ortami Haritala" -
DEGISTIRMEDEN sunan ARA surum, bkz. cafe_ui_server.py: patrol.py/motor
gelene kadar devriye/siparis/mesaj butonlari zararsizca hicbir sey yapmaz,
haritalama+joystick GERCEK calisir).

NAV2/SLAM/patrol.py/tablet_server.py BURADA DEGIL (bkz. navigation.launch.py
/ mapping.launch.py, Asama 4) - bu dosya sadece "gercek sensorleri ROS'a
baglayan" alt katman. use_sim_time HER ZAMAN false (gercek saat).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('cryvex_bringup')

    serial_port = LaunchConfiguration('stm32_port')
    declare_serial_port = DeclareLaunchArgument(
        'stm32_port', default_value='/dev/cryvex_stm32',
        description='STM32 Nucleo-F446RE USART2 (ST-Link VCP) seri portu - '
                     'bkz. udev_rules/99-cryvex-serial.rules (kurulu degilse '
                     '/dev/ttyACM0 gibi ham isimle gec)')

    xacro_file = os.path.join(pkg, 'urdf', 'cryvex_real.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    stm32_bridge = Node(
        package='cryvex_bringup',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        parameters=[{'serial_port': serial_port, 'baud': 115200, 'use_sim_time': False}],
    )

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ydlidar_tmini_plus.yaml'), {'use_sim_time': False}],
        # YDLidar SDK acilista seri hatta kalmis eski baytlara denk gelirse
        # (checksum hatalari -> "YdDataStream: read past end of buffer") COKUYOR
        # ve bir daha kalkmiyordu (2026-09-25, servis yeniden baslatilinca).
        # Cokerse 3sn sonra kendiliginden yeniden baslasin.
        respawn=True,
        respawn_delay=3.0,
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': False}],
        # nav2_params.yaml (bt_navigator/velocity_smoother) hep "/odom" bekliyor -
        # robot_localization'in varsayilan cikis adi olan "odometry/filtered" yerine.
        remappings=[('odometry/filtered', '/odom')],
    )

    # 2026-10-05: LiDAR odometrisi (rf2o). Teker takilip/kayinca teker odometrisinin
    # yonu bozuluyor ve SLAM haritasi donuyordu. EKF donusu buradan, ileri hizi tekerden alir.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom_rf2o', 'publish_tf': False,
                     'base_frame_id': 'base_footprint', 'odom_frame_id': 'odom',
                     'init_pose_from_topic': '', 'freq': 10.0, 'use_sim_time': False}],
        ros_arguments=['--log-level', 'warn'],   # her taramada INFO basip gunlugu dolduruyordu
        respawn=True,
        respawn_delay=3.0,
    )

    # 2026-10-06: ALGILAMA = YOLO11n yapay zeka (NCNN) + lidar kumeleri -> hitbox'lar (eski: kamera_engel.py). Nesneleri tanir, lidarla uzakligini olcer,
    # Nav2 engel haritasina yazar (/kamera_engeller, costmap 'kamera_layer'). Canli ekran :8081
    from launch.actions import ExecuteProcess
    kamera_engel = ExecuteProcess(
        cmd=[os.path.expanduser('~/cryvex_ai/bin/python'), '-u', os.path.expanduser('~/cryvex_araclar/algilama.py'), '--fov', '38', '--yuk', '0.32'],
        cwd=os.path.expanduser('~/cryvex_araclar'), output='screen', respawn=True, respawn_delay=5.0)

    cafe_ui_server = Node(
        package='cryvex_bringup',
        executable='cafe_ui_server',
        name='cafe_ui_server',
        output='screen',
        parameters=[{'http_port': 8080, 'use_sim_time': False}],
    )

    # Devriye beyni (sim'den tasindi). Nav2'yi kendisi baslatmaz - cafe_ui_server
    # kayitli haritayla baslatir; beyin Nav2 gelene kadar bekler, komutlari
    # /patrol_command'dan alir. Cokerse arayuz "idle" yedegine duser; kendini
    # yeniden baslatsin.
    # name= VERILMEZ: surecte 3 dugum var (patrol_command_listener, sonar_reader,
    # basic_navigator); name= hepsini ayni ada cevirip cakistiriyordu.
    patrol = Node(
        package='cryvex_bringup',
        executable='patrol',
        output='screen',
        parameters=[{'use_sim_time': False}],
        respawn=True,
        respawn_delay=3.0,
    )

    return LaunchDescription([
        declare_serial_port,
        robot_state_publisher,
        stm32_bridge,
        ydlidar_node,
        rf2o_node,
        kamera_engel,
        ekf_node,
        cafe_ui_server,
        patrol,
    ])
