"""
Cryvex gercek donanim - OTONOM SURUS modu (Nav2).

hardware_bringup.launch.py'nin (stm32_bridge + ydlidar + ekf +
robot_state_publisher) YANINDA calisir; mapping.launch.py ile AYNI ANDA
calismaz (ikisi de /map + map->odom TF yayinlar, catisir).

~/cryvex_ws/src/cryvex_gazebo/launch/navigation.launch.py'nin gercek-donanim
kopyasi - TEK FARKI: pointcloud_to_laserscan YOK (YDLIDAR native 2D /scan
yayinliyor, sim'deki 3D->2D koprusune gerek yok). Kullanim:

    ros2 launch cryvex_bringup navigation.launch.py map:=/path/to/harita.yaml

Normalde cafe_ui_server.py'nin LaunchManager'i baslatir (harita varsa acilista,
"Ortami Haritala" bitince/vazgecilince), elle calistirmaya gerek yok.

odom -> base_footprint TF'ini HER ZAMAN EKF yayinlar: STM32 bagliyken gercek
teker odometrisiyle, bagli degilken stm32_bridge "robot duruyor" (sifir hiz)
odometrisi yayinlar (motorlar STM32 olmadan zaten donemez). fake_odom:=true
sadece EKF/koprusuz elle hata ayiklama icin (EKF de calisiyorsa CAKISIR).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg_dir = get_package_share_directory('cryvex_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(bringup_pkg_dir, 'maps', 'cafe_map.yaml'),
        description='"Ortami Haritala" ile uretilip map_saver_cli ile kaydedilen harita.')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_pkg_dir, 'config', 'nav2_params.yaml'),
        description='Gercek robota uyarlanmis (sim testinden turetilmis) Nav2 parametreleri.')

    fake_odom = LaunchConfiguration('fake_odom')
    declare_fake_odom = DeclareLaunchArgument(
        'fake_odom', default_value='false',
        description='Sadece hata ayiklama: EKF yerine sabit odom->base_footprint (bkz. docstring).')
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cryvex_temp_odom_tf',
        output='screen',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        condition=IfCondition(fake_odom),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            # Jazzy varsayilani tum Nav2'yi TEK konteyner surece yukler; Pi'de
            # "load_node ... failed to send response (timeout)" ile bazi
            # dugumler (lifecycle_manager_localization dahil) HIC yuklenmedi
            # ve Nav2 yarim kaldi (2026-09-25). Ayri surecler daha saglam.
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_fake_odom,
        static_odom_tf,
        nav2_launch,
    ])
