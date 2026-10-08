"""
Cryvex OTONOM mod: SLAM (mapping.launch.py, cafe_ui "start_mapping" ile acilir) ACIKKEN Nav2'yi haritasiz
(AMCL/map_server YOK) calistirir. Harita sabit degil: slam_toolbox canli /map + map->odom verir.
2026-10-07: islemci yukunu azaltmak icin nav2_bringup/navigation_launch.py yerine SADECE kullanilan parcalar:
  planner_server (NavFn), controller_server (RotationShim + RPP), behavior_server (wait), bt_navigator,
  velocity_smoother, collision_monitor (son guvenlik freni) + lifecycle_manager.
  KAPALI (kullanilmiyor, ~%25 islemci yiyordu): docking_server, route_server, smoother_server, waypoint_follower.
Hiz zinciri: controller/behavior -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel
Kullanim: ros2 launch cryvex_bringup otonom.launch.py   (once /api/start_mapping)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

DUGUMLER = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother', 'collision_monitor']

def generate_launch_description():
    params = os.path.join(os.path.expanduser('~/cryvex_hw_ws/src/cryvex_bringup'), 'config', 'nav2_otonom.yaml')
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ortak = dict(output='screen', parameters=[params, {'use_sim_time': False}], arguments=['--ros-args', '--log-level', 'info'])
    return LaunchDescription([
        Node(package='nav2_controller', executable='controller_server', remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **ortak),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', remappings=tf, **ortak),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **ortak),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', remappings=tf, **ortak),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             remappings=tf + [('cmd_vel', 'cmd_vel_nav')], **ortak),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', remappings=tf, **ortak),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': DUGUMLER}]),
    ])
