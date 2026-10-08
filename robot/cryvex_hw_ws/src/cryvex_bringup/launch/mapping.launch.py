"""
Cryvex gercek donanim - CANLI HARITALAMA modu ("Ortami Haritala").

navigation.launch.py'nin YERINE gecer (ikisi ayni anda calismaz - ikisi de
/map + map->odom TF yayinlar, catisir). hardware_bringup.launch.py'nin
(stm32_bridge + ydlidar + ekf + robot_state_publisher) YANINDA calisir.

~/cryvex_ws/src/cryvex_gazebo/launch/mapping.launch.py'nin gercek-donanim
kopyasi - TEK FARKI: pointcloud_to_laserscan YOK. YDLIDAR T-mini Plus zaten
native 2D lidar, dogrudan /scan yayinliyor (sim'deki Unitree L1 3D lidar gibi
pointcloud'dan cevirmeye gerek yok).

ONEMLI: async_slam_toolbox_node bir LIFECYCLE node - baslar baslamaz hicbir
sey yapmaz (subscribe/publish etmez), disaridan "configure" + "activate"
gecisleri cagrilana kadar rclcpp::spin() icinde bos bos bekler (sim'deki
mapping.launch.py'da da bu eksik, orada da fark edilmemis). Asagida bu iki
gecis launch olaylariyla tetikleniyor: launch_ros node'un servisi hazir olana
kadar KENDISI bekler (sabit gecikme yok - acilista DDS kesfi yavas olsa da
calisir), configure bitince (inactive) activate gelir.

odom -> base_footprint TF'ini HER ZAMAN EKF yayinlar (2026-09-26'dan beri):
STM32 bagliyken gercek teker odometrisiyle, bagli degilken stm32_bridge
"robot duruyor" (sifir hiz) odometrisi yayinlar - motorlar STM32 olmadan
zaten donemez. (Eskiden burada SABIT bir odom TF'i vardi; STM32 baglaninca
EKF'le catisacakti.) fake_odom:=true sadece EKF/koprusuz elle hata ayiklama
icindir. LiDAR elde tasinirken teker odometrisi hareketi gormez; slam_toolbox
konumu yine tarama-eslestirmesinden (scan matching) cikarir.

GECICI (devami): slam_toolbox varsayilan olarak "odom" girdisine gore en az
0.5m/0.5rad hareket algilamadan yeni bir tarama ISLEMEZ (minimum_travel_*).
Bizim odom SABIT oldugu icin bu esik ASLA asilmiyor - ilk taramadan sonra
harita tamamen DONUYOR (elinizi lidarin onune koysanız bile hicbir sey
degismez). Asagida bu esikleri 0'a cekiyoruz ki slam_toolbox her yeni
taramayi (map_update_interval'e - 5sn - gore) islesin. STM32/motor
baglanip gercek odom gelince bu override'lari KALDIRIP varsayilan (0.5/0.5)
degerlere donmek daha dogru (yoksa gercek odometriyle de gereksiz sik
islem yapar).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    slam_pkg_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(slam_pkg_dir, 'config', 'mapper_params_online_async.yaml'))

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': use_sim_time,
            # 2026-10-02: motorlar takildi, teker odometrisi LiDAR ile kalibre edildi
            # (donus %97, ileri %96 uyum) -> motorsuz donemin 0/0/0.1 gecici
            # degerleri kaldirildi. Her 20 cm ya da ~11 derecede bir tarama islenir:
            # varsayilan 0.5 m/0.5 rad kafe olcegi icin seyrek, her tarama ise
            # donerken haritayi gereksiz bozuyordu.
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'minimum_time_interval': 0.5,
            # Canli harita (telefon/ekran) 5sn yerine 1sn'de bir guncellensin.
            'map_update_interval': 1.0,
            # 2026-10-02: harita bozulmasinin asil nedeni. Varsayilanlarla tarama
            # eslestirici odometriyi neredeyse hic cezalandirmiyor (0.5 m^2 / 1 rad^2
            # varyans, aci cezasi en fazla %10) ve +-25 cm / +-20 derece pencerede en
            # iyi benzeyen yere atliyor; sandalye/masa ayagi dolu kafede bu yanlis
            # eslesme demek (olculdu: robot duz giderken SLAM pozu 0.05 <-> 0.87 m ve
            # 16 derece ziplıyordu, teker + LiDAR ICP ise %3 icinde uyumluydu).
            # Odometri artik iyi oldugu icin eslestirici ona yakin kalsin:
            'distance_variance_penalty': 0.1,
            'angle_variance_penalty': 0.5,   # 2026-10-05: takilmada odometri yonu kayiyor, lidar daha cok soz sahibi
            'minimum_distance_penalty': 0.4,
            'minimum_angle_penalty': 0.6,
            'correlation_search_space_dimension': 0.5,   # +-25 cm (2026-10-05)
            'coarse_search_angle_offset': 0.35,          # +-20 derece (2026-10-05: takilmalarda harita donuyordu)
            # Yanlis dongu kapatma tum grafi bukup haritayi katlar - daha secici ol.
            'loop_match_minimum_response_coarse': 0.45,
            'loop_match_minimum_response_fine': 0.55,
            # 12 m'ye kadar seyrek uzak isinlar kafe icinde gurultu; 8 m yeter.
            'max_laser_range': 12.0,  # 2026-10-03: A yonundeki duvar 11 m - 8 m'de o yondeki bos zemin haritaya islenmiyordu
        }],
    )

    # Sadece hata ayiklama (bkz. modul docstring'i) - normalde EKF yayinlar.
    fake_odom = LaunchConfiguration('fake_odom')
    declare_fake_odom = DeclareLaunchArgument(
        'fake_odom', default_value='false',
        description='Sadece hata ayiklama: EKF yerine sabit odom->base_footprint.')
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cryvex_temp_odom_tf',
        output='screen',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        condition=IfCondition(fake_odom),
    )

    configure_slam = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox_node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    activate_when_configured = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    # 2026-10-05: OTONOM - haritalama acikken Nav2 (MPPI + Collision Monitor) da acilir
    # (bkz. otonom.launch.py). SLAM ayaga kalksin diye 8 sn sonra. Kapatmak: otonom:=false
    from launch.actions import IncludeLaunchDescription, TimerAction
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    declare_otonom = DeclareLaunchArgument('otonom', default_value='true')
    otonom_nav = TimerAction(period=8.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('cryvex_bringup'), 'launch', 'otonom.launch.py')),
        condition=IfCondition(LaunchConfiguration('otonom')))])

    return LaunchDescription([
        declare_otonom,
        otonom_nav,
        declare_use_sim_time,
        declare_slam_params,
        declare_fake_odom,
        static_odom_tf,
        slam_toolbox_node,
        configure_slam,
        activate_when_configured,
    ])
