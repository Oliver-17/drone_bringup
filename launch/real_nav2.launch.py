# =============================================================================
#  real_nav2.launch.py — 實機：PX4 橋接 + Nav2（跑在樹莓派上）
#
#  用法（樹莓派上）：
#      export ROS_DOMAIN_ID=42
#      ros2 launch drone_bringup real_nav2.launch.py
#
#  前置條件（這支不會幫你開）：
#      1. 樹莓派上 MicroXRCEAgent 已連上飛控（看得到 /MAV1/fmu/out/*）
#      2. EKF2 已經在融合動捕：estimator_status_flags 的 cs_ev_pos 是 true、
#         vehicle_local_position 的 xy_valid 是 true
#         ⚠️ 這條不成立的話，TF 的位置是慣性推算漂出去的值，Nav2 會完全錯亂
#
#  開完之後：
#      起飛    ros2 run drone_control arm_and_takeoff.py --ns MAV1 --altitude 0.8
#      點目標  筆電上 ros2 launch drone_bringup real_rviz.launch.py → 2D Goal Pose
#
#  這支只做「接線」：節點和它們的預設行為都在 drone_control，
#  這裡只換成實機的參數檔、地圖、時鐘設定。
#
#  ⚠️ 為什麼要跑在樹莓派上而不是筆電：
#     cmd_vel_to_px4_node 以 20 Hz 發 setpoint，PX4 的 COM_OF_LOSS_T 超過就觸發
#     失效保護。走 WiFi 的話網路抖一下就會掉出 offboard。Nav2 的 controller
#     和它放在同一台，控制迴路整條都不經過 WiFi。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = "drone_bringup"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    ctrl_launch = os.path.join(get_package_share_directory("drone_control"), "launch")

    args = [
        DeclareLaunchArgument("namespace", default_value="MAV1"),
        # ⚠️ 一定要明確傳：px4_bridge.launch.py 的這個參數預設是 3.0（模擬用），
        #    而且它會蓋掉參數檔裡的值。室內 3 m 可能撞天花板或飛出動捕範圍。
        DeclareLaunchArgument("flight_altitude", default_value="0.8",
                              description="要和 arm_and_takeoff.py --altitude 一致"),
        # 留空 = 用參數檔裡的 [0, 0, 0]。R3 實測原點不在房間中央時才需要填。
        DeclareLaunchArgument("odom_origin", default_value="",
                              description="x,y,z；留空用 config/real/px4_bridge.yaml 的值"),
        DeclareLaunchArgument("map",
                              default_value=os.path.join(share, "maps", "lab_5x4.yaml")),
        DeclareLaunchArgument("px4_params",
                              default_value=os.path.join(share, "config", "real",
                                                         "px4_bridge.yaml")),
        DeclareLaunchArgument("nav2_params",
                              default_value=os.path.join(share, "config", "real",
                                                         "nav2_params.yaml")),
    ]

    ns = LaunchConfiguration("namespace")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_launch, "px4_bridge.launch.py")),
        launch_arguments={
            "namespace": ns,
            "flight_altitude": LaunchConfiguration("flight_altitude"),
            "odom_origin": LaunchConfiguration("odom_origin"),
            "params_file": LaunchConfiguration("px4_params"),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_launch, "nav2.launch.py")),
        launch_arguments={
            "namespace": ns,
            # full = planner + controller + bt_navigator：RViz 點目標飛機就會飛
            "level": "full",
            "params_file": LaunchConfiguration("nav2_params"),
            "map": LaunchConfiguration("map"),
            # RViz 在筆電上另外開（real_rviz.launch.py），樹莓派不開視窗
            "rviz": "false",
            "goal_tool": "false",
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(args + [bridge, nav2])
