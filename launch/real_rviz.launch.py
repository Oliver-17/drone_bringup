# =============================================================================
#  real_rviz.launch.py — 實機：在筆電上開 RViz 看地圖、點目標
#
#  用法（飛場筆電上）：
#      export ROS_DOMAIN_ID=42
#      ros2 launch drone_bringup real_rviz.launch.py
#
#  然後上方工具列按「2D Goal Pose」，在地圖上點一下 —— 樹莓派上的 bt_navigator
#  會收到 /goal_pose 並開始導航。
#
#  ⚠️ 畫面空白、只看到 "Message Filter dropping message" 的話：
#     樹莓派 4 沒有硬體時鐘，沒連外網時系統時間可能是錯的。TF 蓋的是樹莓派的時間，
#     RViz 用筆電的時間比對，差太多就全部丟掉。先讓兩台時間對齊（例如 chrony）。
#
#  ⚠️ 筆電和樹莓派的 DDS 設定要一致（ROS_DOMAIN_ID、FASTRTPS_DEFAULT_PROFILES_FILE），
#     否則會「看得到 topic 名字但沒有資料」—— 2026-09-16 飛場實際遇過。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("drone_bringup"), "rviz", "real.rviz")
    return LaunchDescription([
        Node(
            package="rviz2", executable="rviz2", name="rviz2_real",
            arguments=["-d", cfg],
            # 實機沒有 /clock，要用系統時間
            parameters=[{"use_sim_time": False}],
            output="log",
        ),
    ])
