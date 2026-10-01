cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

find_package(ament_cmake REQUIRED)

# 這個套件沒有任何節點程式碼，只放「部署設定」：
# 哪個場地、飛多高、跑多快、用模擬時間還是系統時間。
# 節點本身在 drone_control，場景與模擬地圖在 drone_nav2_apriltag。
install(DIRECTORY maps config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/make_blank_map.py
  scripts/frame_probe.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
