#!/usr/bin/env python3
# =============================================================================
#  fake_mocap_bridge.py —— 驗證 frame_probe.py 算得對不對用的假資料
#
#  用法：
#      python3 fake_mocap_bridge.py good   # 正確的 ENU→NED（跟 GZBridge 一樣）
#      python3 fake_mocap_bridge.py bug    # 2026-08-31 懷疑的錯誤：位置轉錯、姿態用正確公式 → 不自洽
#      python3 fake_mocap_bridge.py rot    # 位置和姿態「一起」轉了 90° → 雖然不是標準，但自洽
#      python3 fake_mocap_bridge.py nofuse # 橋接正確，但 EKF2 沒在用動捕（2026-09-16 飛場的狀況）
#
#  它照固定劇本假裝「有人拿著飛機」走：
#      0–2 s 靜止 → 2–4 s 沿 +x 走 0.5 m → 6–8 s 沿 +y 走 0.5 m → 10–12 s 逆時針轉 90°
#  frame_probe 在 1.5 / 5 / 9 / 13 秒各按一次 Enter，就會拿到四個標記點。
#
#  ⚠️ 全部發在 TEST namespace，絕對不會送進真的 PX4。
# =============================================================================

import math
import sys
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry

NS = "TEST"
START = (1.0, 2.0, 0.0)     # 故意不在原點：frame_probe 只能看位移，不能依賴絕對值
YAW0 = math.radians(30.0)   # 故意不朝 +x：絕對朝向的檢查要在任意角度都成立


def ramp(t, t0, t1):
    return min(max((t - t0) / (t1 - t0), 0.0), 1.0)


def pose_at(t):
    """劇本：回傳 ENU 的 (x, y, z, yaw)。"""
    x = START[0] + 0.5 * ramp(t, 2, 4)
    y = START[1] + 0.5 * ramp(t, 6, 8)
    yaw = YAW0 + math.radians(90.0) * ramp(t, 10, 12)
    return x, y, START[2], yaw


def bridge(mode, x, y, z, yaw):
    """假的橋接：ENU → (n, e, d, heading)。"""
    if mode in ("good", "nofuse"):   # GZBridge.cpp:614-617 + 對應的姿態轉換
        return y, x, -z, math.pi / 2 - yaw
    if mode == "bug":       # 位置用了機體系的 (x,-y,-z)，姿態卻用正確公式 → 差 90°
        return x, -y, -z, math.pi / 2 - yaw
    if mode == "rot":       # 位置和姿態都用同一個（非標準但合法的）轉換
        return x, -y, -z, -yaw
    raise ValueError(mode)


class Fake(Node):
    def __init__(self, mode):
        super().__init__("fake_mocap_bridge")
        self.mode = mode
        self.t0 = time.monotonic()
        q = qos_profile_sensor_data
        self.p_mocap = self.create_publisher(PoseStamped, f"/vrpn_mocap/{NS}/pose", q)
        self.p_vo = self.create_publisher(VehicleOdometry, f"/{NS}/fmu/in/vehicle_visual_odometry", q)
        self.p_lp = self.create_publisher(VehicleLocalPosition, f"/{NS}/fmu/out/vehicle_local_position_v1", q)
        self.create_timer(0.02, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        x, y, z, yaw = pose_at(t)

        m = PoseStamped()
        m.header.frame_id = "world"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.p_mocap.publish(m)

        n, e, d, h = bridge(self.mode, x, y, z, yaw)
        vo = VehicleOdometry()
        vo.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vo.position = [float(n), float(e), float(d)]
        vo.q = [math.cos(h / 2), 0.0, 0.0, math.sin(h / 2)]   # PX4 順序 w, x, y, z
        self.p_vo.publish(vo)

        lp = VehicleLocalPosition()       # 假裝 EKF2 完美融合
        if self.mode == "nofuse":
            # EKF2 沒吃動捕：位置停在原地不跟著走（實際上會是 IMU 積分出來、慢慢漂的值）
            n, e, d, h = bridge("good", *pose_at(0.0))
        lp.x, lp.y, lp.z, lp.heading = float(n), float(e), float(d), float(h)
        lp.xy_valid = lp.z_valid = True
        self.p_lp.publish(lp)


def main():
    rclpy.init()
    try:
        rclpy.spin(Fake(sys.argv[1] if len(sys.argv) > 1 else "good"))
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass    # 測試腳本用 kill 收掉它，安靜結束就好


if __name__ == "__main__":
    main()
