#!/usr/bin/env python3
# =============================================================================
#  frame_probe.py —— 不用起飛，量出「OptiTrack 座標」和「PX4 座標」之間到底怎麼轉
#
#  用法（在跑 vrpn / mocap_px4_bridge 的那台電腦上，環境要和那三支腳本一樣）：
#      export ROS_DOMAIN_ID=42
#      export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_unicast.xml
#      python3 ~/ros2_ws/src/drone_bringup/scripts/frame_probe.py \
#          --mocap-topic /vrpn_mocap/MAV1/pose --ns MAV1
#
#      （它是單一 Python 檔，只需要 rclpy + px4_msgs，不用 colcon build 這個套件）
#
#  你要做的（畫面會一步一步提示）：
#      0. 飛機放在起點、靜止              → Enter
#      1. 沿 OptiTrack 的 +x 走約 0.5 m    → Enter   （機頭方向不要變）
#      2. 再沿 OptiTrack 的 +y 走約 0.5 m  → Enter   （機頭方向不要變）
#      3. 原地把機頭逆時針轉約 90°         → Enter   （從上往下看）
#
#  為什麼要做這個：
#      mocap_px4_bridge 是別人寫的，看不到原始碼。但不用看 —— 比對它的「輸入」
#      （OptiTrack pose）和「輸出」（/fmu/in/vehicle_visual_odometry），就知道它怎麼轉。
#
#      最危險的錯誤是「位置轉了、姿態沒跟著轉同樣的角度」：
#      懸停時水平加速度幾乎是零，看不出來；一開始水平移動，位置控制器就往錯的方向推。
#      所以步驟 3 的轉頭是必要的 —— 它檢查姿態有沒有跟位置轉同一個角度。
#
#  它不送任何東西給 PX4，只訂閱，所以隨時可以跑。
# =============================================================================

import argparse
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry

# 每一步至少要移動這麼多，太少的話雜訊比訊號還大，算出來的方向不可信
MIN_MOVE_M = 0.2
MIN_TURN_DEG = 45.0
# 「主要軸」要比另一軸大這麼多倍，才敢判定成乾淨的 ±x / ±y
DOMINANCE = 2.5
# 姿態和位置轉換要一致到這個程度
YAW_TOL_DEG = 15.0
# EKF2 的移動和橋接輸出差太多，代表 EKF2 沒在用動捕
FUSION_TOL_M = 0.1
# 超過這麼久沒收到，就當作那個來源斷了
STALE_S = 1.0


def yaw_from_wxyz(w, x, y, z):
    """四元數的偏航角（繞 z 軸）。ROS 的 PoseStamped 和 PX4 的 q[] 都用得上，
    只是呼叫時要注意順序：ROS 是 (x,y,z,w)、PX4 的 q[] 是 (w,x,y,z)。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class Probe(Node):
    def __init__(self, mocap_topic, ns):
        super().__init__("frame_probe")
        # 全部用 sensor data QoS（BEST_EFFORT）：BEST_EFFORT 的訂閱端可以接
        # RELIABLE 和 BEST_EFFORT 兩種發布端，不管 vrpn / 橋接用哪種都收得到
        self.lock = threading.Lock()
        self.mocap = self.vo = self.lpos = None     # (t, x, y, z, yaw)
        self.create_subscription(PoseStamped, mocap_topic, self._on_mocap,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, f"/{ns}/fmu/in/vehicle_visual_odometry",
                                 self._on_vo, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 self._on_lpos, qos_profile_sensor_data)

    def _on_mocap(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.mocap = (time.monotonic(), p.x, p.y, p.z, yaw_from_wxyz(q.w, q.x, q.y, q.z))

    def _on_vo(self, m):
        # PX4 的 q[] 順序是 w, x, y, z（和 ROS 相反，這是必踩的雷）
        with self.lock:
            self.vo = (time.monotonic(), float(m.position[0]), float(m.position[1]),
                       float(m.position[2]), yaw_from_wxyz(*[float(v) for v in m.q]))

    def _on_lpos(self, m):
        with self.lock:
            self.lpos = (time.monotonic(), float(m.x), float(m.y), float(m.z), float(m.heading))

    def snapshot(self):
        with self.lock:
            return self.mocap, self.vo, self.lpos


def fresh(s):
    return s is not None and time.monotonic() - s[0] < STALE_S


def describe(v):
    """把 2D 向量描述成最接近的軸，例如 (+0.02, -0.49) → '-y'。"""
    ax = abs(v[0]) >= abs(v[1])
    major, minor = (v[0], v[1]) if ax else (v[1], v[0])
    clean = abs(major) > DOMINANCE * abs(minor)
    return ("+" if major > 0 else "-") + ("x" if ax else "y"), clean


def snap(v):
    """把移動方向吸附到最近的單位軸向量。"""
    if abs(v[0]) >= abs(v[1]):
        return (math.copysign(1.0, v[0]), 0.0)
    return (0.0, math.copysign(1.0, v[1]))


def analyze(marks):
    """marks：四次按 Enter 時的快照 [(mocap, vo, lpos), ...]。回傳結果字典。"""
    r = {"ok": True, "problems": [], "notes": []}

    def d(a, b):   # b - a 的水平位移
        return (b[1] - a[1], b[2] - a[2])

    m, v, l = [s[0] for s in marks], [s[1] for s in marks], [s[2] for s in marks]

    # --- 先確認你真的照指示走了（用 OptiTrack 自己的座標判斷）-------------------
    dm1, dm2 = d(m[0], m[1]), d(m[1], m[2])
    for name, dm, want in (("步驟 1", dm1, "+x"), ("步驟 2", dm2, "+y")):
        got, clean = describe(dm)
        if math.hypot(*dm) < MIN_MOVE_M:
            r["problems"].append(f"{name} OptiTrack 只移動了 {math.hypot(*dm):.2f} m，太少，請重做")
        elif got != want or not clean:
            r["problems"].append(f"{name} 應該沿 OptiTrack {want} 走，實際是 {got}"
                                 f"（Δx={dm[0]:+.2f} Δy={dm[1]:+.2f}），請重做")
    dpsi_m = wrap_deg(math.degrees(m[3][4] - m[2][4]))
    if abs(dpsi_m) < MIN_TURN_DEG:
        r["problems"].append(f"步驟 3 OptiTrack 只轉了 {dpsi_m:+.0f}°，請轉約 90°")
    if r["problems"]:
        r["ok"] = False
        return r

    # --- 橋接輸出的位置怎麼對應 ------------------------------------------------
    dv1, dv2 = d(v[0], v[1]), d(v[1], v[2])
    c1, c2 = snap(dv1), snap(dv2)
    # ned_xy = M · mocap_xy，M 的第一欄是「mocap +x 走到 NED 的哪裡」，第二欄同理
    M = [[c1[0], c2[0]], [c1[1], c2[1]]]
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    r["matrix"] = M
    r["det"] = det
    r["map_x"] = describe(dv1)
    r["map_y"] = describe(dv2)
    r["dv"] = (dv1, dv2)
    r["dm"] = (dm1, dm2)
    # 實際角度和吸附後的軸差多少 —— 看動捕和 NED 是不是剛好差 90° 的倍數
    r["offset_deg"] = [wrap_deg(math.degrees(math.atan2(dv[1], dv[0]) - math.atan2(c[1], c[0])))
                       for dv, c in ((dv1, c1), (dv2, c2))]

    if not (r["map_x"][1] and r["map_y"][1]):
        r["problems"].append("橋接輸出的移動方向不是乾淨的單一軸（可能有非 90° 倍數的旋轉），見下方角度")
    if det == 0:
        r["problems"].append("OptiTrack 的 x 和 y 被轉到同一個 PX4 軸 —— 轉換是錯的")
    elif det > 0:
        # OptiTrack z 往上、PX4 z 往下：z 翻轉了，xy 就必須是鏡像（det = -1）才是合法的右手座標系
        r["problems"].append("xy 對應的行列式是 +1，但 z 已經翻轉 —— 合起來變成左手座標系，轉換是錯的")

    # --- 姿態有沒有跟位置轉同一個角度（最重要的檢查）--------------------------------
    dpsi_v = wrap_deg(math.degrees(v[3][4] - v[2][4]))
    r["dpsi"] = (dpsi_m, dpsi_v)
    if det != 0 and abs(wrap_deg(dpsi_v - det * dpsi_m)) > YAW_TOL_DEG:
        r["problems"].append(f"轉頭方向不一致：OptiTrack 轉 {dpsi_m:+.0f}°，橋接輸出轉 {dpsi_v:+.0f}°"
                             f"（依位置的轉換應該是 {det * dpsi_m:+.0f}°）")

    # 絕對朝向：OptiTrack 剛體的 +x（應該就是機頭）經過同一個 M 轉過去，要等於橋接輸出的 heading。
    # 這一條抓的就是「位置轉了、姿態沒轉」—— 轉頭的「變化量」可能對，但整體差了一個固定角度。
    if det != 0:
        diffs = []
        for i in (0, 3):
            fm = (math.cos(m[i][4]), math.sin(m[i][4]))
            fn = (M[0][0] * fm[0] + M[0][1] * fm[1], M[1][0] * fm[0] + M[1][1] * fm[1])
            pred = math.degrees(math.atan2(fn[1], fn[0]))
            diffs.append(wrap_deg(math.degrees(v[i][4]) - pred))
        r["heading_diff"] = diffs
        if max(abs(x) for x in diffs) > YAW_TOL_DEG:
            r["problems"].append(
                f"機頭朝向和位置的轉換差了約 {diffs[0]:+.0f}° —— 這就是「位置轉了、姿態沒轉」，"
                "或 Motive 裡剛體的 +x 不是機頭。懸停可能正常，但一水平移動就會歪")

    # --- EKF2 有沒有在用動捕 ----------------------------------------------------
    if all(fresh_at(s) for s in l):
        dl1, dl2 = d(l[0], l[1]), d(l[1], l[2])
        err = max(math.hypot(dl1[0] - dv1[0], dl1[1] - dv1[1]),
                  math.hypot(dl2[0] - dv2[0], dl2[1] - dv2[1]))
        r["fusion_err"] = err
        if err > FUSION_TOL_M:
            r["problems"].append(f"EKF2 的移動和橋接輸出差了 {err:.2f} m —— EKF2 可能沒在融合動捕"
                                 "（檢查 EKF2_EV_CTRL、cs_ev_pos）")
    else:
        r["notes"].append("沒收到 vehicle_local_position，略過「EKF2 有沒有在融合」這項")

    r["ok"] = not r["problems"]
    return r


def fresh_at(s):
    # 按 Enter 當下拍的快照裡，時間戳是那時的 monotonic；這裡只看有沒有值
    return s is not None


def report(r):
    line = "=" * 64
    print("\n" + line)
    if "matrix" in r:
        (dm1, dm2), (dv1, dv2) = r["dm"], r["dv"]
        print(" 位置對應（OptiTrack → 橋接輸出，也就是 PX4 看到的）")
        print(f"   OptiTrack +x（Δ {dm1[0]:+.2f}, {dm1[1]:+.2f}） → PX4 {r['map_x'][0]}"
              f"（Δ {dv1[0]:+.2f}, {dv1[1]:+.2f}）  偏 {r['offset_deg'][0]:+.0f}°")
        print(f"   OptiTrack +y（Δ {dm2[0]:+.2f}, {dm2[1]:+.2f}） → PX4 {r['map_y'][0]}"
              f"（Δ {dv2[0]:+.2f}, {dv2[1]:+.2f}）  偏 {r['offset_deg'][1]:+.0f}°")
        M = r["matrix"]
        print(f"   矩陣（ned_xy = M · mocap_xy）：[[{M[0][0]:+.0f}, {M[0][1]:+.0f}], "
              f"[{M[1][0]:+.0f}, {M[1][1]:+.0f}]]   行列式 {r['det']:+.0f}")
        if M == [[0, 1], [1, 0]]:
            print("   → 這是標準的 ENU→NED（和 PX4 官方 GZBridge 相同）")
    if "dpsi" in r:
        print(f" 轉頭：OptiTrack {r['dpsi'][0]:+.0f}° → 橋接輸出 {r['dpsi'][1]:+.0f}°")
    if "heading_diff" in r:
        print(f" 機頭朝向與位置轉換的差：起點 {r['heading_diff'][0]:+.0f}°、轉頭後 {r['heading_diff'][1]:+.0f}°")
    if "fusion_err" in r:
        print(f" EKF2 與橋接輸出的差：{r['fusion_err']:.3f} m")
    for n in r["notes"]:
        print(f" 註：{n}")
    print(line)
    if r["ok"]:
        M = r["matrix"]
        print(" ✓ 自洽：位置和姿態轉了同一個角度，EKF2 也跟得上。可以進行飛行測試。")
        print(f"   飛行測試請帶參數： mocap_to_ned:=\"[{M[0][0]:.0f}.0, {M[0][1]:.0f}.0, "
              f"{M[1][0]:.0f}.0, {M[1][1]:.0f}.0]\"")
        tx = describe((M[0][0], M[1][0]))[0]
        ty = describe((M[0][1], M[1][1]))[0]
        print(f"   白話：要往 OptiTrack +x 飛，PX4 要往 {tx}；要往 OptiTrack +y 飛，PX4 要往 {ty}")
    else:
        print(" ✗ 不能飛：")
        for p in r["problems"]:
            print(f"   - {p}")
    print(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mocap-topic", default="/vrpn_mocap/MAV1/pose")
    ap.add_argument("--ns", default="MAV1")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.mocap_topic, args.ns)
    # ROS 在背景收資料，前景等你按 Enter
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    steps = [
        "0. 飛機放在起點、靜止不動",
        "1. 沿 OptiTrack 的 +x 走約 0.5 m（機頭方向不要變）",
        "2. 再沿 OptiTrack 的 +y 走約 0.5 m（機頭方向不要變）",
        "3. 原地把機頭逆時針轉約 90°（從上往下看）",
    ]
    print(f"OptiTrack：{args.mocap_topic}")
    print(f"橋接輸出：/{args.ns}/fmu/in/vehicle_visual_odometry")
    print(f"EKF2   ：/{args.ns}/fmu/out/vehicle_local_position_v1\n")

    marks = []
    try:
        for s in steps:
            while True:
                mo, vo, lp = node.snapshot()
                state = (f"OptiTrack {'✓' if fresh(mo) else '✗'}  橋接 {'✓' if fresh(vo) else '✗'}  "
                         f"EKF2 {'✓' if fresh(lp) else '✗'}")
                input(f"{s}\n   [{state}]  完成後按 Enter：")
                mo, vo, lp = node.snapshot()
                if fresh(mo) and fresh(vo):
                    marks.append((mo, vo, lp if fresh(lp) else None))
                    break
                # 沒資料時算出來的東西是垃圾，寧可不算
                print("   ✗ OptiTrack 或橋接輸出沒有資料（或超過 1 秒沒更新），這一步不算，請確認後再按一次")
    except (KeyboardInterrupt, EOFError):
        print("\n中止")
        rclpy.shutdown()
        return

    r = analyze(marks)
    report(r)

    out = os.path.expanduser(f"~/.ros/frame_probe_{time.strftime('%Y%m%d-%H%M%S')}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump({"marks": marks, "result": {k: v for k, v in r.items()}}, f,
                  ensure_ascii=False, indent=1, default=str)
    print(f"原始資料存在 {out}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
