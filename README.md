# drone_bringup — 實機飛行的接線與操作手冊

室內（OptiTrack、沒有 GPS）單機飛行：**起飛 → 懸停 → 用房間座標給目標 → Nav2 飛過去**。

這個套件自己幾乎沒有程式，只有**實機的參數檔、地圖、launch**。
節點本身都在 `drone_control` 和 `drone_mocap`。

- [整條鏈長什麼樣](#整條鏈長什麼樣)
- [橋接的行為（這次改動的重點）](#橋接的行為這次改動的重點)
- [前置條件](#前置條件)
- [完整啟動指令](#完整啟動指令)
- [給目標點](#給目標點)
- [降落與收工](#降落與收工)
- [排錯](#排錯)
- [已知問題](#已知問題)

---

## 整條鏈長什麼樣

```
[Motive] 動捕 → 剛體位姿
   ↓ ═══ VRPN 協定（埠 3883）═══
[vrpn_mocap]  PoseStamped（ENU）
   ↓
[drone_mocap] mocap_to_px4_node   ENU→NED、補時間戳
   ↓ px4_msgs/VehicleOdometry
[MicroXRCEAgent]
   ↓ ═══ 序列埠 921600 ═══
[PX4 EKF2]  融合成位置估計（cs_ev_pos / cs_ev_yaw / cs_ev_hgt）
   ↓ vehicle_local_position
   ├─→ [drone_control] px4_tf_node          → TF: map → odom → base_link
   │                                            Nav2 全靠這個知道飛機在哪
   └─→ [drone_control] cmd_vel_to_px4_node  ← /MAV1/cmd_vel
                ↑                              → trajectory_setpoint
          [Nav2] controller_server
                ↑ /plan
          [Nav2] planner_server ← /goal_pose（map 座標）
```

**全部跑在樹莓派上，一台。**不要把 Nav2 放在筆電 ——
`cmd_vel_to_px4_node` 以 20 Hz 發 setpoint，走 WiFi 的話網路抖一下就掉出 offboard。
動捕那一段的理由見 [`drone_mocap/README.md`](../drone_mocap/README.md)。

**map 座標 = 動捕座標。**2026-10-02 實飛驗證：`map` 的 x = 動捕 x（東）、y = 動捕 y（北），
軸沒有對調也沒有反向，`odom_origin_in_map: [0, 0, 0]` 正確。

---

## 橋接的行為（這次改動的重點）

`cmd_vel_to_px4_node` 有三個狀態。**室內能不能安全飛，整個取決於它。**

```
            沒收過任何 cmd_vel
          ┌──────────────────┐
          │  完全不發布       │  連 publisher 都不建，不搶控制權
          └────────┬─────────┘
            收到第一個 cmd_vel
                   ↓
    ┌──────────────────────────────┐
    │  速度控制                     │  水平用 cmd_vel 轉成的 NED 速度
    │  OffboardControlMode.velocity │  高度用自己的 P 控制器（kp_z）
    └──────┬────────────────┬──────┘
           │                │
   水平指令 < deadband    水平指令 ≥ deadband
           ↓                ↑
    ┌──────────────────────────────┐
    │  原地定點                     │  三軸全位置設定點
    │  OffboardControlMode.position │  目標 = 進入定點那一瞬間的 x, y + flight_altitude
    └──────┬───────────────────────┘
           │  逾時（cmd_timeout_s）也留在這裡，setpoint 不中斷
           │
      呼叫 ~/release_hold
           ↓
    ┌──────────────────────────────┐
    │  銷毀 publisher，交還控制權    │  count_publishers 歸零，別的節點才接得手
    └──────────────────────────────┘
```

### 為什麼室內一定要用定點，不能靠 PX4 的 failsafe

setpoint 一斷，PX4 必定觸發 failsafe。而室內**沒有全球位置**（`global_position_invalid: true`），
每一條 failsafe 出路都會讓飛機下降：

| `COM_OBL_RC_ACT` | 室內結果 |
|---|---|
| 5 Hold | 需要全球位置 → 降級成 **Land** |
| 0 Position | 爬升率由油門桿決定，而**解鎖要求油門在最低點** → **全速下降** |
| 2 Land / 3 Descend | 本來就是降落 |

**沒有一條可以用。**所以正解不是挑一個好的 failsafe，是**不要讓 failsafe 發生**。

> 2026-10-02 實測舊行為：`arm_and_takeoff.py` 退出 → 橋接銷毀 publisher →
> 飛機進 Position(2) + failsafe → 五秒後自己上鎖落地。

### 為什麼「零速度」不等於「留在原地」

速度控制時水平的位置欄位是 NaN（`sp.position = {NaN, NaN, NaN}`），代表**這一層不控制**。
所以送零速度的意思是「把速度壓到零」，**不是「回到原來的位置」** ——
已經產生的位移沒有任何力會把它討回來。

> 2026-10-02 實測：`arm_and_takeoff.py` 全程送零速度爬升，從起飛點飄了 **1.30 m**
> （方向大致朝機頭正前方）。改成位置控制後降到 **0.25 m**。
>
> ⚠️ **SITL 驗不出這件事** —— SITL 同樣的爬升只飄 4 cm。

### 兩個參數

| 參數 | 預設 | 實機值 | 作用 |
|---|---|---|---|
| `hold_on_timeout` | `false` | **`true`** | 逾時後原地定點，而不是銷毀 publisher |
| `zero_cmd_deadband` | `0.0`（關閉） | **`0.02`** m/s | 水平指令小於這個值就位置鎖住當下位置 |

**為什麼預設都是關閉**：模擬端那一整套（`mission_node`、T3 精準降落）是用
「零速度就是零速度 / 逾時就銷毀 publisher」驗過的，不該被默默改掉。
實機值寫在 [`config/real/px4_bridge.yaml`](config/real/px4_bridge.yaml)，
所以**實機一定要用 launch 啟動，不能用 `ros2 run`** —— 手動 `ros2 run` 不會讀那個檔。

### 交棒服務

定點會一直佔著 `trajectory_setpoint`，而 `precision_land_node.cpp:387-391` 看到
`count_publishers > 1` 就拒絕接手降落。要交棒給別的節點時：

```bash
ros2 service call /MAV1/cmd_vel_to_px4_node/release_hold std_srvs/srv/Trigger
```

旗標是**一次性**的 —— 收到新的 `cmd_vel` 就清掉，不會永久關閉定點保護。

---

## 前置條件

**樹莓派上的工作區**：`~/ws_sensor_combined`（不是 `ros2_ws`）。

**每個終端機都要先 source 這個**：

```bash
source ~/ros2_local.sh
```

內容（如果檔案壞了就用這個重建）：

```bash
cat > ~/ros2_local.sh <<'EOF'
# 本機模式：拿掉跨機單播設定
# 那份 XML 的 initialPeersList 沒有指定埠，Fast DDS 只會探測
# participant ID 0~4 —— 同一台機器上超過 5 個節點就會發現不了。
# Nav2 一口氣開十幾個節點，所以一定會中。
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/ws_sensor_combined/install/setup.bash
EOF
```

> `<<'EOF'` 的單引號很重要，沒有的話 `$` 和反引號會被展開。
> ⚠️ 2026-10-02 踩過：把 launch 的輸出重導向到這個檔名，整個環境檔被寫爛，
> 症狀是一堆 `[INFO]: command not found`。**log 要寫 `~/nav2.log`，不要寫 `~/ros2_local.sh`。**

**PX4 參數**（EKF2 的六個，見 [`drone_mocap/config/ekf2_mocap.md`](../drone_mocap/config/ekf2_mocap.md)）要先設好。
驗證方法在下面終端機 4。

**編譯**（改過程式之後）：

```bash
cd ~/ws_sensor_combined
colcon build --packages-select drone_control drone_bringup drone_mocap
```

編完**一定要開新的終端機**再 source —— 舊終端機的 `AMENT_PREFIX_PATH` 還指著舊的 install 目錄。
`drone_control` 的腳本是用 `install(PROGRAMS ...)` **複製**進 install 的，不是 symlink，
所以 `git pull` 之後不重新 build，`ros2 run` 執行到的永遠是舊副本。

---

## 完整啟動指令

**每個終端機第一行都是 `source ~/ros2_local.sh`。**
**順序很重要，不要跳。遙控器全程握在手上。**

### 終端機 1 — uXRCE agent

```bash
source ~/ros2_local.sh
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 終端機 2 — 動捕

```bash
source ~/ros2_local.sh
ros2 launch drone_mocap mocap.launch.py \
  namespace:=MAV1 rigid_body:=MAV1 vrpn_server:=192.168.1.2
```

✅ 要看到節點每 5 秒回報收到／送出的頻率，兩邊都接近 **100 Hz**。

### 終端機 3 — 飛前檢查（EKF2 真的在吃動捕嗎）

```bash
source ~/ros2_local.sh
ros2 topic echo /MAV1/fmu/out/estimator_status_flags --once \
  | grep -E "cs_ev_pos|cs_ev_yaw|cs_ev_hgt|cs_gps|cs_baro_hgt"
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once \
  | grep -E "^x:|^y:|^z:|xy_valid|z_valid"
```

✅ `cs_ev_pos` / `cs_ev_yaw` / `cs_ev_hgt` 全 **true**，`cs_gps_hgt` / `cs_baro_hgt` 全 **false**，
`xy_valid` / `z_valid` 都 true。

**`cs_ev_*` 是 false 就停在這裡，不要起飛** —— 那表示 PX4 在用純慣性推算，起飛一定會飄走。

> ⚠️ `estimator_status_flags` **沒有 `_v1` 後綴**，而 `vehicle_local_position` / `vehicle_status` **有**。
> PX4 v1.17 的版本後綴是逐訊息加的。名字不對的話錯誤訊息是
> `Could not determine the type for the passed topic`，用 `ros2 topic list | grep -i estimator` 確認。

檢查完之後，把這個終端機改成狀態監看：

```bash
while true; do
  N=$(ros2 topic echo /MAV1/fmu/out/vehicle_status_v1 --once 2>/dev/null \
        | grep -E "^nav_state:" | awk '{print $2}')
  P=$(ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once 2>/dev/null \
        | grep -E "^x:|^y:|^z:" | awk '{printf "%+.2f ", $2}')
  echo "$(date +%T)  nav=$N  北東下=[ $P]"
  sleep 1
done
```

`nav_state` 常見值：**14** = OFFBOARD、**2** = POSCTL、**15** = STABILIZED、**18/19** = AUTO_LAND。

### 終端機 4 — 橋接（只要懸停，不給 Nav2 目標時用這個）

```bash
source ~/ros2_local.sh
ros2 launch drone_control px4_bridge.launch.py \
  namespace:=MAV1 \
  flight_altitude:=0.5 \
  params_file:=$(ros2 pkg prefix drone_bringup)/share/drone_bringup/config/real/px4_bridge.yaml
```

### 終端機 4（替代）— 橋接 + Nav2（要給目標點時用這個）

```bash
source ~/ros2_local.sh
ros2 daemon stop && ros2 daemon start
ros2 launch drone_bringup real_nav2.launch.py flight_altitude:=0.5 2>&1 | tee ~/nav2.log
```

`ros2 daemon stop/start`：Nav2 會開十幾個節點，daemon 快取舊的發現設定時會看不到它們。

✅ **這三樣都要對，不對就停**：

```
hold_on_timeout : true（逾時後原地定點，setpoint 不斷 —— 室內用這個）
zero_cmd_deadband: 0.020 m/s（指令速度小於這個值就改用位置鎖住原地）
[lifecycle_manager_costmap]: Managed nodes are active          ← 只有 real_nav2 會印
```

順便把這一行記下來，**那是起飛點**：

```
[px4_tf_node] 已開始發布 odom -> base_link（首筆 ENU：東 -1.01 北 +0.07 上 +0.13）
```

> ⚠️ `flight_altitude` **一定要明確傳**。
> `px4_bridge.launch.py` 的預設是 **3.0**（模擬用，室內會撞天花板），
> `real_nav2.launch.py` 的預設是 **0.8**。而 launch 參數會蓋掉 yaml 裡的值。
> **必須和 `arm_and_takeoff.py --altitude` 一致**，不一致的話起飛完成判定永遠不會通過。

### 終端機 5 — 起飛

```bash
source ~/ros2_local.sh
ros2 run drone_control arm_and_takeoff.py --ns MAV1 --altitude 0.5
```

✅ 進度列要有 `誤差 / vz / 穩定 x/6` 三欄，而且 `穩定` 要數到 6：

```
高度  0.43 / 0.50   誤差 0.07   vz -0.03 m/s   穩定 1/6
高度  0.46 / 0.50   誤差 0.04   vz -0.01 m/s   穩定 5/6
✓ 已懸停在 0.46 m
```

**沒有那三欄就是跑到舊版，停下來重新 build。**

✅ 終端機 4 要同時出現（鎖定點應該和「首筆 ENU」幾乎一樣，差幾公分）：

```
水平指令是零 → 位置鎖在（北 +0.08 東 -1.00 高度 0.50 m）
超過 0.50 秒沒收到 cmd_vel → 維持原地定點（…）。要交棒給別的節點請呼叫 ~/release_hold
定點中：目標(北+0.08 東-1.00 高0.50)  實際(北+0.13 東-0.81 高0.49)  偏差 0.19 m
```

✅ 終端機 3 的 `nav_state` 要**維持 14**、`failsafe: false`，飛機停在空中不動。

**腳本退出後飛機不會自己降落。**這是設計，不是 bug。

`--tolerance 0.08` / `--vz-tolerance 0.05` / `--stable-count 6` / `--timeout 90` 可以用 CLI 覆寫。
高度一直收斂不到 8 cm 的話放寬 `--tolerance 0.12`，先不要急著改 EKF2。

### 可選 — 量懸停品質（30 秒，只讀資料不下指令）

```bash
source ~/ros2_local.sh
ros2 run drone_mocap hover_check.py
```

---

## 給目標點

**⚠️ 一定要等起飛完成、飛機在空中穩住之後才給目標。**

地上給目標的話，Nav2 會送 `cmd_vel` 但飛機沒解鎖不會動，
`required_movement_radius: 0.3` / `movement_time_allowance: 15.0` 的進度檢查會在
15 秒後把目標 **ABORT** 掉。之後你起飛就沒有有效目標了 ——
症狀是「起飛懸停正常，但完全沒去目標」。

### 終端機 6

```bash
source ~/ros2_local.sh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

**用 action，不要用 `ros2 topic pub /goal_pose`。**
`topic pub` 是射後不理，看不到結果；action 會持續印 `distance_remaining`，
結束時印 `SUCCEEDED` 或 `ABORTED`。

**不需要 RViz。**`bt_navigator` 不在 namespace 底下，收的是全域的 `/navigate_to_pose`，
直接在樹莓派上給點就好，省掉跨機 DDS 的麻煩。

**座標**：`map` 的 x = 東（動捕 x）、y = 北（動捕 y）。z 被 Nav2 忽略，高度由橋接鎖著。
房間 5×4 m、中心 (0, 0)，所以 **x ∈ [-2.5, 2.5]、y ∈ [-2, 2]**。

✅ 終端機 4 要依序出現：

```
Begin navigating from current location (-0.09, 0.12) to (1.00, 1.00)
收到非零的水平指令，離開定點，回到速度控制
cmd_vel(前+0.17 左+0.23 轉+0.39) -> NED(北+0.10 東+0.27 下-0.00)  高度 0.49/0.50 m
Reached the goal! / Goal succeeded
水平指令是零 → 位置鎖在（北 +0.87 東 +0.88 高度 0.50 m）
```

**精度參考**（2026-10-02 實飛）：

| 目標（map） | 動捕最終位置 | 誤差 |
|---|---|---|
| 東 0.00 北 0.00 | 東 -0.04 北 +0.08 | 0.09 m |
| 東 1.00 北 1.00 | 東 +0.98 北 +0.93 | 0.07 m |

---

## 降落與收工

橋接在定點模式下**不會放手**，所以要降落有兩個方法：

1. **遙控器接手**（建議）—— 切到你熟悉的手動模式自己降。
2. **Ctrl+C 終端機 4** —— setpoint 斷掉，PX4 走 failsafe。
   室內這等於降落（見上面那張表），所以**要先確認飛機下方是淨空的**。

關閉順序：終端機 6 → 5 → 4 → 2 → 1（反著開的順序關）。

確認沒有殘留：

```bash
pgrep -af "MicroXRCEAgent|cmd_vel_to_px4|px4_tf_node|vrpn|nav2|planner_server|controller_server"
```

---

## 排錯

| 現象 | 原因 | 處理 |
|---|---|---|
| 進度列沒有 `誤差 / vz / 穩定` | 跑到舊版腳本 | `colcon build` 之後**開新終端機** |
| `hold_on_timeout` 印 `false` | 用了 `ros2 run` 而不是 launch，或 `params_file` 路徑錯 | 用上面的 launch 指令 |
| `Could not determine the type for the passed topic` | topic 名字的 `_v1` 後綴猜錯 | `ros2 topic list \| grep <關鍵字>` |
| `cs_ev_*` 是 false | EKF2 沒在融合動捕 | 查 EKF2 六個參數、終端機 2 的頻率 |
| 起飛後往一個方向飄走 | `cs_ev_*` 沒全 true，或 `zero_cmd_deadband` 沒生效 | 回終端機 3 檢查 |
| `nav_state` 變 2 / 18 / 19 | setpoint 斷了（`hold_on_timeout` 沒生效） | 檢查橋接啟動 log |
| 起飛懸停正常但完全不去目標 | 在地上就給了目標，已被進度檢查 ABORT | 重新給一次 |
| `Aborting handle` 每秒一次、永不恢復 | 見[已知問題](#已知問題)第一條 | **Ctrl+C 重開 Nav2**，繼續給目標沒用 |
| Nav2 有 plan 但飛機不動 | `cmd_vel` remap 沒生效 | `ros2 topic hz /MAV1/cmd_vel` |
| `node list` 只剩兩三個節點 | `ros2 daemon` 快取舊設定 | `ros2 daemon stop && ros2 daemon start` |
| 一堆 `[INFO]: command not found` | `~/ros2_local.sh` 被 log 寫爛 | `bash -n ~/ros2_local.sh` 確認，用前面的 heredoc 重建 |
| `duplicate package` | 工作區裡有兩份同名套件 | 舊的那份 `touch COLCON_IGNORE` |

---

## 已知問題

### 1. ⚠️ 連續給點第三次，Nav2 會永久卡死

症狀：`[follow_path] [ActionServer] Aborting handle.` 每秒一次，再也不恢復。
飛機靠定點停在原地，沒有危險，但**重開 Nav2 才會好，繼續給目標完全沒用**。

直接原因是 `bt_navigator` 的 `default_server_timeout: 20` ——
**單位是毫秒**（`bt_action_server_impl.hpp:119-120`）。超時的處理是把 goal handle 丟掉
（`bt_action_node.hpp:219-228`），但那個 goal 晚一點真的會到 `controller_server`，
於是 server 手上有孤兒 goal、BT 手上沒 handle，無限循環。

背後原因是樹莓派 CPU 被 DWB 吃光（`20 × 10 × 10 = 2000` 條軌跡 × 6 critics × 20 Hz）
——**788 行 log 裡有 335 行（42%）是 `Behavior Tree tick rate 100.00 was exceeded`**。

修法（還沒做）：`default_server_timeout: 20 → 200`，加上降 DWB 取樣數、
`bt_loop_duration: 10 → 50`、`controller_frequency: 20 → 10`。兩類要一起改。

### 2. 懸停有 10～19 cm 的穩態偏差，方向固定偏東北

位置控制有回授卻留下固定誤差，代表有持續的側向偏差在抵抗它。
航向已經排除（機頭朝 Motive +x 時 PX4 讀 93°，和 `N = y_mocap` 吻合）。
下一個要查的是水平／加速度計校正。改程式救不了。

### 3. 復原行為在實機上真的會動

卡住時 `behavior_server` 會跑 `spin`（轉 90°）和 `backup`（往後退）。
5×4 m 房間裡這不是好事。**不能只刪 `behavior_plugins` 裡的 `backup`** ——
預設行為樹會呼叫它，伺服器沒有這個動作會卡在等待。要拿掉得換自訂 BT xml。

### 4. 高度有 13 cm 的偏移

飛機停在地上時動捕讀到離原點 +0.13 m，那是剛體標記中心離地的高度。
意思是**目標 0.5 m 時飛機實際離地約 0.37 m**。要消掉是在 Motive 裡把剛體原點校到地面。

---

## 相關

- [`drone_mocap/README.md`](../drone_mocap/README.md) — 動捕那一段、EKF2 參數
- [`drone_control/README.md`](../drone_control/README.md) — 節點本身、模擬端流程
- [`config/real/px4_bridge.yaml`](config/real/px4_bridge.yaml) — 橋接的實機參數
- [`config/real/nav2_params.yaml`](config/real/nav2_params.yaml) — Nav2 的實機參數
