# 2D / 弧线开环标定（`loco` 卡片）

`loco_target.move` 现在支持 `forward_m`/`left_m`（斜线）和 `turn_deg`（边走边转）。
闭环的控制参数依赖两个开环事实：**组合速度指令实际走成什么方向/多快**，以及
**边forward边走时偏航指令的实际响应**。先用 `loco` 卡片的定时 `move` 标定，再上闭环。

## 安全前提

- 机器人站立、walking 模式，地面平坦防滑；锥桶/人员清空；
- 每次动作后确认停止；随时可用 `loco.stop_move` / `loco_target.stop_move` 中止；
- 建议先用较小幅值（duration=1.5–2 s）。

## 记录

在机器人上被动记录 IMU（不动底盘）：

```bash
docker exec -i embodied-noetix-bumi bash -lc \
  "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42 \
   RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
   FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml; \
   python3 -u - --seconds 12" < collect_motion_observer.py > /tmp/calib.jsonl
```

（`collect_motion_observer.py` 为被动订阅 IMU 的小脚本；也可用回放轨迹里的
`/tmp/bumi-imu-distance/imu-*.json` 替代。）

## 测试 1：斜线（同时 x+y）

用 `loco` 卡片定时 move（归一化指令，duration 秒）：

| 次数 | vx | vy | duration | 记录 |
|---|---|---|---|---|
| D1 | 0.6 | +0.9 | 2.0 | 起点/终点落点、yaw 变化 |
| D2 | 0.6 | -0.9 | 2.0 | 同上 |
| D3 | 0.6 | +0.6 | 2.0 | 小横移分量是否真的动 |
| D4 | 0.6 | 0 | 2.0 | 纯前进对照 |

判读：实际路径方向与指令方向（`atan2(vy, vx)`，注意归一化）的差、实际位移长度、
yaw 漂移。若 D3 几乎不横移，说明约 0.60 的横移死区确实存在，2D 闭环要设最小横移
分量或限制最小角度。

## 测试 2：边走边转（弧线）

| 次数 | vx | vyaw | duration | 记录 |
|---|---|---|---|---|
| A1 | 0.6 | +0.5 | 2.0 | 起点/终点落点、起止 yaw、弧线半径 |
| A2 | 0.6 | -0.5 | 2.0 | 同上 |
| A3 | 0.6 | +0.8 | 2.0 | 更大转角/半径 |

判读：由起止位置/朝向反推实际弧线（半径、总转角、路径长度），与指令对照；
确认 `turn_deg` 的闭环里“目标航向随进度变化”的速度是否够用、是否需要前馈补偿。

## 结果怎么用

- 斜线：得到“组合指令 → 实际方向/速度”的比例，修正 2D 指令的 vx/vy 配比；
- 弧线：得到“vyaw 指令 → 实际转角/半径”的响应，设定 `turn_deg` 闭环的脉冲强度
  和航向调度的跟随速率；
- 数据稳定后，再在真机上验证 `move(forward_m, left_m[, turn_deg])` 的落点和终态朝向，
  把每段误差记进 `IMU_DISTANCE_VALIDATION.md`。
