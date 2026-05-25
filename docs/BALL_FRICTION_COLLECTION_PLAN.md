# Ball friction collection plan

目标：把真实球在场地上的滚动衰减、停止距离、停止时间和有效摩擦系数采出来，用来校准 Unity 里的 `ball.drag`、`ball.angularDrag`、球/地面物理材质摩擦，以及踢球后的滚动模型。

## 明天要采什么

建议每条记录都用场地坐标，单位统一为 cm 和秒：

```csv
session_id,trial_id,mode,strength,t_s,x_cm,y_cm,visible,robot_x_cm,robot_y_cm,robot_yaw_deg,kick_strength,notes
```

字段说明：

- `session_id`: 一次采集的编号，例如 `20260522_real_ball`.
- `trial_id`: 单次滚动编号，例如 `free_mid_03`.
- `mode`: `free_roll` / `robot_kick` / `side_roll`.
- `strength`: 手推可写 `slow`、`mid`、`fast`；机器人踢球写数值档位。
- `t_s`: 从本 trial 开始的时间秒。
- `x_cm,y_cm`: 球心在场地坐标里的位置。
- `visible`: 1 表示这一帧球可信，0 表示丢球或遮挡。
- `robot_*` 和 `kick_strength`: 只有机器人踢球时必须填，手推可以留空。
- `notes`: 表面情况、异常碰撞、是否碰墙等。

## 采集动作

1. 自由滚动 `free_roll`

在不碰机器人和墙的区域，从同一条线向前滚球。每个速度档做 10 次：

- `slow`: 约 40-80 cm/s，短距离滚动。
- `mid`: 约 80-160 cm/s，常规比赛速度。
- `fast`: 尽量接近踢球后的自由滚动，但不要撞墙。

2. 机器人踢球 `robot_kick`

用外置接口先摆位，再让机器人踢球。每个强度做 10 次：

```powershell
python -m src.utils.XeleratorSceneControl set_robot --robot-id 1 --x -70 --y 0 --yaw 0
python -m src.utils.XeleratorSceneControl set_ball --x -55 --y 0
```

建议至少采 `kick_strength=0.5,0.75,1.0`。机器人踢球 trial 里记录机器人初始 pose，这样可以在模拟器里复现同样的起始条件。

3. 斜向滚动 `side_roll`

从场地中线附近斜向滚，左右各 5 次。这个主要看球的二维速度衰减是否各向同性。

## 采集要求

- 采样频率尽量 30 Hz 以上，最低不要低于 20 Hz。
- 每次 trial 从球刚释放前 0.2 秒开始，到球连续 1 秒基本不动后结束。
- 球碰墙、碰机器人、被遮挡超过 0.3 秒的 trial 标 `notes=bad`，不要用于主校准。
- 每个 trial 开始前确认球不在禁区、墙边、场地接缝或明显起伏处。

## 离线分析

采完后把 CSV 放到：

```text
Records/ball_friction/ball_friction_real_YYYYMMDD_HHMMSS.csv
```

然后运行：

```powershell
.\venv\Scripts\python.exe -m src.tests.analyze_ball_friction `
  --csv Records\ball_friction\ball_friction_real_YYYYMMDD_HHMMSS.csv `
  --out Records\ball_friction\ball_friction_summary_latest.json
```

分析输出包含：

- `initial_speed_cm_s`: 释放后的初始速度估计。
- `stop_time_s`: 球从释放到基本停止的时间。
- `stop_distance_cm`: 轨迹长度。
- `decel_cm_s2`: 速度衰减斜率。
- `mu_eff`: 有效滚动摩擦，按 `abs(decel_m_s2) / 9.81` 估计。

## 和模拟器对齐

校准顺序建议：

1. 先用 `free_roll` 拟合球自由滚动的停止距离和停止时间。
2. 再用 `robot_kick` 拟合踢球初速度和踢后自旋。
3. 最后用追球/运球场景验证球不会瞬停、不会无意义自转、不会被 dribble 吸得过死。

新的 scene-control 接口已经能做模拟器复现：

```powershell
python -m src.utils.XeleratorSceneControl set_ball --x 0 --y 0 --vx 120 --vy 0 --no-reset-velocity
python -m src.utils.XeleratorSceneControl status
```

注意：`set_ball` 直接给速度适合复现实验，不等价于真实踢球；踢球模型还需要用 `robot_kick` 数据单独拟合。
