# ball_current 工具集

盘球电流 x 球坐标：采集、可视化、持球检测评估、空转校准。

这些工具会自动定位 `~/Software/src`（加进 `sys.path` 并切到该目录），
所以**从任意目录运行都可以**，例如 `~/Software/Tools/ball_current/` 下直接跑。
其中 `test_ballstatus.py` / `eval_ballstatus.py` / `calibrate_dribble.py`（在线模式）
需要能 import `LunaPre`，因此要用带依赖的解释器（robot1 用 `~/Software/venv/bin/python`）。

## 文件

| 文件 | 作用 |
|---|---|
| `ball_current_viz.py` | 采集球坐标/盘球电流/位姿并画折线图 + HTML |
| `analyze_ball_current.py` | 对录制的 CSV 做统计/ROC 分析 |
| `test_ballstatus.py` | 实时打印 `BallStatus`/电流/基线/margin/threshold/loaded |
| `validate_ballstatus.py` | 离线回放：与 vision 弱标签对比 |
| `eval_ballstatus.py` | 分功率回放评估 TPR/FPR/平衡准确率 |
| `calibrate_dribble.py` | 生成/更新空转校准表 `src/ReasonData/data/dribble_calib.json` |

## 常用命令

```bash
cd ~/Software                      # 任意位置都行
PY=~/Software/venv/bin/python      # robot1 用；robot2 可直接 python3

# 采集（会记录 dribble_cmd）
$PY Tools/ball_current/ball_current_viz.py --dribble 0.5 --duration 15

# 实时看持球检测
$PY Tools/ball_current/test_ballstatus.py --dribble 0.5

# 校准（球必须离开滚轮）：在线实测
$PY Tools/ball_current/calibrate_dribble.py --powers 0.2 0.3 0.5 0.8 --seconds 2
# 或离线反推（用已录、球在远处的数据）
$PY Tools/ball_current/calibrate_dribble.py --from-csv

# 评估
$PY Tools/ball_current/eval_ballstatus.py
```

## 校准说明

- 校准表按功率存 `baseline`/`margin`，运行时按 `Peripherals.DribbleStatus` 的
  当前下发功率查表，**开机即用、无需预热**；随后在线用“球远=空转 / 球近=咬合”
  缓慢细化 margin（越跑越准）。换机器或换齿轮箱后重新校准一次即可。
