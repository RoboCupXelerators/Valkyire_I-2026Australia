# Vision Dashboard（网页仪表盘）

这个目录是 `src/vision_dashboard.py` 的前端静态页面。

## 功能

- 鸟瞰/雷达图：显示球与底盘目标（单位 cm，X 前进、Y 左）
- 4 路相机画面：以 `img + canvas overlay` 的方式叠加 ball bbox（仅 TI 后端提供 bbox）
- 目标表格：ball 位置、bbox、chassis 列表与置信度

## 启动

在仓库根目录运行：

```bash
python3 src/vision_dashboard.py --host 0.0.0.0 --port 8080
```

浏览器打开：

```
http://127.0.0.1:8080/
```

## API

- `GET /api/state`
- `GET /api/cam/0.jpg` ~ `GET /api/cam/3.jpg`（可选 `?size=720`）

## 备注

- 画面刷新用轮询 jpg 的方式实现（轻量、零依赖）。
- bbox 坐标来自 640x640 推理输入空间，因此叠加时按显示尺寸缩放。
