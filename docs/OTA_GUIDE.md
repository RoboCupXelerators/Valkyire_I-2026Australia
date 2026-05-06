# OTA 模型与驱动管理系统指南

## 概述

OTA 系统用于管理 AI 模型和驱动文件的远程更新，**服务端位于 YOLO_Models 仓库**。

### 文件位置

| 组件 | 位置 | 说明 |
|------|------|------|
| OTA 服务端 | `YOLO_Models/ota/` | Flask HTTP API 服务 |
| OTA 客户端 | `Software/src/utils/ota_client.py` | 机器人端更新工具 |
| 模型文件 | `YOLO_Models/models/` | HEF/PT/ENGINE 文件 |
| 驱动文件 | `YOLO_Models/ota/drivers/` | DTB/DEB 文件 |

### 支持的文件类别

| 类别 | 描述 | 文件格式 |
|------|------|----------|
| `model_hailo` | Hailo AI 模型 | `.hef` |
| `model_jetson` | Jetson YOLO 模型 | `.pt`, `.engine` |
| `driver_dtb` | 设备树文件 | `.dtb` |
| `driver_hailo` | Hailo 驱动包 | `.deb` |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    YOLO_Models 仓库                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ models/     │  │ ota/        │  │ drivers/    │         │
│  │ (HEF/PT)    │  │ (服务端)    │  │ (DTB/DEB)   │         │
│  └─────────────┘  └──────┬──────┘  └─────────────┘         │
│                          │                                   │
│                   ┌──────▼──────┐                           │
│                   │ HTTP API    │                           │
│                   └──────┬──────┘                           │
└──────────────────────────┼───────────────────────────────────┘
                           │ WiFi/网络
                   ┌───────▼───────┐
                   │  机器人端      │
                   │  (Software)   │
                   └───────┬───────┘
                           │
                   ┌───────▼───────┐
                   │  ota_client   │
                   │  下载安装更新   │
                   └───────────────┘
```

---

## 服务端部署 (YOLO_Models 仓库)

### 1. 启动服务

```bash
cd YOLO_Models/ota
pip install flask
python ota_server.py --port 8080
```

### 2. 上传模型/驱动

```bash
# 上传 Hailo 模型
curl -X POST http://SERVER_IP:8080/api/upload \
    -F "file=@ArisuModel_v1.3s.hef" \
    -F "category=model_hailo" \
    -F "version=1.3.0" \
    -F "changelog=提升小球检测准确率"

# 上传 Jetson 模型
curl -X POST http://SERVER_IP:8080/api/upload \
    -F "file=@yolov8nForX5_v2.engine" \
    -F "category=model_jetson" \
    -F "version=2.0.0"

# 上传 Hailo 驱动
curl -X POST http://SERVER_IP:8080/api/upload \
    -F "file=@hailort_4.24.0_arm64.deb" \
    -F "category=driver_hailo" \
    -F "version=4.24.0"

# 上传 DTB
curl -X POST http://SERVER_IP:8080/api/upload \
    -F "file=@tegra234-p3768-0000+p3767-0005-nv-super.dtb" \
    -F "category=driver_dtb" \
    -F "version=1.1.0"
```

### 3. 创建发布包

```bash
# 创建完整发布包
curl -X POST http://SERVER_IP:8080/api/bundle \
    -H "Content-Type: application/json" \
    -d '{"version": "1.5.0"}'

# 创建选择性发布包
curl -X POST http://SERVER_IP:8080/api/bundle \
    -H "Content-Type: application/json" \
    -d '{"version": "1.5.0", "categories": ["model_hailo", "driver_hailo"]}'
```

---

## 客户端使用 (Software 仓库)

### 1. 配置

在 `config.json` 中添加 OTA 配置：

```json
{
  "OTA": {
    "Enabled": true,
    "Server": "http://OTA_SERVER_IP:8080",
    "CheckInterval": 300,
    "AutoInstall": false,
    "Categories": ["model_hailo", "driver_hailo"]
  }
}
```

### 2. 命令行使用

```bash
# 检查更新
python -m utils.ota_client --check

# 指定服务器
python -m utils.ota_client --check --server http://192.0.2.100:8080

# 查看当前版本
python -m utils.ota_client --status

# 安装所有更新
python -m utils.ota_client --install

# 安装特定类别
python -m utils.ota_client --install --category model_hailo

# 自动安装（无需确认）
python -m utils.ota_client --install --auto

# 回滚到备份版本
python -m utils.ota_client --rollback --category model_hailo
```

### 3. 程序化使用

```python
from utils.ota_client import OTAClient

# 创建客户端
client = OTAClient(server="http://192.0.2.100:8080")

# 检查更新
updates = client.check_updates()
for update in updates:
    print(f"{update.category}: {update.current_version} -> {update.available_version}")

# 安装更新
for update in updates:
    if update.category == "model_hailo":
        client.install_update(update)
```

---

## 文件安装位置

更新后的文件会安装到以下位置：

| 类别 | 安装位置 |
|------|----------|
| `model_hailo` | `YOLO_Models/models/` |
| `model_jetson` | `YOLO_Models/models/` |
| `driver_dtb` | `/boot/` |
| `driver_hailo` | 项目根目录，通过 dpkg 安装 |

---

## 常见问题

### Q1: 模型下载后无法加载

**解决方案**: 验证模型文件完整性
```bash
sha256sum YOLO_Models/models/ArisuModel_v1.3s.hef
```

### Q2: Hailo 驱动安装失败

**解决方案**: 检查依赖并手动安装
```bash
sudo apt-get install -f
sudo dpkg -i hailort_4.24.0_arm64.deb
```

### Q3: 如何回滚到旧版本

系统会自动备份原文件为 `.bak`，可以手动恢复：
```bash
mv YOLO_Models/models/ArisuModel_v1.3s.hef.bak YOLO_Models/models/ArisuModel_v1.3s.hef
```

---

## API 参考

详见 `YOLO_Models/ota/README.md`

---

**文档版本**: 2026-05-06