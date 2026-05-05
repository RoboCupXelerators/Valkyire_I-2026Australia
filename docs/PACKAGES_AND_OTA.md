# Packages 管理与 OTA 更新

## 概述

为了减小 git 仓库体积，大文件（安装包、驱动等）存储在独立的 `xelerators-packages` 仓库中，通过脚本按需拉取。

## 架构

```
Xelerators/                    # 主仓库（代码）
├── packages_config.json       # 包配置
├── packages/                  # 本地包目录（.gitignore）
└── scripts/fetch_packages.py  # 拉取脚本

xelerators-packages/           # 包仓库（大文件）
├── packages/
│   ├── hailort-*.whl
│   ├── hailort-*.deb
│   └── *.dtb
└── packages.json              # 版本信息
```

## 使用方法

### 1. 拉取包

```bash
# 拉取所有需要的包
python scripts/fetch_packages.py

# 检查包状态
python scripts/fetch_packages.py --check

# 强制更新
python scripts/fetch_packages.py --upgrade

# 拉取特定包
python scripts/fetch_packages.py --package hailort-4.23.0-cp312-cp312-linux_aarch64.whl
```

### 2. 配置

编辑 `packages_config.json`：

```json
{
  "packages_repo": "https://github.com/your-org/xelerators-packages.git",
  "packages_branch": "main",
  "packages_dir": "packages",
  "packages": {
    "hailort-4.23.0-cp312-cp312-linux_aarch64.whl": {
      "size": 9932383,
      "sha256": "auto",
      "platform": ["linux_aarch64"],
      "description": "Hailo runtime wheel for Jetson"
    }
  },
  "version": "1.0.0"
}
```

## OTA 更新

### 检查更新

```bash
# 检查是否有新版本
python -m utils.ota_updater --check

# 应用更新
python -m utils.ota_updater --update

# 查看更新历史
python -m utils.ota_updater --history

# 回滚到备份
python -m utils.ota_updater --rollback .ota_backup/backup_20260505_120000
```

### 自动更新

在 `ota_config.json` 中配置：

```json
{
  "repo_url": "https://github.com/your-org/Xelerators.git",
  "branch": "main",
  "check_interval": 3600,
  "auto_update": false,
  "backup_enabled": true
}
```

添加到 crontab：

```bash
# 每小时检查一次
0 * * * * cd /path/to/Software && python -m utils.ota_updater --auto
```

## 创建 packages 仓库

1. 创建新仓库：

```bash
mkdir xelerators-packages
cd xelerators-packages
git init
mkdir packages
```

2. 添加大文件：

```bash
cp /path/to/hailort-*.whl packages/
cp /path/to/hailort-*.deb packages/
cp /path/to/*.dtb packages/
```

3. 创建版本信息：

```bash
python -c "
import json
from pathlib import Path

packages = {}
for f in Path('packages').iterdir():
    if f.is_file():
        packages[f.name] = {
            'size': f.stat().st_size,
            'sha256': __import__('hashlib').sha256(f.read_bytes()).hexdigest()
        }

with open('packages.json', 'w') as f:
    json.dump({'packages': packages, 'version': '1.0.0'}, f, indent=2)
"
```

4. 提交并推送：

```bash
git add .
git commit -m 'Add packages'
git remote add origin https://github.com/your-org/xelerators-packages.git
git push -u origin main
```

## 优势

| 方案 | 仓库大小 | 更新方式 |
|------|----------|----------|
| 直接提交 | 16MB+ | git pull |
| 独立仓库 | ~1MB | 脚本拉取 |
| Git LFS | ~1MB | LFS 拉取 |

## 注意事项

1. **首次部署**：运行 `python scripts/fetch_packages.py` 拉取所需包
2. **更新包**：修改 `packages_config.json` 中的版本号后运行 `--upgrade`
3. **离线环境**：提前下载包到 `packages/` 目录