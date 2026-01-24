#!/bin/bash

set -e

echo "🔧 正在禁用 cloud-init..."

# 方法1：创建禁用标记文件（官方推荐）
sudo touch /etc/cloud/cloud-init.disabled

# 方法2（可选）：停止并禁用 cloud-init 相关服务（更彻底）
echo "🛑 停止并禁用 cloud-init 服务..."
sudo systemctl stop cloud-init.service \
                 cloud-init-local.service \
                 cloud-config.service \
                 cloud-final.service 2>/dev/null || true

sudo systemctl disable cloud-init.service \
                    cloud-init-local.service \
                    cloud-config.service \
                    cloud-final.service 2>/dev/null || true

# 清理已生成的 cloud-init 状态（避免残留）
sudo rm -rf /var/lib/cloud/

echo "✅ cloud-init 已成功禁用！"
echo "💡 建议重启系统以使更改完全生效：sudo reboot"

#!/bin/bash
set -e

echo "🗑️ 正在移除 snapd..."

# 1. 停止并禁用服务
sudo systemctl stop snapd.service snapd.socket
sudo systemctl disable snapd.service snapd.socket

# 2. 卸载 snapd 包
sudo apt purge -y snapd gnome-software-plugin-snap 2>/dev/null || true

# 3. 清理残留目录
sudo rm -rf /snap /var/snap /var/lib/snapd /var/cache/snapd /usr/lib/snapd

# 4. 防止未来自动安装（可选）
sudo cat > /etc/apt/preferences.d/nosnap.pref <<EOF
Package: snapd
Pin: release *
Pin-Priority: -1
EOF

echo "✅ snapd 已彻底移除！"
echo "💾 节省内存 + 加快启动速度 🚀"

sudo apt update
sudo apt install -y avahi-daemon        