#!/bin/bash

# 设置 8GB 交换文件的脚本 - 适用于 Raspberry Pi OS (Bookworm)
# 要求：root 权限 或 使用 sudo 运行

set -e  # 遇错退出

SWAP_FILE="/swapfile"
SWAP_SIZE_MB=8192  # 8GB = 8192 MB

echo "🔧 正在检查系统信息..."
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "⚠️  警告：未检测到 Raspberry Pi 设备，但将继续执行。"
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "❌ 请使用 sudo 运行此脚本！"
    exit 1
fi

# 检查是否已有 swap
if swapon --show | grep -q "$SWAP_FILE"; then
    echo "✅ 已存在并启用了 $SWAP_FILE，无需重复操作。"
    exit 0
fi

# 如果 swap 文件已存在但未启用，先关闭
if [ -f "$SWAP_FILE" ]; then
    echo "🔄 发现旧的 $SWAP_FILE，正在关闭并删除..."
    swapoff "$SWAP_FILE" 2>/dev/null || true
    rm -f "$SWAP_FILE"
fi

echo "📝 创建 $SWAP_SIZE_MB MB 交换文件（这可能需要几分钟）..."

# 创建 swap 文件（使用 fallocate 更快，若不支持则 fallback 到 dd）
if command -v fallocate >/dev/null 2>&1; then
    fallocate -l ${SWAP_SIZE_MB}M "$SWAP_FILE"
else
    dd if=/dev/zero of="$SWAP_FILE" bs=1M count=$SWAP_SIZE_MB status=progress
fi

# 设置权限（必须为 600）
chmod 600 "$SWAP_FILE"

echo "⚙️  设置交换文件格式..."
mkswap "$SWAP_FILE"

echo "🚀 启用交换文件..."
swapon "$SWAP_FILE"

# 永久生效：写入 /etc/fstab
if ! grep -q "$SWAP_FILE" /etc/fstab; then
    echo "$SWAP_FILE none swap sw 0 0" >> /etc/fstab
    echo "💾 已将交换文件添加到 /etc/fstab（开机自动启用）"
fi

sudo apt install -y zram-config
echo "ALGO=zstd" | sudo tee -a /etc/default/zramswap
sudo systemctl restart zramswap

# 可选：调整 swappiness（默认 60，建议 Pi 上设为 10~30 减少 SD 卡写入）
CURRENT_SWAPPINESS=$(cat /proc/sys/vm/swappiness)
if [ "$CURRENT_SWAPPINESS" -gt 30 ]; then
    echo 'vm.swappiness=10' > /etc/sysctl.d/99-swap.conf
    sysctl --system >/dev/null
    echo "🎚️  已将 swappiness 从 $CURRENT_SWAPPINESS 降低到 10（减少 swap 使用频率）"
fi

echo "✅ 8GB 交换文件已成功创建并启用！"
echo "📊 当前 swap 状态："
swapon --show
free -h