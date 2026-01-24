#!/bin/bash
set -e  # 遇错退出

echo "🚀 正在检测系统..."
if ! lsb_release -d | grep -q "Ubuntu 24.04"; then
    echo "⚠️  警告：此脚本仅适用于 Ubuntu 24.04 LTS。"
    echo "当前系统：$(lsb_release -d)"
    read -p "是否继续？(y/N): " confirm
    [[ "${confirm,,}" != "y" ]] && exit 1
fi

ARCH=$(dpkg --print-architecture)
if [[ "$ARCH" != "arm64" ]]; then
    echo "⚠️  警告：检测到非 ARM64 架构（$ARCH），但将继续使用 ubuntu-ports。"
fi

echo "💾 正在备份原始 sources.list 到 /etc/apt/sources.list.bak ..."
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak

echo "📝 正在写入清华大学镜像源..."
cat > /tmp/sources.list.tsinghua <<EOF
# Ubuntu 24.04 LTS (Noble Numbat) - 清华大学开源软件镜像站 (ARM64/Raspberry Pi)
# https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu-ports/
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ noble-security main restricted universe multiverse
EOF

sudo mv /tmp/sources.list.tsinghua /etc/apt/sources.list

echo "🧹 清理旧缓存并更新软件源..."
sudo apt clean
sudo apt update

echo ""
echo "✅ 操作完成！APT 源已成功切换为清华大学镜像。"
echo "💡 提示：如需恢复原配置，请运行："
echo "   sudo cp /etc/apt/sources.list.bak /etc/apt/sources.list"

sudo apt-get update
# Or use the following for newer Ubuntu versions if the above fails
sudo apt-get install libgl1 libglx-mesa0

# === 6. 重建虚拟环境 ===
PROJECT_DIR="$HOME/Xelerator-Soccer-OpenSource"
VENV_DIR="$PROJECT_DIR/venv"

if [ -d "$VENV_DIR" ]; then
    echo "🔄 备份并重建虚拟环境..."
    mv "$VENV_DIR" "$VENV_DIR.bak_$(date +%Y%m%d_%H%M%S)"
fi

echo "🔧 安装hailo8环境..."
sudo apt install linux-headers-$(uname -r) linux-modules-extra-$(uname -r) build-essential dkms
sudo dpkg -i hailort-pcie-driver_4.23.0_all.deb
sudo dpkg -i hailort_4.23.0_arm64.deb
sudo apt --fix-broken install

echo "🔧 创建新的虚拟环境..."
sudo apt install python3.12-venv
python3 -m venv "$VENV_DIR" --system-site-packages

# 激活新环境
source "$VENV_DIR/bin/activate"

# 设置 pip 全局和局部使用清华大学 PyPI 镜像源

set -e  # 出现错误时退出

echo "🔧 正在设置 pip 使用清华大学 PyPI 镜像源..."

# 检查是否以 root 权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "⚠️  注意：此脚本建议以 root 权限运行以设置全局 pip 配置。"
    echo "你可以使用 'sudo' 命令重新运行此脚本。"
fi

# 创建或更新全局 pip 配置文件
GLOBAL_PIP_CONF="/etc/pip.conf"

if [ -f "$GLOBAL_PIP_CONF" ]; then
    echo "🔄 已存在全局 pip 配置文件 $GLOBAL_PIP_CONF，正在更新..."
    sudo sed -i '/index-url/d' "$GLOBAL_PIP_CONF"
    sudo sed -i '/trusted-host/d' "$GLOBAL_PIP_CONF"
else
    echo "📝 创建全局 pip 配置文件 $GLOBAL_PIP_CONF..."
    sudo mkdir -p "$(dirname "$GLOBAL_PIP_CONF")"
    sudo touch "$GLOBAL_PIP_CONF"
fi

cat << EOF | sudo tee "$GLOBAL_PIP_CONF"
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
timeout = 120
trusted-host = pypi.tuna.tsinghua.edu.cn
EOF

echo "✅ 已成功设置全局 pip 配置！"

# 创建或更新当前用户的 pip 配置文件（以防万一）
USER_PIP_CONF="$HOME/.pip/pip.conf"

mkdir -p "$HOME/.pip"
if [ -f "$USER_PIP_CONF" ]; then
    echo "🔄 已存在用户级 pip 配置文件 $USER_PIP_CONF，正在更新..."
    sed -i '/index-url/d' "$USER_PIP_CONF"
    sed -i '/trusted-host/d' "$USER_PIP_CONF"
else
    echo "📝 创建用户级 pip 配置文件 $USER_PIP_CONF..."
    touch "$USER_PIP_CONF"
fi

cat << EOF | tee "$USER_PIP_CONF"
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
timeout = 120
trusted-host = pypi.tuna.tsinghua.edu.cn
EOF

echo "✅ 已成功设置用户级 pip 配置！"

echo "🚀 所有设置已完成！pip 现在将默认使用清华大学 PyPI 镜像源。"


# 升级 pip
pip install --upgrade pip

# 尝试恢复依赖
REQ_FILE="$PROJECT_DIR/requirements.txt"
if [ -f "$REQ_FILE" ]; then
    echo "📦 安装依赖 from requirements.txt..."
    pip install -r "$REQ_FILE"
elif [ -f "$VENV_DIR.bak_*/bin/pip" ]; then
    echo "📦 从旧虚拟环境导出依赖..."
    "$VENV_DIR.bak_"*/bin/pip freeze > "$REQ_FILE"
    pip install -r "$REQ_FILE"
else
    echo "⚠️ 未找到 requirements.txt，请手动安装依赖"
fi

# === 7. 验证 ===
echo ""
echo "✅ 安装完成！验证信息："
echo "Python version: $(python --version)"
echo "Python path: $(which python)"
echo "Virtual environment: $VIRTUAL_ENV"
echo ""
echo "💡 请重新打开终端，或运行：source ~/.bashrc"
echo "然后进入项目目录：cd ~/Xelerator-Soccer-OpenSource && source venv/bin/activate"

echo "🔧 安装hailo8环境..."
wget http://fishros.com/install -O fishros && . fishros
