#!/bin/bash

# 1. 权限检查
if [ "$EUID" -ne 0 ]; then 
  echo "请使用 sudo 运行此脚本"
  exit 1
fi

#!/bin/bash

# 获取当前登录的用户名
CURRENT_USER=$(whoami)

# 定义配置文件路径，使用 /etc/sudoers.d/ 是更规范的实践
# 这样可以避免直接修改主 sudoers 文件，降低系统风险
CONFIG_FILE="/etc/sudoers.d/$CURRENT_USER"

echo "正在为用户 [$CURRENT_USER] 配置免密码 sudo 权限..."

# 检查是否已经存在配置文件，避免重复写入
if [ -f "$CONFIG_FILE" ]; then
    echo "配置文件已存在，正在检查内容..."
    if grep -q "NOPASSWD: ALL" "$CONFIG_FILE"; then
        echo "✅ 用户 [$CURRENT_USER] 已经配置了免密码 sudo。"
        exit 0
    fi
fi

# 核心命令：将配置写入文件
# 注意：必须使用 visudo -c -f 来检查语法，或者直接写入后由系统读取
# 这里使用 echo 重定向的方式，简单直接
echo "$CURRENT_USER ALL=(ALL:ALL) NOPASSWD:ALL" | sudo tee "$CONFIG_FILE" > /dev/null

# 设置文件权限为 440 (只读)，这是 sudoers 文件的安全标准
sudo chmod 440 "$CONFIG_FILE"

# 验证配置是否生效（可选，检查语法错误）
if sudo visudo -c -f "$CONFIG_FILE" > /dev/null 2>&1; then
    echo "✅ 配置成功！用户 [$CURRENT_USER] 现在可以免密码执行 sudo。"
    echo "💡 提示：你可以运行 'sudo ls /root' 来测试一下。"
else
    echo "❌ 配置写入完成，但语法检查未通过，请检查文件内容。"
    exit 1
fi

echo "==========================================="
echo "   Ubuntu USB 物理路径持久化工具"
echo "==========================================="

# 2. 彻底屏蔽干扰服务 (BRLTTY)
echo "[1/4] 正在检查并禁用干扰服务..."
systemctl mask brltty.service >/dev/null 2>&1
systemctl stop brltty.service >/dev/null 2>&1

# 3. 引导用户识别路径
echo ""
echo "[2/4] 识别步骤："
read -p "请输入你想要设置的别名 (例如: sllidar): " ALIAS_NAME
echo "请确认设备已插入，且当前对应的临时端口 (例如: ttyUSB0): "
read -p "/dev/" DEV_NODE

DEV_PATH="/dev/$DEV_NODE"

if [ ! -e "$DEV_PATH" ]; then
    echo "❌ 错误：找不到设备 $DEV_PATH，请检查输入或连接。"
    exit 1
fi

# 4. 提取精确的 ID_PATH 和硬件 ID
echo "[3/4] 正在提取物理坐标..."
ID_PATH=$(udevadm info --query=property --name="$DEV_PATH" | grep "ID_PATH=" | cut -d'=' -f2)
ID_V=$(udevadm info -a -n "$DEV_PATH" | grep '{idVendor}' | head -n1 | cut -d'"' -f2)
ID_P=$(udevadm info -a -n "$DEV_PATH" | grep '{idProduct}' | head -n1 | cut -d'"' -f2)

if [ -z "$ID_PATH" ] || [ -z "$ID_V" ]; then
    echo "❌ 错误：提取硬件属性失败。"
    exit 1
fi

echo "     - 物理路径: $ID_PATH"
echo "     - 硬件特征: $ID_V:$ID_P"

# 5. 生成规则文件
RULE_FILE="/etc/udev/rules.d/99-$ALIAS_NAME.rules"
echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$ID_PATH\", ATTRS{idVendor}==\"$ID_V\", ATTRS{idProduct}==\"$ID_P\", SYMLINK+=\"$ALIAS_NAME\", MODE=\"0666\"" > "$RULE_FILE"

# 6. 重新加载
echo "[4/4] 正在注入规则并刷新系统..."
udevadm control --reload-rules
udevadm trigger --action=add

echo "==========================================="
if [ -L "/dev/$ALIAS_NAME" ]; then
    echo "✅ 成功！"
    echo "设备已永久绑定: /dev/$ALIAS_NAME -> $(readlink -f /dev/$ALIAS_NAME)"
    echo "规则保存位置: $RULE_FILE"
else
    echo "⚠️ 规则已写入，但别名未出现。"
    echo "建议：拔掉设备重新插入，或执行 'sudo udevadm trigger'"
fi
echo "==========================================="