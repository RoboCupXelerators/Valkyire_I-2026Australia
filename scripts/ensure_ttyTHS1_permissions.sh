#!/usr/bin/env bash
# 持久化设置 /dev/ttyTHS1 权限：添加 udev 规则并为当前用户临时授予访问权限
# 用法: sudo ./ensure_ttyTHS1_permissions.sh [设备路径] [用户名] [组名]
# 例: sudo ./ensure_ttyTHS1_permissions.sh /dev/ttyTHS1 alice dialout

set -euo pipefail

DEVICE_PATH=${1:-/dev/ttyTHS1}
USERNAME=${2:-${SUDO_USER:-${USER}}}
GROUP=${3:-dialout}

KERNEL_NAME=$(basename "$DEVICE_PATH")
RULE_PATH="/etc/udev/rules.d/99-tty-${KERNEL_NAME}.rules"

if [[ $EUID -ne 0 ]]; then
  echo "此脚本需要 root 权限，正在通过 sudo 重新运行..."
  exec sudo bash "$0" "$@"
fi

echo "目标设备: $DEVICE_PATH"
echo "将 udev 规则写入: $RULE_PATH"
echo "目标组: $GROUP，赋予临时访问的用户: $USERNAME"

if [[ -e "$RULE_PATH" ]]; then
  echo "已有规则文件，先备份为 ${RULE_PATH}.bak"
  cp -a "$RULE_PATH" "${RULE_PATH}.bak"
fi

cat > "$RULE_PATH" <<EOF
SUBSYSTEM==\"tty\", KERNEL==\"${KERNEL_NAME}\", MODE=\"0660\", GROUP=\"${GROUP}\"
EOF

echo "已写入 udev 规则，重载规则并触发设备变更。"
udevadm control --reload-rules
udevadm trigger --action=change || true

# 若设备存在，设置 setfacl 以便当前用户立即可读写（不影响 udev 规则）
if [[ -e "$DEVICE_PATH" ]]; then
  if command -v setfacl >/dev/null 2>&1; then
    echo "为用户 $USERNAME 设置临时 ACL 权限。"
    setfacl -m u:${USERNAME}:rw "$DEVICE_PATH" || true
  else
    echo "系统未安装 setfacl，跳过 ACL 设置。若需要请安装 acl 包。"
  fi
fi

echo "完成：udev 规则已添加。"
echo "注意：如果将用户加入了 '${GROUP}' 组，需要重新登录或运行 'newgrp ${GROUP}' 生效。"
echo "若仍报权限错误，请检查 /dev/${KERNEL_NAME} 的当前权限："
echo "  ls -l /dev/${KERNEL_NAME}"

exit 0
