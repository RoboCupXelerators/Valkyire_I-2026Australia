#!/bin/bash

# 1. 自动获取当前环境信息
USER_NAME=$(whoami)
PROJECT_DIR=$(pwd)
SCRIPT_PATH="${1:?Usage: bash scripts/install_system_service.sh src/Hyperion.py}"
if [[ "$SCRIPT_PATH" != /* ]]; then
    SCRIPT_PATH="$PROJECT_DIR/$SCRIPT_PATH"
fi
SERVICE_NAME="ENDFIELD"

# Main runs directly in the project's Python environment.
if [ ! -x "$PROJECT_DIR/venv/bin/python" ] || [ ! -f "$SCRIPT_PATH" ]; then
    echo "Run this installer from Software after creating venv."
    exit 1
fi

echo "正在为用户 $USER_NAME 配置服务..."

# 3. 写入 Systemd 服务文件
sudo bash -c "cat > /etc/systemd/system/$SERVICE_NAME.service" <<EOF
[Unit]
Description=Xelerators Robot Service for $USER_NAME
After=network.target

[Service]
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$PROJECT_DIR
# 这里的路径完全是根据当前环境“写死”生成的
ExecStart="$PROJECT_DIR/venv/bin/python" "$SCRIPT_PATH"
Restart=always
RestartSec=5

# Make the project sources importable.
Environment=PYTHONPATH=$PROJECT_DIR/src

[Install]
WantedBy=multi-user.target
EOF

# 4. 重新加载并启动
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
sudo systemctl restart $SERVICE_NAME
# sudo systemctl stop $SERVICE_NAME

echo "服务已部署！可通过以下命令查看实时日志："
echo "journalctl -u $SERVICE_NAME -f"
