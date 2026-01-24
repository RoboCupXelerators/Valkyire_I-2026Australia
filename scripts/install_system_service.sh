#!/bin/bash

# 1. 自动获取当前执行脚本的用户名和绝对路径
USER_NAME=$(whoami)
PROJECT_DIR=$(pwd)
# 假设你的虚拟环境在项目目录下的 venv 文件夹
PYTHON_PATH="$PROJECT_DIR/venv/bin/python"
SCRIPT_PATH="$PROJECT_DIR/src/main.py"
SERVICE_NAME="Xelerator_Main_Process"

echo "正在为用户 $USER_NAME 配置服务..."

# 2. 使用 'Here Document' 生成服务文件内容
# 我们直接把变量写入文件，实现“写死”用户名的效果
sudo bash -c "cat > /etc/systemd/system/$SERVICE_NAME.service" <<EOF
[Unit]
Description=Python Autostart Service for $USER_NAME
After=network.target

[Service]
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$PROJECT_DIR
ExecStart=$PYTHON_PATH $SCRIPT_PATH
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

# 3. 重新加载并启动
echo "正在启动服务..."
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
sudo systemctl restart $SERVICE_NAME

echo "配置完成！使用 'sudo systemctl status $SERVICE_NAME' 查看状态。"