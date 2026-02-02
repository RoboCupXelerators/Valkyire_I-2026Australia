#!/bin/bash

# 1. 自动获取当前环境信息
USER_NAME=$(whoami)
PROJECT_DIR=$(pwd)
SCRIPT_PATH="$PROJECT_DIR/src/main.py"  # 请确保路径正确
SERVICE_NAME="ENDFIELD"

# 2. 动态探测 ROS2 安装路径
# 优先从当前环境变量获取，如果没有，则去 /opt/ros 下找最新的版本
if [ -n "$ROS_DISTRO" ]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
else
    # 自动寻找 /opt/ros 下存在的 setup.bash
    ROS_SETUP=$(ls /opt/ros/*/setup.bash 2>/dev/null | tail -n 1)
fi

# 检查是否找到了 ROS 环境
if [ ! -f "$ROS_SETUP" ]; then
    echo "错误: 未找到 ROS2 安装路径，请确保已安装 ROS2。"
    exit 1
fi

echo "检测到 ROS2 环境: $ROS_SETUP"
echo "正在为用户 $USER_NAME 配置服务..."

# 3. 写入 Systemd 服务文件
sudo bash -c "cat > /etc/systemd/system/$SERVICE_NAME.service" <<EOF
[Unit]
Description=ROS2 Autostart Service for $USER_NAME
After=network.target

[Service]
User=$USER_NAME
Group=$USER_NAME
WorkingDirectory=$PROJECT_DIR
# 这里的路径完全是根据当前环境“写死”生成的
ExecStart=/usr/bin/bash -c 'source $PROJECT_DIR/src/ROS2_Nodes/sllidarROS/install/setup.bash; source $ROS_SETUP; source $PROJECT_DIR/venv/bin/activate; python3 $SCRIPT_PATH'
Restart=always
RestartSec=5

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