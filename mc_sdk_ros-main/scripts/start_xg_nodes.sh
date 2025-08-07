#!/bin/bash

# SSH 登录信息
USER="firefly"
PASSWORD="firefly" # 请务必将此处替换为您的实际密码
COMMAND="cd ~/mc_sdk_node-main; source install/setup.bash; ros2 launch mc_sdk_node mc_sdk_node.launch.py"

# IP地址范围
START_IP=101
END_IP=102

# 检查所需工具是否安装
if ! command -v sshpass &> /dev/null; then
    echo "sshpass 未安装，请运行: sudo apt install sshpass"
    exit 1
fi

if ! command -v xdotool &> /dev/null; then
    echo "xdotool 未安装，请运行: sudo apt install xdotool"
    exit 1
fi

# 循环遍历IP地址范围，从小到大递增
for ((i=START_IP; i<=END_IP; i++)); do
  IP="192.168.1.$i"
  TERMINAL_TITLE="SSH to $IP mc_sdk_node"

  # 检查IP是否可达
  if ping -c 1 -W 1 $IP > /dev/null; then
    echo "$IP is reachable."
  else
    echo "$IP is not reachable. Skipping."
    continue
  fi

  # 检查是否已经存在具有该标题的终端窗口
  if xdotool search --name "$TERMINAL_TITLE" > /dev/null; then
    echo "Terminal for $IP already exists. Skipping."
    continue
  fi

  # 在新的终端中运行SSH命令，并设置终端标题
  gnome-terminal --title="$TERMINAL_TITLE" -- bash -c "
    if sshpass -p '$PASSWORD' ssh -o ConnectTimeout=10 -t '$USER'@'$IP' '$COMMAND'; then
      exec bash  # 保持终端打开
    else
      echo 'Connection to $IP failed. Exiting terminal.'
      exec bash  # 保持终端打开，即使连接失败
    fi
  "

  sleep 5 # 等待新终端启动
done