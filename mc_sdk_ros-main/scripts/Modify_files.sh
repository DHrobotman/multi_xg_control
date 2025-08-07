#!/bin/bash

# 定义变量
REMOTE_USER="firefly"
REMOTE_PASSWORD="firefly"

LOCAL_PATH_config="/home/jszr/jszr_sdk/mc_sdk_node-main"
REMOTE_PATH_config="/home/firefly/"

# 定义IP地址和对应的ID数组
declare -A IP_ID_MAP=(
    ["192.168.1.101"]=""
    ["192.168.1.102"]=""
)

# 遍历IP地址，并执行文件复制操作
for REMOTE_IP in "${!IP_ID_MAP[@]}"; do

    # 先移除远程目录
    echo "正在移除 $REMOTE_IP 上的旧目录..."
    sshpass -p "$REMOTE_PASSWORD" ssh "$REMOTE_USER@$REMOTE_IP" "rm -rf /home/firefly/mc_sdk_node-main"
    
    if [ $? -ne 0 ]; then
        echo "警告：移除 $REMOTE_IP 上旧目录失败"
    else
        echo "成功：已移除 $REMOTE_IP 上的旧目录"
    fi

    # 尝试复制到远程服务器
    sshpass -p "$REMOTE_PASSWORD" scp -r "$LOCAL_PATH_config" "$REMOTE_USER@$REMOTE_IP:$REMOTE_PATH_config"
       
    if [ $? -ne 0 ]; then
        echo "错误：复制文件到 $REMOTE_IP 失败"
    else
        echo "成功：已复制到 $REMOTE_IP"
        
        # 传输完成后进行编译，确保ROS2环境已设置
        #sshpass -p "$REMOTE_PASSWORD" ssh "$REMOTE_USER@$REMOTE_IP" "source /opt/ros/*/setup.bash && cd /home/firefly/mc_sdk_node-main && colcon build"
        
        #if [ $? -ne 0 ]; then
        #    echo "错误：在 $REMOTE_IP 上编译失败"
        #    echo "提示：请确保远程机器已正确安装ROS2环境"
        #else
        #    echo "成功：已在 $REMOTE_IP 上完成编译"
        #fi
    fi
	
done
