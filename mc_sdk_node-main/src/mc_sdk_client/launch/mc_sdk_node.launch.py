import socket
from launch import LaunchDescription
from launch_ros.actions import Node

def get_local_ip():
    """获取本机非回路地址的IP"""
    try:
        # 创建一个临时socket连接来获取本机IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            # 并不需要实际连接成功，只是为了获取本地IP
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
        return ip_address
    except Exception:
        # 出错时返回默认IP
        return "192.168.1.100"

def generate_group_prefix(ip):
    """根据IP地址生成group前缀，如192.168.1.101生成/xg_101/"""
    # 分割IP地址并获取最后一段
    last_octet = ip.split('.')[-1]
    # 生成并返回group前缀
    return f"/xg_{last_octet}/"

def generate_launch_description():
    # 获取本机IP
    local_ip = get_local_ip()
    # 生成group前缀
    group_prefix = generate_group_prefix(local_ip)
    
    return LaunchDescription([
        # 启动节点并设置group参数
        Node(
            package='mc_sdk_node',
            executable='mc_sdk',
            name='mc_sdk_node',
            output='screen',
            parameters=[{
                # 这里可以添加其他参数
            }],
            namespace=group_prefix
        )
    ])
