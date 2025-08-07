mc_sdk_node-main 远程端
mc_sdk_ros-main 本地端（地面站）


/highlevel_cmd

mc_sdk_msg/msg/HighLevelCmd
int32 control_mode        # 控制模式，例如 MOVE、STANDUP、LIE_DOWN 等
int32 motion_mode         # 动作模式，例如 JUMP、BACKFLIP 等
geometry_msgs/Vector3 cmd_vel     # 平移动速度指令 (x, y, z)
geometry_msgs/Vector3 cmd_angular # 角速度/姿态控制指令 (roll, pitch, yaw)


该两个文件实现了xg同局域网下多机指控，底层通信走dds，同步性能好
相关文档参考
https://www.yuque.com/u32457495/avy8gy/yu3si0zm8fkrwb5u/edit?toc_node_uuid=EdM7OKgDDF1yBHo7
