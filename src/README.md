# 机器人运动规划系统

## 系统环境

- **宿主机系统**: Ubuntu 22.04
- **容器**: Docker
- **ROS 版本**: ROS Noetic

## 启动步骤

### 1. 宿主机配置

在宿主机终端执行以下命令，允许Docker容器访问X服务器：

```bash
xhost +
```

### 2. 启动Docker容器

```bash
docker start ros_noetic
```

### 3. 进入容器

```bash
docker exec -it ros_noetic bash
```

### 4. 容器内配置

进入容器后，设置显示环境变量：

```bash
export DISPLAY=:1
```

### 5. 启动MoveIt

启动MoveIt运动规划框架：

```bash
roslaunch haiheng_moveit_config demo.launch use_rviz=true
```

### 6. 启动通信桥脚本

在新的容器终端中（重复步骤3和4后），启动HTTP通信桥：

```bash
rosrun controller_driver http_to_plan.py
```

## 注意事项

- 确保Docker容器已正确配置ROS Noetic环境
- 启动MoveIt和通信桥脚本需要在不同的终端中执行
- 每次进入容器后都需要设置`DISPLAY`环境变量
