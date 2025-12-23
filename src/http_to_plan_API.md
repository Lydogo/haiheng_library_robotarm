# HTTP to Plan API 接口文档

## 概述

`http_to_plan.py` 是一个基于Flask的HTTP服务，提供MoveIt运动规划的Web API接口。该服务允许通过HTTP请求控制机器人手臂的运动规划和执行。

## 全局变量

- `_robot`: MoveIt机器人接口对象
- `_scene`: MoveIt场景接口对象
- `_current_joint_state`: 当前关节状态
- `_ik_service`: 逆运动学服务代理
- `_move_groups`: MoveGroupCommander对象缓存
- `_initialized`: 初始化状态标志

## 核心函数

### move_along_orientation_from_pose(position, orientation, distance=0.1)

根据给定的姿态数据，计算末端执行器向远离书架方向运动指定距离后的新位置。

**参数:**
- `position` (dict): 位置字典 {"x": float, "y": float, "z": float}
- `orientation` (dict): 四元数字典 {"x": float, "y": float, "z": float, "w": float}
- `distance` (float): 运动距离，正值表示向远离书架方向运动，默认0.1

**返回值:**
- `dict`: 新位置字典

### get_end_effector_link(group_name)

根据规划组名称获取对应的末端执行器链接名称。

**参数:**
- `group_name` (str): 规划组名称

**返回值:**
- `str`: 末端执行器链接名称
  - "Right_arm" -> "R_WRIST_R_S"
  - "Left_arm" -> "L_WRIST_R_S"

### solve_ik_for_pose(group_name, target_pose, timeout=10,path_constraints=None)

使用逆运动学服务求解目标位姿对应的关节角度。

**参数:**
- `group_name` (str): 规划组名称
- `target_pose` (dict): 目标位姿
- `timeout` (int): 超时时间，默认10秒
- `path_constraints` (Constraints): 路径约束，可选

**返回值:**
- `tuple`: (成功标志, 关节角度列表或错误信息)

### plan_to_joint_target(move_group, joint_values, joint_names)
使用关节空间规划到达目标关节角度

**参数:**
- `move_group` (MoveGroupCommander): MoveIt规划组对象
- `joint_values` (list): 目标关节角度列表
- `joint_names` (list): 关节名称列表

**返回值:**
- `bool`: 规划成功标志

## API接口

### 1. 获取当前位姿

**接口:** `GET /get_current_pose_http`

**描述:** 获取指定规划组末端执行器的当前6维位置数据

**请求参数:**
- `group_name` (query): 动作组名称（必需）

**请求示例:**
```
GET http://localhost:5000/get_current_pose_http?group_name=Right_arm
```

**响应格式:**
```json
{
  "success": true,
  "group_name": "Right_arm",
  "pose": {
    "position": { "x": 0.123, "y": 0.456, "z": 0.789 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

**错误响应:**
- `400`: 缺少group_name参数
- `503`: MoveIt接口未初始化

### 2. 计算退出位置

**接口:** `POST /calculate_out_position`

**描述:** 根据当前位姿计算向远离书架方向运动后的新位置

**请求格式:**
```json
{
  "group_name": "Right_arm",
  "position": { "x": 0.3, "y": 0.2, "z": 0.5 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "distance": 0.1
}
```

**请求参数:**
- `group_name` (str): 动作组名称（可选，默认 `Right_arm`）
- `position` (dict): 初始位姿位置 {x, y, z}（必需）
- `orientation` (dict): 初始位姿四元数 {x, y, z, w}（必需）
- `distance` (float): 沿当前姿态方向偏移距离（可选，默认0.1米）

**响应格式:**
```json
{
  "success": true,
  "group_name": "Right_arm",
  "pose": {
    "position": { "x": 0.35, "y": 0.25, "z": 0.55 },
    "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
  }
}
```

**错误响应:**
- `400`: 无JSON数据或参数非法
- `500`: 内部错误或计算失败

### 3. 运动规划到指定位置

**接口:** `POST /plan_to_position`

**描述:** 接收动作组和末端执行器位置，通过MoveIt Python接口进行运动规划和执行

**请求格式:**
```json
{
    "group_name": "Right_arm",
    "position": {
        "x": 0.3,
        "y": 0.2,
        "z": 0.5
    },
    "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
    },
    "is_straight_constraint": false
}
```

**请求参数:**
- `group_name` (str): 动作组名称（必需）
- `position` (dict): 末端执行器目标位置 {x, y, z}（必需）
- `orientation` (dict): 末端执行器目标姿态 {x, y, z, w}（必需）
- `is_straight_constraint` (bool): 是否启用直线约束，保持当前旋转姿态不变（可选，默认false）

**约束功能说明:**
当 `is_straight_constraint` 设置为 `true` 时，系统会：
1. 获取当前末端执行器的方向
2. 创建方向约束（OrientationConstraint），限制末端执行器在运动过程中保持当前的旋转姿态
3. 约束参数：
   - 容差：xyz轴各0.1弧度
   - 权重：1.0
   - 坐标系：base_link

**响应格式:**
```json
{ "message": "Success" }
```

**错误响应:**
- `400`: 请求数据错误（缺少必需字段或无JSON数据）
- `500`: 规划执行失败（返回 `{ "message": "Failed" }`）

### 4. 腿部电机动作

**接口:** `POST /leg_move`

**描述:** 发送腿部电机目标位置数组，通过 Action 客户端调用服务端执行。

**请求格式:**
```json
{
  "target_positions": [2500, 8500, 7500]
}
```

**请求参数:**
- `target_positions` (int[]): 目标位置数组（长度应与腿部电机数量一致，单位取决于服务端约定）

**成功响应:**
```json
{
  "success": true,
  "message": "Leg move command sent successfully",
  "target_positions": [2500, 8500, 7500]
}
```

**错误响应:**
- `400`: 无JSON数据或缺少 `target_positions`
- `500`: 执行失败或超时（返回 `{ "success": false, "error": "failed" }`）

**说明:**
- 客户端会同步等待动作结果，默认超时 `10s`。服务端需要正确设置 `SUCCEEDED/ABORTED` 状态与结果消息。

## 坐标系说明

### BASE_S坐标系
- **使用场景**: 逆运动学计算、位姿获取
- **描述**: 机器人基座坐标系，用于定义末端执行器的目标位姿

### base_link坐标系  
- **使用场景**: 方向约束
- **描述**: ROS标准基座坐标系，用于定义约束的参考坐标系

## 末端执行器链接映射

- **Right_arm** -> **R_WRIST_R_S**: 右臂末端执行器
- **Left_arm** -> **L_WRIST_R_S**: 左臂末端执行器

## 使用示例

### 启动服务
```bash
rosrun controller_driver http_to_plan.py
```

### 获取当前位姿
```bash
curl "http://localhost:5000/get_current_pose_http?group_name=Right_arm"
```

### 规划到指定位置（无约束）
```bash
curl -X POST http://localhost:5000/plan_to_position \
  -H "Content-Type: application/json" \
  -d '{
    "group_name": "Right_arm",
    "position": {"x": 0.3, "y": 0.2, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "is_straight_constraint": false
  }'
```

### 规划到指定位置（启用直线约束）
```bash
curl -X POST http://localhost:5000/plan_to_position \
  -H "Content-Type: application/json" \
  -d '{
    "group_name": "Right_arm", 
    "position": {"x": 0.3, "y": 0.2, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "is_straight_constraint": true
  }'
```

### 计算退出位置
```bash
curl -X POST http://localhost:5000/calculate_out_position \
  -H "Content-Type: application/json" \
  -d '{
    "group_name": "Right_arm",
    "position": {"x": 0.3, "y": 0.2, "z": 0.5},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "distance": 0.1
  }'
```

### 腿部电机动作
```bash
curl -X POST http://localhost:5000/leg_move \
  -H "Content-Type: application/json" \
  -d '{ "target_positions": [2500, 8500, 7500] }'
```

## 注意事项

1. 服务启动前需要确保ROS环境和MoveIt已正确配置
2. 所有位置坐标使用米为单位
3. 四元数需要归一化
4. 启用直线约束时，系统会自动获取当前姿态并应用方向约束
5. 服务默认运行在localhost:5000端口