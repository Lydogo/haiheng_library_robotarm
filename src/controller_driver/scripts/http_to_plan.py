#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np
import tf.transformations as tf_trans
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import moveit_msgs.msg
import rospy
import actionlib
import ros_numpy
from flask import Flask, request, jsonify
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState, Constraints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import OrientationConstraint
from controller_driver.msg import *
from moveit_msgs.msg import PositionConstraint, BoundingVolume, Constraints
from geometry_msgs.msg import Pose, Point,PoseStamped
from shape_msgs.msg import SolidPrimitive
from tf.transformations import unit_vector, quaternion_from_matrix

app = Flask(__name__)

# 全局变量
_robot = None
_scene = None
_current_joint_state = None
_ik_service = None
_move_groups = {}
_initialized = False
_legmove_client = None
_TF_BUFFER = None
_legs_moving = False
_arms_moving = False
import threading
_legs_lock = threading.Lock()
_arms_lock = threading.Lock()
# K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
_CAMERA_K = np.array([
    [1368.677978515625, 0.0, 969.1411743164062],
    [0.0, 1368.361083984375, 558.4945068359375],
    [0.0, 0.0, 1.0]
])

"""
初始化相关函数
"""


def initialize_moveit():
    global _robot, _scene, _initialized, _legmove_client
    if _initialized:
        return True

    try:
        rospy.sleep(10)
        # 初始化moveit_commander和rospy节点
        moveit_commander.roscpp_initialize(sys.argv)
        # 实例化RobotCommander对象
        _robot = moveit_commander.RobotCommander()
        # 实例化PlanningSceneInterface对象
        _scene = moveit_commander.PlanningSceneInterface()
        # 初始化腿部动作客户端
        _legmove_client = actionlib.SimpleActionClient("/legs_controller/leg_move", LegMoveAction)
        # 订阅关节状态话题
        rospy.Subscriber('/joint_states', JointState, joint_state_callback)
        rospy.sleep(0.5)
        _initialized = True
        print("MoveIt initialized successfully!")
        return True

    except Exception as e:
        print(f"Failed to initialize: {str(e)}")
        return False


def get_move_group(group_name):
    """获取或创建MoveGroupCommander对象，使用缓存避免重复创建"""
    global _move_groups
    if group_name not in _move_groups:
        try:
            move_group = moveit_commander.MoveGroupCommander(group_name)
            # 设置工作空间
            move_group.set_workspace([-1.0, -1.0, -1.0, 1.0, 1.0, 1.0])
            move_group.set_planning_pipeline_id("ompl")
            move_group.set_planner_id("RRT")
            _move_groups[group_name] = move_group
            print(f"MoveGroupCommander for group '{group_name}' created successfully!")

        except Exception as e:
            print(f"Failed to create MoveGroupCommander for group '{group_name}': {str(e)}")
            return None

    return _move_groups[group_name]


def joint_state_callback(msg):
    """关节状态回调函数"""
    global _current_joint_state
    _current_joint_state = msg


def get_end_effector_link(group_name):
    if group_name == "Right_arm":
        return "R_GRIPPER_END_S"
    elif group_name == "Left_arm":
        return "L_WRIST_R_S"


"""
坐标转换相关函数
"""


# 将像素坐标和深度转换为相机坐标系下的三维坐标。
def pixel_to_camera_xyz(depth, u, v, K=_CAMERA_K):
    if K is None:
        K = _CAMERA_K
    if not isinstance(K, np.ndarray):
        K = np.array(K).reshape(3, 3)

    fx = float(K[0, 0])
    fy = float(K[1, 1])
    cx = float(K[0, 2])
    cy = float(K[1, 2])
    zc = float(depth)
    xc = (float(u) - cx) * zc / fx
    yc = (float(v) - cy) * zc / fy
    return {'x': xc, 'y': yc, 'z': zc}


def caculate_target_pose_from_pixel_pose(pixel_x, pixel_y, depth, theta,
                                         camera_frame,
                                         base_frame,
                                         timeout=1.0):
    """
    输入像素(u,v)、深度d与旋转角度theta(单位: degrees)，
    输出在base坐标系下的位置(position)与姿态(orientation)。
    返回(position_dict, orientation_dict)，失败返回(None, None)。
    """
    global _TF_BUFFER
    if _TF_BUFFER is None:
        _TF_BUFFER = tf2_ros.Buffer()
        tf2_ros.TransformListener(_TF_BUFFER)
    cam_xyz = pixel_to_camera_xyz(depth, pixel_x, pixel_y)
    if _TF_BUFFER is None:
        rospy.logerr("TF buffer not injected!")
        return None, None
    try:
        tf_msg = _TF_BUFFER.lookup_transform(base_frame, camera_frame, rospy.Time(0), rospy.Duration(timeout))
        T_base_cam = ros_numpy.numpify(tf_msg.transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(f"TF lookup failed ({camera_frame} -> {base_frame}): {str(e)}")
        return None, None

    # 转换位置 (点变换：需要乘平移和旋转)
    p_cam_homo = np.array([cam_xyz['x'], cam_xyz['y'], cam_xyz['z'], 1.0])
    p_base_homo = np.dot(T_base_cam, p_cam_homo)
    final_pos = p_base_homo[0:3]
    print(f"Final position in base frame: {final_pos}")

    # 计算相机坐标系下的物体向量：
    theta_rad = np.deg2rad(theta)
    obj_vec_z_cam = np.array([np.sin(theta_rad), -np.cos(theta_rad), 0.0])
    print(f"Object vector in camera frame: {obj_vec_z_cam}")

    # 转换向量 (向量变换：只需要乘旋转部分，忽略平移)
    R_base_cam = T_base_cam[0:3, 0:3]
    v_cam = np.array(obj_vec_z_cam)
    v_base = np.dot(R_base_cam, v_cam)

    # 在 BASE_S 下构建姿态矩阵
    # 1. 确定 Z 轴 (物体朝向)
    z_axis = unit_vector(v_base)
    # 2. 确定 X 轴参考方向 (Base系下的水平向前)，这里的含义是：我们希望抓取时的 X 轴尽量指向 Base 的 X+ 方向
    ref_x_axis = np.array([1, 0, 0])
    # 3. 计算 Y 轴 (利用叉乘: Z x Ref_X)
    y_axis = np.cross(z_axis, ref_x_axis)
    y_axis = unit_vector(y_axis)
    # 4. 重新计算 X 轴 (确保 X, Y, Z 正交)
    x_axis = np.cross(y_axis, z_axis)
    x_axis = unit_vector(x_axis)

    # 生成最终变换矩阵并转为四元数
    T_final = np.eye(4)
    T_final[0:3, 0] = x_axis
    T_final[0:3, 1] = y_axis
    T_final[0:3, 2] = z_axis
    T_final[0:3, 3] = final_pos
    final_quat = quaternion_from_matrix(T_final)

    # 补偿URDF中rpy从(pi, 0, -pi/2)到(0,0,0)的坐标系变化
    # 原始rpy(pi, 0, -pi/2)对应的四元数
    q_orig_rpy = tf_trans.quaternion_from_euler(np.pi, 0, -np.pi/2)
    # 将目标姿态乘以原始rpy的逆，得到新的目标姿态
    final_quat = tf_trans.quaternion_multiply(final_quat, q_orig_rpy)

    target_position = {
        "x": final_pos[0],
        "y": final_pos[1],
        "z": final_pos[2]
    }
    target_orientation = {
        "x": final_quat[0],
        "y": final_quat[1],
        "z": final_quat[2],
        "w": final_quat[3]
    }
    print(f"Successfully calculated target pose!")
    return target_position, target_orientation


def caculate_pre_pose_from_target_pose(position, orientation, distance=0.1):
    target_position = np.array([
        float(position['x']),
        float(position['y']),
        float(position['z'])
    ])
    quat = [
        float(orientation['x']),
        float(orientation['y']),
        float(orientation['z']),
        float(orientation['w'])
    ]
    quat_norm = np.linalg.norm(quat)
    quat = np.array(quat) / quat_norm
    rotation_matrix = tf_trans.quaternion_matrix(quat)
    y_axis = rotation_matrix[:3, 1]
    direction_vector = y_axis / np.linalg.norm(y_axis)
    # 向远离书架方向运动指定距离
    displacement = direction_vector * distance
    pre_position = target_position + displacement

    print(f"Successfully calculated pre pose!")
    pre_position_dict = {
        "x": float(pre_position[0]),
        "y": float(pre_position[1]),
        "z": float(pre_position[2])
    }
    return pre_position_dict, orientation


"""
电机服务相关函数
"""


def done_cb(state, result):
    # 服务端返回的结果：LegMoveResult(success, message)
    if state == actionlib.GoalStatus.SUCCEEDED:
        print(f"腿部电机服务完成: success={result.success}, message={result.message}")
    else:
        success = getattr(result, 'success', False)
        message = getattr(result, 'message', '')
        print(f"腿部电机服务结束，状态码={state}, success={success}, message={message}")


def active_cb():
    print("腿部电机运动服务被激活....")


def fb_cb(fb):
    print("当前进度:%.2f", fb.progress_bar)


def leg_client_start(target_positions, timeout=25.0):
    global _legmove_client, _legs_moving
    import time
    
    # 等待机械臂运动完成
    wait_start = time.time()
    wait_timeout = 65.0
    while True:
        with _arms_lock:
            if not _arms_moving:
                break
        if time.time() - wait_start > wait_timeout:
            print("Timeout waiting for arms to finish before leg move")
            return False
        time.sleep(0.2)
    
    # 等待动作服务器可用
    if not _legmove_client.wait_for_server(rospy.Duration(5.0)):
        print("Leg move action server not available")
        return False
    
    with _legs_lock:
        _legs_moving = True
    
    try:
        goal_obj = LegMoveGoal()
        goal_obj.target_positions = target_positions
        _legmove_client.send_goal(goal_obj, done_cb=done_cb, active_cb=active_cb, feedback_cb=fb_cb)
        finished = _legmove_client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            print("Leg move timed out; canceling goal")
            _legmove_client.cancel_goal()
            return False
        result = _legmove_client.get_result()
        if result is None:
            print("Leg move result is None")
            return False
        return bool(getattr(result, 'success', False))
    finally:
        with _legs_lock:
            _legs_moving = False


"""
运动规划相关函数
"""
def clear_temp_obstacles():
    global _scene
    try:
        _scene.remove_world_object()  # 不传 name = 清空全部
        rospy.sleep(0.3)
    except Exception as e:
        rospy.logwarn(f"Failed to clear obstacles: {e}")

def add_front_wall(
    frame_id="BASE_S",
    wall_x=0.5,
    wall_y=0.0,
    wall_z=1.0,
    thickness=0.05,
    width=2.0,
    height=2.0
):
    """
    在机器人前方添加一堵墙（Collision Object）
    """
    global _scene

    wall_pose = PoseStamped()
    wall_pose.header.frame_id = frame_id
    wall_pose.header.stamp = rospy.Time.now()

    #  注意：必须访问 pose 内的 position 和 orientation
    wall_pose.pose.position.x = wall_x
    wall_pose.pose.position.y = wall_y
    wall_pose.pose.position.z = wall_z
    wall_pose.pose.orientation.w = 1.0

    _scene.add_box(
        name="temp_front_wall",
        pose=wall_pose,
        size=(thickness, width, height)
    )

    rospy.sleep(0.3)  # 给 PlanningSceneMonitor 同步时间

def create_cartesian_position_constraint(
    group_name,
    frame_id="BASE_S",  # 约束参考坐标系（机器人基坐标系）
    x_min=-4.0, x_max=4.0,
    y_min=-4.0, y_max=4.0,
    z_min=-1.0, z_max=4.0
):
    """创建笛卡尔空间位置约束（立方体范围）- 修复Python列表兼容问题"""
    # 1. 定义立方体原语（SolidPrimitive）
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX  # 指定为立方体类型
    # 设置立方体尺寸：x/y/z轴的长度（max - min）
    box.dimensions = [
        x_max - x_min,  # x轴长度
        y_max - y_min,  # y轴长度
        z_max - z_min   # z轴长度
    ]

    # 2. 定义立方体中心点（在参考坐标系下的位置）
    center_pose = Pose()
    center_pose.position.x = (x_min + x_max) / 2.0
    center_pose.position.y = (y_min + y_max) / 2.0
    center_pose.position.z = (z_min + z_max) / 2.0
    # 姿态默认（无旋转）
    center_pose.orientation.w = 1.0

    # 3. 定义边界体积（BoundingVolume）- 修复resize()问题
    bounding_volume = BoundingVolume()
    # Python中直接append原语，而非resize()
    bounding_volume.primitives.append(box)
    bounding_volume.primitive_poses.append(center_pose)

    # 4. 创建位置约束
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = frame_id
    position_constraint.link_name = get_end_effector_link(group_name)  # 末端执行器链接
    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0  # 约束权重（1.0=严格遵守）

    # 5. 整合到Constraints
    constraints = Constraints()
    constraints.position_constraints.append(position_constraint)
    return constraints


def solve_ik_for_pose(group_name, target_pose, timeout=0.05, path_constraints=None):
    global _ik_service
    if _ik_service is None:
        rospy.wait_for_service('compute_ik', timeout=10.0)
        _ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # 创建IK请求
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = group_name
    ik_request.ik_request.pose_stamped.header.frame_id = "BASE_S"
    ik_request.ik_request.pose_stamped.pose = target_pose
    ik_request.ik_request.timeout = rospy.Duration(timeout)

    # 设置末端执行器链接名称
    ik_request.ik_request.ik_link_name = get_end_effector_link(group_name)
    # 设置起始状态
    robot_state = RobotState()
    robot_state.joint_state = _current_joint_state
    ik_request.ik_request.robot_state = robot_state

    # 如果有路径约束，添加到IK请求中
    if path_constraints is not None:
        ik_request.ik_request.constraints = path_constraints
    print(f"Get current joint state: {robot_state.joint_state.position}")
    # 调用IK服务
    ik_response = _ik_service(ik_request)
    print(f"Group: {group_name}")
    print(f"Target pose: {target_pose}")
    print(f"  End-effector link: {ik_request.ik_request.ik_link_name}")

    if ik_response.error_code.val == ik_response.error_code.SUCCESS:
        joint_values = list(ik_response.solution.joint_state.position)
        joint_names = list(ik_response.solution.joint_state.name)
        print(f"IK solution found!")
        return joint_values, joint_names
    else:
        print(f"IK solver failed with error code: {ik_response.error_code.val}")
        return None, None


def plan_to_joint_target(move_group, joint_values, joint_names):
    global _arms_moving
    with _arms_lock:
        _arms_moving = True
    try:
        # 只保留属于当前规划组的关节，否则ik逆解会把其他关节也设置为目标
        group_joint_names = move_group.get_active_joints()
        if joint_names and joint_values:
            full_joint_goal = dict(zip(joint_names, joint_values))
            filtered_joint_goal = {}
            for joint_name in group_joint_names:
                if joint_name in full_joint_goal:
                    filtered_joint_goal[joint_name] = full_joint_goal[joint_name]
            move_group.set_joint_value_target(filtered_joint_goal)
        else:
            move_group.set_joint_value_target(joint_values)
        move_group.set_planning_time(0.5)
        move_group.set_num_planning_attempts(5)
        move_group.set_max_velocity_scaling_factor(0.8)
        move_group.set_max_acceleration_scaling_factor(0.8)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        return success
    except Exception as e:
        print(f"Error in joint space planning: {str(e)}")
        return False
    finally:
        with _arms_lock:
            _arms_moving = False

def plan_to_pose(move_group, pose_target, path_constraints,pose_tol=0.01, ori_tol=0.01):
    """
    使用关节空间规划从当前位置移动到目标位姿
    
    Args:
        move_group: MoveGroupCommander对象
        pose_target: 目标位姿 (geometry_msgs.msg.Pose)
        pose_tol: 位置误差容限
        ori_tol: 姿态误差容限
    """
    try:
        # 清理历史目标
        move_group.clear_pose_targets()
        # 清理路径约束
        move_group.clear_path_constraints()
        # 设置末端 link（非常重要，避免歧义）
        move_group.set_end_effector_link(
            move_group.get_end_effector_link()
        )

        # 设置目标位姿
        move_group.set_pose_target(pose_target)

        # 设置容差（中等偏松）
        move_group.set_goal_position_tolerance(pose_tol)      # 1 cm
        move_group.set_goal_orientation_tolerance(ori_tol)   # ~5.7°
        if path_constraints is not None:
            move_group.set_path_constraints(path_constraints)

        # 规划参数
        move_group.set_planning_time(1.0)
        move_group.set_num_planning_attempts(5)
        move_group.set_max_velocity_scaling_factor(0.8)
        move_group.set_max_acceleration_scaling_factor(0.8)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return success

    except Exception as e:
        rospy.logerr(f"Pose target planning failed: {e}")
        return False

def plan_cartesian_path(move_group, target_pose, eff_step=0.01):
    """
    使用笛卡尔直线规划从当前位置移动到目标位姿
    
    Args:
        move_group: MoveGroupCommander对象
        target_pose: 目标位姿 (geometry_msgs.msg.Pose)
        eff_step: 笛卡尔路径步长，越小路径点越密集
    
    Returns:
        bool: 规划和执行是否成功
    """
    # 获取当前末端执行器位姿
    current_pose_stamped = move_group.get_current_pose()
    if current_pose_stamped is None:
        print("Failed to get current pose for cartesian planning")
        return False
    
    # 创建路径点：从当前位置到目标位置
    waypoints = []
    waypoints.append(current_pose_stamped.pose)
    waypoints.append(target_pose)
    
    # 使用compute_cartesian_path进行笛卡尔直线规划
    # eff_step越小，路径点越密集
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        eff_step,  # eef_step: 笛卡尔路径步长，可减小此值获得更密集的路径点
        True       # avoid_collisions: 是否避免碰撞
    )
    
    print(f"Cartesian path planning completed: {fraction * 100:.2f}% achieved")
    
    # 如果路径规划成功率大于等于0.9，执行运动
    if fraction >= 0.9:
        # 执行笛卡尔路径
        move_group.set_max_velocity_scaling_factor(0.8)
        move_group.set_max_acceleration_scaling_factor(0.8)
        success = move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        
        if success:
            print("Cartesian path executed successfully!")
        else:
            print("Cartesian path execution failed!")
        return success
    else:
        print(f"Cartesian path planning failed: only {fraction * 100:.2f}% of path achieved (required >= 90%)")
        return False


def plan_pose_once(group_name, position, orientation, is_straight_constraint=False, is_path_constraint=False,eff_step=0.01):
    global _arms_moving
    move_group = get_move_group(group_name)
    if move_group is None:
        print(f"Invalid group_name: {group_name}")
        return False

    # 使用订阅到的真实关节状态作为起始状态
    if _current_joint_state is not None:
        start_state = move_group.get_current_state()
        start_state.joint_state.header = _current_joint_state.header
        start_state.joint_state.name = _current_joint_state.name
        start_state.joint_state.position = _current_joint_state.position
        start_state.joint_state.velocity = _current_joint_state.velocity
        start_state.joint_state.effort = _current_joint_state.effort
        move_group.set_start_state(start_state)
    else:
        move_group.set_start_state_to_current_state()

    # 创建目标姿态
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = float(position['x'])
    pose_goal.position.y = float(position['y'])
    pose_goal.position.z = float(position['z'])
    pose_goal.orientation.x = float(orientation['x'])
    pose_goal.orientation.y = float(orientation['y'])
    pose_goal.orientation.z = float(orientation['z'])
    pose_goal.orientation.w = float(orientation['w'])
    if is_path_constraint:
        path_constraint=create_cartesian_position_constraint(group_name)
    else:
        path_constraint=None

    with _arms_lock:
        _arms_moving = True
    try:
        if is_straight_constraint:
            return plan_cartesian_path(move_group, pose_goal, eff_step)
        else:
            return plan_to_pose(move_group, pose_goal, path_constraint, pose_tol=0.01, ori_tol=0.01)
    finally:
        with _arms_lock:
            _arms_moving = False


"""
flask接口函数
"""


@app.route('/leg_move', methods=['POST'])
def leg_move():
    data = request.get_json()
    if data is None:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400
    if 'target_positions' not in data:
        return jsonify({
            "success": False,
            "error": "Missing target_positions"
        }), 400

    target_positions = data['target_positions']
    # 调用腿部动作客户端并等待执行结果
    success = leg_client_start(target_positions)
    print("success=", success)
    if success:
        return jsonify({
            "success": True,
            "message": "Leg move command sent successfully",
            "target_positions": target_positions
        }), 200
    else:
        return jsonify({
            "success": False,
            "error": "failed"
        }), 500


@app.route('/get_current_pose_http', methods=['GET'])
def get_current_pose_http():
    if not _initialized:
        return jsonify({
            "success": False,
            "error": "MoveIt interface not initialized"
        }), 503

    group_name = request.args.get('group_name')
    if not group_name:
        return jsonify({
            "success": False,
            "error": "Missing required parameter: group_name"
        }), 400

    move_group = get_move_group(group_name)
    if move_group is None:
        return jsonify({
            "success": False,
            "error": f"Invalid group_name: {group_name}"
        }), 400
    current_pose_stamped = move_group.get_current_pose()
    if current_pose_stamped is None:
        return jsonify({
            "success": False,
            "error": "Failed to get current pose: pose_stamped is None"
        }), 500
    current_pose = current_pose_stamped.pose
    if current_pose is None:
        return jsonify({
            "success": False,
            "error": "Failed to get current pose: pose is None"
        }), 500

    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(current_pose.position.x, 6),
                "y": round(current_pose.position.y, 6),
                "z": round(current_pose.position.z, 6)
            },
            "orientation": {
                "x": round(current_pose.orientation.x, 6),
                "y": round(current_pose.orientation.y, 6),
                "z": round(current_pose.orientation.z, 6),
                "w": round(current_pose.orientation.w, 6)
            }
        }
    }
    return jsonify(response_data), 200


@app.route('/get_current_joint_states', methods=['GET'])
def get_current_joint_states():
    if not _initialized:
        return jsonify({
            "success": False,
            "error": "MoveIt interface not initialized"
        }), 503

    group_name = request.args.get('group_name')
    if not group_name:
        return jsonify({
            "success": False,
            "error": "Missing required parameter: group_name"
        }), 400

    move_group = get_move_group(group_name)
    if move_group is None:
        return jsonify({
            "success": False,
            "error": f"Invalid group_name: {group_name}"
        }), 400

    # 检查关节状态是否可用
    if _current_joint_state is None:
        return jsonify({
            "success": False,
            "error": "Joint state not available"
        }), 503

    # 获取该动作组的活动关节名称列表
    try:
        active_joints = move_group.get_active_joints()
    except Exception as e:
        return jsonify({
            "success": False,
            "error": f"Failed to get active joints: {str(e)}"
        }), 500

    # 从当前关节状态中提取对应关节的角度值
    joint_states_dict = {}
    joint_names = list(_current_joint_state.name)
    joint_positions = list(_current_joint_state.position)

    # 创建关节名称到位置的映射
    joint_map = dict(zip(joint_names, joint_positions))

    # 提取该动作组的关节角度
    for joint_name in active_joints:
        if joint_name in joint_map:
            joint_states_dict[joint_name] = round(float(joint_map[joint_name]), 6)
        else:
            # 如果关节名称不在当前关节状态中，记录警告但继续
            rospy.logwarn(f"Joint '{joint_name}' not found in current joint state")

    if not joint_states_dict:
        return jsonify({
            "success": False,
            "error": f"No joint states found for group '{group_name}'"
        }), 500

    response_data = {
        "success": True,
        "group_name": group_name,
        "joint_states": joint_states_dict
    }
    return jsonify(response_data), 200


@app.route('/calculate_pre_position', methods=['POST'])
def calculate_pre_position():
    data = request.get_json()
    if not data:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400

    group_name = data.get('group_name', 'Right_arm')
    position = data.get('position', {})
    orientation = data.get('orientation', {})
    distance = float(data.get('distance', 0.1))
    # 调用计算函数
    pre_position, pre_orientation = caculate_pre_pose_from_target_pose(position, orientation, distance)
    if pre_position is None or pre_orientation is None:
        return jsonify({
            "success": False,
            "error": f"Failed to calculate pre_position"
        }), 500

    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(pre_position['x'], 6),
                "y": round(pre_position['y'], 6),
                "z": round(pre_position['z'], 6)
            },
            "orientation": {
                "x": round(pre_orientation['x'], 6),
                "y": round(pre_orientation['y'], 6),
                "z": round(pre_orientation['z'], 6),
                "w": round(pre_orientation['w'], 6)
            }
        }
    }
    return jsonify(response_data), 200


@app.route('/calculate_target_position_from_pixel', methods=['POST'])
def calculate_target_position_from_pixel():
    data = request.get_json()
    if not data:
        return jsonify({
            "success": False,
            "error": "No JSON data provided"
        }), 400

    group_name = data.get('group_name', 'Right_arm')
    pixel_x = data.get('x')
    pixel_y = data.get('y')
    depth = data.get('d')
    theta = data.get('theta', 0.0)
    camera_frame = 'camera_color_optical_frame'
    base_frame = 'BASE_S'

    target_position, target_orientation = caculate_target_pose_from_pixel_pose(
        pixel_x, pixel_y, depth, theta,
        camera_frame, base_frame
    )

    if target_position is None or target_orientation is None:
        return jsonify({
            "success": False,
            "error": "Failed to calculate target pose from pixel"
        }), 500

    response_data = {
        "success": True,
        "group_name": group_name,
        "pose": {
            "position": {
                "x": round(target_position['x'], 6),
                "y": round(target_position['y'], 6),
                "z": round(target_position['z'], 6)
            },
            "orientation": {
                "x": round(target_orientation['x'], 6),
                "y": round(target_orientation['y'], 6),
                "z": round(target_orientation['z'], 6),
                "w": round(target_orientation['w'], 6)
            }
        }
    }
    return jsonify(response_data), 200


@app.route('/plan_to_position', methods=['POST'])
def plan_to_position():
    import time
    # 等待腿部移动完成
    wait_start = time.time()
    wait_timeout = 65.0
    while True:
        with _legs_lock:
            if not _legs_moving:
                break
        if time.time() - wait_start > wait_timeout:
            return jsonify({"error": "Timeout waiting for legs to finish"}), 503
        time.sleep(0.2)
    
    data = request.get_json()
    if data is None:
        return jsonify({"error": "No JSON data provided"}), 400
    if 'group_name' not in data:
        return jsonify({"error": "Missing group_name"}), 400
    if 'position' not in data:
        return jsonify({"error": "Missing position"}), 400
    if 'orientation' not in data:
        return jsonify({"error": "Missing orientation"}), 400

    group_name = data['group_name']
    position = data['position']
    orientation = data['orientation']
    is_straight_constraint = data.get('is_straight_constraint', False)  # 默认为False
    is_path_constraint = data.get('is_path_constraint')
    eff_step = float(data.get('eff_step', 0.001))  # 笛卡尔路径步长，默认0.01米

    ### 在planning scene中添加障碍物 ###

    # 1. 清空上一次规划残留的障碍
    clear_temp_obstacles()

    # 2. 判断是否需要加墙（由上位机 / 感知 / 逻辑决定）
    is_wall_limit = data.get('is_wall_limit', False)
    if is_wall_limit:
        if 'wall_x_distance' not in data:
            return jsonify({"error": "Missing wall_x_distance when is_wall_limit=True"}), 400
        wall_x_distance = float(data['wall_x_distance'])
        add_front_wall(
            frame_id="BASE_S",
            wall_x=wall_x_distance,
            wall_y=0.0,
            wall_z=1.0,
            thickness=0.05,
            width=3.0,
            height=3.0
        )

    ok = plan_pose_once(group_name, position, orientation, is_straight_constraint, is_path_constraint , eff_step)
    if ok:
        return jsonify({"message": "Success"}), 200
    else:
        return jsonify({"message": "Failed to reach goal position"}), 500

@app.route('/plan_to_specific_joint', methods=['POST'])
def plan_to_specific_joint():
    import time
    # 等待腿部移动完成
    wait_start = time.time()
    wait_timeout = 65.0
    while True:
        with _legs_lock:
            if not _legs_moving:
                break
        if time.time() - wait_start > wait_timeout:
            return jsonify({"error": "Timeout waiting for legs to finish"}), 503
        time.sleep(0.2)
    
    data = request.get_json()
    if data is None:
        return jsonify({"error": "No JSON data provided"}), 400
    if 'group_name' not in data:
        return jsonify({"error": "Missing group_name"}), 400
    if 'joint_names' not in data:
        return jsonify({"error": "Missing joint_names"}), 400
    if 'joint_values' not in data:
        return jsonify({"error": "Missing joint_values"}), 400


    group_name = data['group_name']
    joint_names = data['joint_names']
    joint_values = data['joint_values']
    
    # 1. 清空上一次规划残留的障碍
    clear_temp_obstacles()

    # 2. 判断是否需要加墙（由上位机 / 感知 / 逻辑决定）
    is_wall_limit = data.get('is_wall_limit', False)
    if is_wall_limit:
        if 'wall_x_distance' not in data:
            return jsonify({"error": "Missing wall_x_distance when is_wall_limit=True"}), 400
        wall_x_distance = float(data['wall_x_distance'])
        add_front_wall(
            frame_id="BASE_S",
            wall_x=wall_x_distance,
            wall_y=0.0,
            wall_z=1.0,
            thickness=0.05,
            width=3.0,
            height=3.0
        )
    # 获取 MoveGroupCommander 对象
    move_group = get_move_group(group_name)
    if move_group is None:
        return jsonify({"error": f"Invalid group_name: {group_name}"}), 400

    # 按正确顺序调用：move_group, joint_values, joint_names
    success = plan_to_joint_target(move_group, joint_values, joint_names)
    if success:
        return jsonify({"message": "Success"}), 200
    else:
        return jsonify({"message": "Failed to reach goal position"}), 500

    

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('http_moveit_bridge', anonymous=True, disable_signals=True)

    # 初始化MoveIt接口
    if not initialize_moveit():
        print("Failed to initialize MoveIt interface")
        sys.exit(1)

    # 获取参数
    host = str(rospy.get_param('~host', '0.0.0.0'))
    port = int(rospy.get_param('~port', 7550))
    debug = bool(rospy.get_param('~debug', True))
    print(f"Starting HTTP server on {host}:{port}")
    # 启动Flask应用
    app.run(host=host, port=port, debug=debug, use_reloader=False, threaded=True)
