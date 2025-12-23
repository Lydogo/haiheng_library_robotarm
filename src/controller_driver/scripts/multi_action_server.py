#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Arm MoveIt Action Server Node
控制多个 CAN 通道下的机械臂，并提供 MoveIt Action 接口。
"""
###打印当前工作目录###
import os
print("Current working directory:", os.getcwd())
import math
import rospy
import threading
import actionlib
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from controller_driver.msg import *

# 导入你已有的控制类
from motor_driver_pkg.arm_ctrl import ArmController   # 含多电机并行控制能力
#from motor_driver_pkg.motor_controller import MotorController  # 提供电流/错误读取等底层接口
#from src.motor_driver_pkg.arm_ctrl import ArmController   # 含多电机并行控制能力
# from src.motor_driver_pkg.motor_controller import MotorController  # 提供电流/错误读取等底层接口


# # 弧度与编码器计数转换函数
# def rad_to_cnt(rad):
#     # 示例转换函数（实际请根据编码器参数调整）
#     return int(rad / 0.001)  # 假设每 count = 0.001 rad
# # 编码器技术与弧度转换函数
# def cnt_to_rad(cnt):
#     # 示例转换函数（实际请根据编码器参数调整）
#     return cnt * 0.001  # 假设每 count = 0.001 rad
# 通过配置文件加载机械臂关节对应的电机 ID 列表

# ============================================================
#                MultiArmMoveItActionServer 主类
# ============================================================
fw_fb = 1.0

class MultiArmMoveItActionServer:
    def __init__(self):
        rospy.init_node("multi_arm_moveit_action_server", anonymous=False)

        # ───────────── 参数加载 ─────────────
        self.arm_names = rospy.get_param("~arm_names", ["waist","left_arm", "right_arm","head"])
        # self.joint_names = rospy.get_param("~joint_names", [])
        self.config_file = rospy.get_param("~config_file", "config/arm_bind.yaml")
        self.current_threshold_ma = rospy.get_param("~current_threshold_ma", 15000)
        self.publish_rate_hz = rospy.get_param("~publish_rate", 10)
        self.monitor_rate_hz = rospy.get_param("~monitor_rate", 1)

        # ───────────── 初始化 ArmController 实例 ─────────────
        # 初始化一个实例就够了，把要控制的arm都放在这个实例里控制

        self.arm=ArmController(side_names=self.arm_names, config_file=self.config_file)
        self.waist_break_on_arrival()  # 初始化时让腰部电机进入制动状态

        # ───────────── 共享状态 ─────────────
        self.state_lock = threading.Lock()
        self.current_positions = {}
        self.current_velocities = {}
        self.error_flag = False
        self.legs_moving = False  # 腿部移动期间锁住机械臂
        self.arms_executing = set()  # 正在执行轨迹的机械臂名称集合
        self.arms_lock = threading.Lock()  # 保护 arms_executing 的锁

        # ───────────── ROS 通信接口 ─────────────
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.diagnostics_pub = rospy.Publisher("/motor_diagnostics", DiagnosticArray, queue_size=5)
        #腿部服务
        self.legs_action_server = actionlib.SimpleActionServer(
            "/legs_controller/leg_move",
            LegMoveAction,
            execute_cb=self.legs_break_on_arrival_cb,
            auto_start=False
        )
        threading.Thread(target=self.legs_action_server.start, daemon=True).start()
        rospy.loginfo("[ACTION SERVER] /legs_controller/leg_move ready.")

        # ——————————为每个move_group创建独立的 action server———————        

        self.action_servers = {}
        for arm_name in self.arm_names:
            action_name = f"/{arm_name}_controller/follow_joint_trajectory"
            server = actionlib.SimpleActionServer(
                action_name,
                FollowJointTrajectoryAction,
                execute_cb=lambda goal, arm=arm_name: self.execute_cb(goal, arm),
                ###actionlib 的回调函数只接受一个参数 goal，但我们需要知道这个回调是属于哪个机械臂的，所以用 lambda 包装一下，让它既能接收 goal，又能知道是哪个 arm_name####
                
                auto_start=False
            )
            self.action_servers[arm_name] = server

            # 独立线程启动 action server
            threading.Thread(target=server.start, daemon=True).start()
            rospy.loginfo(f"[ACTION SERVER] {action_name} ready.")


        # ───────────── 启动后台线程 ────────────
        threading.Thread(target=self._joint_state_publisher, daemon=True).start()
        # 电机监控已整合到_joint_state_publisher中，每monitor_interval个周期执行一次诊断
        self.monitor_interval = max(1, int(self.publish_rate_hz / self.monitor_rate_hz))


        rospy.loginfo("[SYSTEM] Multi-Arm MoveIt Action Server running.")

        # ───────────── 注册 shutdown 回调 ─────────────
        rospy.on_shutdown(self._shutdown_cleanup)
        rospy.loginfo("[SYSTEM] Shutdown handler registered.")

        rospy.spin()

    def waist_break_on_arrival(self):

        # motor_id= self.arm.joint_name_to_motor_id.get("WAIST_R", None)
        motor_ids=[13,15]
        if motor_ids is not None:
            self.arm.motors[14].set_target_position(0) #让WAIST_Y回到0位置
            commands = [(motor_id,'BRKTO',0) for motor_id in motor_ids]
            self.arm.set_async_move(commands, is_check_move=False)

    def _any_arm_executing(self):
        """检查是否有任何机械臂正在执行轨迹"""
        with self.arms_lock:
            return len(self.arms_executing) > 0

    def legs_break_on_arrival_cb(self,goal):
        """
        腿部电机的server回调服务，传入动作参数
        """
        print("Received leg move goal:", goal.target_positions)
        
        # 机械臂运动期间拒绝腿部请求
        if self._any_arm_executing():
            rospy.logwarn(f"[LEGS ACTION] Rejected: arms are moving, legs locked.")
            res = LegMoveResult()
            res.success = False
            res.message = "Arms are moving, legs locked."
            self.legs_action_server.set_aborted(res)
            return
        
        self.legs_moving = True
        try:
            motor_ids= self.arm.ARM_CONFIG["legs"]["motor_ids"]
            if motor_ids is not None:
                commands = [(motor_id,'BRKTO',goal.target_positions[i]) for motor_id,i in zip(motor_ids,range(len(motor_ids)))]
                self.arm.set_async_move(commands, is_check_move=False)

        except Exception as e:
            self.legs_moving = False
            rospy.logerr(f"[LEGS ACTION] Failed to send leg positions: {e}")
            res=LegMoveResult()
            res.success=False
            res.message=str(e)
            self.legs_action_server.set_aborted(res)
            return
        
        # 等待BRKTO监控线程完成所有任务后再返回
        import time
        timeout = 60
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.arm.monitoring_lock:
                if not self.arm.brkto_tasks:
                    break
            time.sleep(0.2)
        
        self.legs_moving = False
        res=LegMoveResult()
        res.success=True
        res.message="Legs moved successfully."
        self.legs_action_server.set_succeeded(res)

    # ============================================================
    #                Joint State 发布线程
    # ============================================================
    # joint_states消息示例：
    # ---
    #     header: 
    #   seq: 10262
    #   stamp: 
    #     secs: 1761033517
    #     nsecs: 414884567
    #   frame_id: ''
    # name: 
    #   - WAIST_R
    #   - WAIST_Y
    #   - WAIST_P
    #   - R_SHOULDER_P
    #   - R_SHOULDER_R
    #   - R_SHOULDER_Y
    #   - R_ELBOW_Y
    #   - R_WRIST_P
    #   - R_WRIST_Y
    #   - R_WRIST_R
    #   - L_SHOULDER_P
    #   - L_SHOULDER_R
    #   - L_SHOULDER_Y
    #   - L_ELBOW_Y
    #   - L_WRIST_P
    #   - L_WRIST_Y
    #   - L_WRIST_R
    #   - NECK_Y
    #   - NECK_P
    #   - NECK_R
    # position: [0.0, 0.0, 0.0, -0.7634958393548881, 1.3619239295405314, 2.612671641606576, 1.886643603331932, -2.4280815190133826, -0.8568889869109686, 0.08956826518391328, -5.851699505001307e-07, 1.5381025744320358, 6.655037207528949e-05, 2.438206884544343e-05, 1.1454376531764866e-05, 7.217745948582889e-05, -7.967071095481516e-06, 0.0, 0.0, 0.0]
    # velocity: []
    # effort: []
    # ---



    # 根据motor_id_list与joint_names的对应关系，按照joint_state信息示例的顺序和名称来发布话题。
    def _joint_state_publisher(self):
        rate = rospy.Rate(self.publish_rate_hz)
        cached_states = None
        cycle_count = 0
        while not rospy.is_shutdown():
            with self.state_lock:
                with self.arm.monitoring_lock:
                    has_brkto = bool(self.arm.brkto_tasks)
                
                if has_brkto and cached_states is not None:
                    states = cached_states
                else:
                    states = self.arm.get_all_motor_states()
                    if states and len(states) == len(self.arm.motor_id_list):
                        cached_states = states
                
                # 从states提取位置
                motor_positions = []
                for motor_id in self.arm.motor_id_list:
                    state = states.get(motor_id) if states else None
                    motor_positions.append(state.get('position_cnt') if state else None)
                
                if None in motor_positions or len(motor_positions) != len(self.arm.joint_names_list):
                    rospy.logwarn(f"[JOINT STATE] Invalid motor states read")
                    rate.sleep()
                    continue

                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = self.arm.joint_names_list
                msg.position = [self._cnt_to_rad(p) for p in motor_positions]
                msg.velocity = [0.0] * len(msg.name)
                self.current_positions = dict(zip(msg.name, msg.position))

            self.joint_state_pub.publish(msg)
            
            # 周期性监控电流和错误码
            cycle_count += 1
            if cycle_count >= self.monitor_interval and not has_brkto and states:
                cycle_count = 0
                try:
                    error_statuses = self.arm.get_error_statuses()
                except Exception as e:
                    rospy.logwarn(f"[MONITOR] Error status read failed: {e}")
                    error_statuses = {}
                
                for motor_id in self.arm.motor_id_list:
                    state = states.get(motor_id)
                    err_status = error_statuses.get(motor_id)
                    current_ma = state.get('current_ma') if state else None
                    speed = state.get('speed_001hz') if state else None
                    position = state.get('position_cnt') if state else None
                    err_str = f"0x{err_status:08X}" if err_status is not None else "None"
                    # rospy.loginfo(f"[MONITOR] ID={motor_id}, CURR={current_ma}mA, SPD={speed}, POS={position}, ERR_CODE={err_str}")
                    #
                    # if current_ma is not None and abs(current_ma) > self.current_threshold_ma:
                    #     rospy.logwarn(f"[MONITOR] Motor {motor_id} overcurrent: {current_ma}mA")
                    if err_status is not None and err_status != 0:
                        err_msg=f"[MONITOR] Motor {motor_id} error=0x{err_status:08X}"
                        rospy.logerr(err_msg)
                        raise Exception(err_msg)
            
            rate.sleep()


    # ============================================================
    #                Action Server 执行回调
    # ============================================================
    #/follow_joint_trajectory/goal 消息示例：
    # ---
    #     header: 
    #   seq: 1
    #   stamp: 
    #     secs: 1761036800
    #     nsecs: 899251683
    #   frame_id: ''
    # goal_id: 
    #   stamp: 
    #     secs: 1761036800
    #     nsecs: 899252674
    #   id: "/move_group-2-1761036800.899252674"
    # goal: 
    #   trajectory: 
    #     header: 
    #       seq: 0
    #       stamp: 
    #         secs: 0
    #         nsecs:         0
    #       frame_id: "BASE_S"
    #     joint_names: 
    #       - L_ELBOW_Y
    #       - L_SHOULDER_P
    #       - L_SHOULDER_R
    #       - L_SHOULDER_Y
    #       - L_WRIST_P
    #       - L_WRIST_R
    #       - L_WRIST_Y
    #     points: 
    #       - 
    #         positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         accelerations: [0.0, -0.07999853998319502, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         effort: []
    #         time_from_start: 
    #           secs: 0
    #           nsecs:         0
    # ---
    # 根据实际机械臂的关节命名和顺序，调整arm.set_target_positions(point.positions)电机id和goal.trajectory.points关节顺序的对应关系
    def execute_cb(self, goal, arm_name):
        rospy.loginfo(f"[ACTION] Received FollowJointTrajectory goal for {arm_name}.")
        action_server = self.action_servers[arm_name]

        # 0. 腿部移动期间拒绝机械臂请求
        if self.legs_moving:
            rospy.logwarn(f"[ACTION] Rejected: legs are moving, arm {arm_name} locked.")
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            res.error_string = "Legs are moving, arm locked."
            action_server.set_aborted(res)
            return

        # 标记该机械臂开始执行
        with self.arms_lock:
            self.arms_executing.add(arm_name)

        try:
            # 1. 基础检查
            if not goal.trajectory.points:
                res = FollowJointTrajectoryResult()
                res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
                res.error_string = "Empty trajectory."
                action_server.set_aborted(res)
                return

            # 2. 按时间执行轨迹
            start_time = rospy.Time.now()
            for point_idx, point in enumerate(goal.trajectory.points):
                if action_server.is_preempt_requested():
                    rospy.logwarn(f"[ACTION] Goal preempted by client for {arm_name}.")
                    self._stop_all()
                    action_server.set_preempted()
                    return

                fb = FollowJointTrajectoryFeedback()
                target_time = start_time + point.time_from_start

                with self.state_lock:
                    fb.joint_names = goal.trajectory.joint_names
                    fb.actual.positions = [self.current_positions[jn] for jn in goal.trajectory.joint_names]
                    fb.desired.positions = list(point.positions)
                    action_server.publish_feedback(fb)
                if all(abs(a - d) < 0.02 for a, d in zip(fb.actual.positions, fb.desired.positions)):
                    continue

                while rospy.Time.now() < target_time and all(abs(a - d) < 0.02 for a, d in zip(fb.actual.positions, fb.desired.positions)):
                    rospy.sleep(0.1)
                    if self.error_flag:
                        rospy.logerr(f"[ACTION] Hardware error detected! Abort trajectory for {arm_name}.")
                        res = FollowJointTrajectoryResult()
                        res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                        res.error_string = "Hardware fault detected"
                        self._stop_all()
                        action_server.set_aborted(res)
                        return

                try:
                    with self.state_lock:
                        velocities = list(point.velocities) if point.velocities else [0.0] * len(point.positions)
                        
                        motor_commands = []
                        for joint_name, position_rad, velocity_rad_s in zip(
                            goal.trajectory.joint_names, point.positions, velocities
                        ):
                            motor_id = self.arm.joint_name_to_motor_id[joint_name]
                            position_cnt = self._rad_to_cnt(position_rad)
                            reduction_ratio = self.arm.motor_reduction_ratios.get(motor_id, 101)
                            speed_001hz = int(fw_fb * self._rad_per_sec_to_speed_001hz(velocity_rad_s, reduction_ratio))
                            motor_commands.append((motor_id, position_cnt, speed_001hz))
                        
                        commands_by_device = self.arm._group_motors_by_device([cmd[0] for cmd in motor_commands])
                        cmd_map = {cmd[0]: (cmd[1], cmd[2]) for cmd in motor_commands}
                        for dev_idx, motor_ids in commands_by_device.items():
                            if motor_ids:
                                with self.arm.can_locks[dev_idx]:
                                    for motor_id in motor_ids:
                                        pos, spd = cmd_map[motor_id]
                                        self.arm.motors[motor_id].set_target_position_with_speed(pos, spd)
                        
                        fb.actual.positions = [self.current_positions[jn] for jn in goal.trajectory.joint_names]
                        fb.desired.positions = list(point.positions)
                        action_server.publish_feedback(fb)
                except Exception as e:
                    rospy.logerr(f"[ACTION] Failed to send positions for {arm_name}: {e}")
                    res = FollowJointTrajectoryResult()
                    res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                    res.error_string = str(e)
                    action_server.set_aborted(res)
                    return

            # 3. 执行完毕
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            action_server.set_succeeded(res)
            rospy.loginfo(f"[ACTION] Trajectory execution completed successfully for {arm_name}.")
        finally:
            # 清除该机械臂的执行标记
            with self.arms_lock:
                self.arms_executing.discard(arm_name)

    # ============================================================
    #                辅助函数
    # ============================================================
    def _handle_error(self, msg):
        rospy.logerr(f"[MONITOR ERROR] {msg}")
        self.error_flag = True
        diag = DiagnosticArray()
        d = DiagnosticStatus()
        d.name = "Motor Error"
        d.level = DiagnosticStatus.ERROR
        d.message = msg
        diag.status.append(d)
        diag.header.stamp = rospy.Time.now()
        self.diagnostics_pub.publish(diag)

    def _stop_all(self):
        rospy.logwarn("[SYSTEM] Stopping all motors.")       
        try:
            self.arm.stop_all_motors()
        except Exception as e:
            rospy.logwarn(f"[STOP] Failed to stop motors: {e}")

    def _shutdown_cleanup(self):
        rospy.loginfo("[SYSTEM] ROS node is shutting down. Stopping all motors and cleaning up...")
        
        try:
            #self.arm.stop_all_motors()
            del self.arm
            rospy.loginfo(f"[CLEANUP] motor stopped and deleted.")
        except Exception as e:
            rospy.logwarn(f"[CLEANUP] Failed to stop motor: {e}")

    @staticmethod
    def _cnt_to_rad(cnt):
        #2π rad对应262144 count
        return cnt * 2*math.pi/262144
    @staticmethod
    def _rad_to_cnt(rad):
        return int(rad * 262144/(2*math.pi))
    
    @staticmethod
    def _rad_per_sec_to_speed_001hz(rad_per_sec: float, reduction_ratio: int) -> int:
        """
        将关节端速度(rad/s)转换为电机端速度(0.01Hz)
        rad/s → 输出端RPM → 电机端RPM → 0.01Hz
        """
        if rad_per_sec == 0.0:
            return 0
        output_rpm = rad_per_sec * 60.0 / (2.0 * math.pi)
        motor_rpm = output_rpm * reduction_ratio
        speed_001hz = int(motor_rpm * 100.0 / 60.0)
        return max(-32768, min(32767, speed_001hz))

# ============================================================
#                节点入口
# ============================================================
if __name__ == "__main__":
    try:
        MultiArmMoveItActionServer()
    except rospy.ROSInterruptException:
        pass
