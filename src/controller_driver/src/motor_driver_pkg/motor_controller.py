# -*- coding: utf-8 -*-
import ctypes
import platform
import struct
import time
from typing import Dict, List, Optional, Tuple, Union
from motor_driver_pkg.CANalyst2_canbus_handler import CanbusHandler, VCI_CAN_OBJ, VCI_INIT_CONFIG


# --- 电机控制类 ---

class MotorController:
    """
    基于供应商提供的自定义电机协议，实现电机控制与信息获取。
    """

    def __init__(self, can_handler: CanbusHandler, motor_id: int):
        """
        :param can_handler: 已初始化的CanbusHandler实例。
        :param motor_id: 要通信的电机ID (1-127)。
        """
        if not (1 <= motor_id <= 127):
            raise ValueError("Motor ID must be between 1 and 127.")
        self.can = can_handler
        self.motor_id = motor_id

    def _create_frame(self, data: bytes) -> VCI_CAN_OBJ:
        """辅助函数，用于创建标准CAN数据帧"""
        frame = VCI_CAN_OBJ()
        frame.ID = self.motor_id
        frame.SendType = 0  # 正常发送
        frame.RemoteFlag = 0  # 数据帧
        frame.ExternFlag = 0  # 标准帧
        frame.DataLen = len(data)

        # 填充数据
        ubyte_array = (ctypes.c_ubyte * len(data))(*data)
        for i in range(len(data)):
            frame.Data[i] = ubyte_array[i]

        return frame

    def _send_command(self, cmd_code: int, data: bytes = b''):
        """发送一个指令到电机"""
        full_data = cmd_code.to_bytes(1, 'little') + data
        frame = self._create_frame(full_data)
        self.can.send(frame)

    def _read_command(self, cmd_code: int, timeout: float = 0.2) -> Optional[bytes]:
        """
        发送一个读取指令，并等待电机返回相应的数据。
        :param cmd_code: 要发送的指令功能码。
        :param timeout: 等待响应的超时时间（秒）。
        :return: 电机返回的数据部分 (不包含指令码)，超时或失败则返回None。
        """
        self._send_command(cmd_code)
        start_time = time.time()
        while time.time() - start_time < timeout:
            frames = self.can.receive(wait_time_ms=5)
            if frames:
                for frame in frames:
                    if frame.ID == self.motor_id and frame.Data[0] == cmd_code:
                        response_data = bytes(frame.Data[1:frame.DataLen])
                        return response_data
        return None

    # --- 电机控制指令 ---

    def stop_motor(self):
        """立刻停止电机并抱死刹车 (指令码 2, 0x02)"""
        print("发送停止指令...")
        self._send_command(2)

    def clear_errors(self):
        """清除电机错误 (指令码 11, 0x0B)"""
        print("发送清除错误指令...")
        self._send_command(11)

    def enable_motor(self) -> bool:
        """
        使能电机：命令电机保持其当前位置。
        这可以安全地解除抱闸，让电机准备好接收新的运动指令。
        :return: 操作是否成功 (True/False)。
        """
        print("\n[使能电机] 正在读取当前位置...")
        current_pos = self.get_current_position()

        if current_pos is None:
            print("  [错误] 无法读取当前位置，使能失败。")
            return False

        print(f"  读取到位置: {current_pos} cnt。正在发送位置保持指令...")
        self.set_target_position(current_pos)
        time.sleep(0.1)  # 短暂延时，确保电机处理指令
        print("  使能指令已发送，电机应处于活动状态。")
        return True

    # --- 参数设置指令 ---

    def set_target_position(self, position_cnt: int):
        """进入位置模式，并设置目标位置 (指令码 30, 0x1E)"""
        data = struct.pack('<i', position_cnt)  # '<i' 表示小端序有符号4字节整数
        self._send_command(30, data)

    def set_target_position_with_speed(self, position_cnt: int, speed_001hz: int):
        """
        前馈速度位置模式 (七字节指令, 指令码 0x11)
        格式: 1字节指令码 + 4字节位置(cnt) + 2字节速度前馈(0.01Hz)
        """
        speed_001hz = max(-32768, min(32767, speed_001hz))
        data = struct.pack('<ih', position_cnt, speed_001hz)
        self._send_command(0x11, data)

    def set_target_speed(self, speed_001hz: int):
        """进入速度模式，并设置目标速度 (指令码 29, 0x1D)"""
        data = struct.pack('<i', speed_001hz)
        self._send_command(29, data)

    def set_target_current(self, current_ma: int):
        """进入电流模式，并设置目标电流 (指令码 28, 0x1C)"""
        data = struct.pack('<i', current_ma)
        self._send_command(28, data)

    def set_position_feedforward_current(self, current_ma: int):
        """设置位置环前馈电流 (指令码 71, 0x47)"""
        data = struct.pack('<i', current_ma)
        self._send_command(71, data)

    # --- 信息获取指令 ---

    def get_current_current(self) -> Optional[int]:
        """获取当前电流 (指令码 4, 0x04)"""
        response = self._read_command(4)
        if response and len(response) == 4:
            return struct.unpack('<i', response)[0]
        return None

    def get_current_position(self) -> Optional[int]:
        """获取当前位置 (指令码 8, 0x08)"""
        response = self._read_command(8)
        #print(f"pos back raw:{response}")
        if response and len(response) == 4:
            return struct.unpack('<i', response)[0]
        return None

    def get_current_speed(self) -> Optional[int]:
        """获取当前速度 (指令码 6, 0x06)"""
        response = self._read_command(6)
        if response and len(response) == 4:
            return struct.unpack('<i', response)[0]
        return None

    def get_bus_voltage(self) -> Optional[float]:
        """获取母线电压 (指令码 20, 0x14)"""
        response = self._read_command(20)
        if response and len(response) == 4:
            voltage_val = struct.unpack('<i', response)[0]
            # 协议单位是V，但通常以mV形式传输，这里假设返回值为mV
            return voltage_val / 1000.0
        return None

    def get_motor_version(self):
        response = self._read_command(101)
        if response:
            version_motor = struct.unpack("<i", response)[0]
            return version_motor
        return None

    def get_error_status(self) -> Optional[int]:
        """获取错误状态 (指令码 10, 0x0A)"""
        response = self._read_command(10)
        if response and len(response) == 4:
            # 返回的是U32，按位表示错误
            return struct.unpack('<I', response)[0]
        return None

    def get_working_mode(self) -> Optional[int]:
        """获取当前工作模式 (指令码 3, 0x03)"""
        response = self._read_command(3)
        if response and len(response) == 4:
            return struct.unpack('<i', response)[0]
        return None

    def get_reduction_ratio(self) -> Optional[int]:
        """
        获取电机减速比 (六字节指令, 0x41 0x40)
        发送: 41 40 00 00 00 00
        回复: 41 40 XX XX XX XX (后4字节为减速比，小端序)
        """
        while self.can.receive(wait_time_ms=0):
            pass
        
        data = bytes([0x40, 0x00, 0x00, 0x00, 0x00])
        self._send_command(0x41, data)
        
        start_time = time.time()
        while time.time() - start_time < 0.5:
            frames = self.can.receive()
            if frames:
                for frame in frames:
                    # 响应格式: 41 40 XX XX XX XX (6字节)
                    if frame.ID == self.motor_id and frame.DataLen == 6:
                        response_data = bytes(frame.Data[:6])
                        # 验证响应头: 0x41 0x40
                        if response_data[0] == 0x41 and response_data[1] == 0x40:
                            # 后4字节为减速比 (小端序)
                            ratio = struct.unpack('<I', response_data[2:6])[0]
                            # 合理性检查: 减速比应为 51/81/101/121
                            if ratio in [51, 81, 101, 121]:
                                return ratio
                            else:
                                print(f"Warning: Motor {self.motor_id} unexpected ratio {ratio}")
                                return None
            time.sleep(0.01)
        print(f"Warning: Timed out waiting for reduction ratio from motor {self.motor_id}")
        return None

    def get_all_states(self, timeout: float = 0.2) -> Optional[Dict[str, int]]:
        """
        一次性获取电流、速度、位置 (指令码 65, 0x41)
        返回: 8字节组合数据 (2字节电流mA + 2字节速度0.01Hz + 4字节位置cnt)
        """
        self._send_command(65)
        start_time = time.time()
        while time.time() - start_time < timeout:
            frames = self.can.receive(wait_time_ms=5)
            if frames:
                for frame in frames:
                    if frame.ID == self.motor_id and frame.DataLen == 8:
                        data = bytes(frame.Data[:8])
                        try:
                            current, speed, position = struct.unpack('<hhi', data)
                            return {"current_ma": current, "speed_001hz": speed, "position_cnt": position}
                        except struct.error:
                            return None
        return None

    # --- 复合功能指令 ---

    def set_target_position_with_feedforward(self, position_cnt: int):
        """
        【复合功能】设置带前馈电流的目标位置。
        该功能会先使能电机，然后读取维持当前位置的电流，将其作为前馈值，最后发送位置指令。
        """
        print(f"\n--- 正在执行带前馈的位置控制，目标: {position_cnt} ---")

        # 步骤 1: 使能电机，让其维持当前位置
        if not self.enable_motor():
            print("  [错误] 使能电机失败，无法执行带前馈的移动。")
            return

        # 步骤 2: 读取当前维持位置的电流
        print("  [步骤 2] 正在读取维持当前姿态的电流...")
        # holding_current = self.get_current_current()
        holding_current = 8000

        if holding_current is not None and holding_current != 0:
            print(f"  [步骤 3] 读取到维持电流: {holding_current} mA")

            # 步骤 4: 将此电流设置为位置环前馈
            print(f"  [步骤 4] 正在设置 {holding_current} mA 为位置环前馈电流...")
            self.set_position_feedforward_current(holding_current)
            time.sleep(0.05)  # 短暂延时确保设置生效

        else:
            print(f"  [警告] 读取到的维持电流为 {holding_current} mA。将不使用前馈直接移动。")

        # 步骤 5: 发送最终的目标位置指令
        print(f"  [步骤 5] 正在发送目标位置 {position_cnt}...")
        self.set_target_position(position_cnt)
        print("--- 带前馈的位置指令已发送 ---")
