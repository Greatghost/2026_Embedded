#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboMaster 2026 裁判系统串口帧生成工具
用于生成裁判系统向底盘回传的串口帧数据，可通过串口调试助手发送

协议版本: V1.2.0 (20260209)
"""

import struct

# CRC8 查找表 (与官方协议一致)
# crc8 generator polynomial: G(x)=x8+x5+x4+1
CRC8_TABLE = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
]

# CRC16 查找表 (与官方协议一致)
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
]

HEADER_SOF = 0xA5
CRC8_INIT = 0xFF
CRC16_INIT = 0xFFFF

# 命令码定义
CMD_GAME_STATE = 0x0001        # 比赛状态数据
CMD_GAME_RESULT = 0x0002       # 比赛结果数据
CMD_GAME_ROBOT_HP = 0x0003     # 机器人血量数据

CMD_FIELD_EVENTS = 0x0101      # 场地事件数据
CMD_DART_REMAINING = 0x0105    # 飞镖发射口倒计时

CMD_ROBOT_STATE = 0x0201       # 机器人性能体系数据 (重点!)
CMD_POWER_HEAT = 0x0202        # 实时功率热量数据 (重点!)
CMD_ROBOT_POS = 0x0203         # 机器人位置数据
CMD_BUFF_MUSK = 0x0204         # 机器人增益和底盘能量数据
CMD_ROBOT_HURT = 0x0206        # 伤害状态数据
CMD_SHOOT_DATA = 0x0207        # 实时射击数据
CMD_BULLET_REMAINING = 0x0208  # 子弹剩余发射数
CMD_RFID_STATUS = 0x0209       # 机器人RFID状态


def crc8_calc(data):
    """计算CRC8校验值 (与官方协议一致)"""
    crc = CRC8_INIT
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc


def crc16_calc(data):
    """计算CRC16校验值 (与官方协议一致)"""
    crc = CRC16_INIT
    for byte in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]) & 0xFFFF
    return crc


def build_frame(cmd_id, data, seq=0):
    """
    构建完整的裁判系统串口帧

    帧结构: [帧头5字节][命令码2字节][数据n字节][CRC16 2字节]
    帧头: SOF(1) + data_length(2) + seq(1) + CRC8(1)

    Args:
        cmd_id: 命令码 (uint16)
        data: 数据域 (bytes)
        seq: 包序号 (默认0)

    Returns:
        完整帧数据 (bytes)
    """
    data_len = len(data)

    # 构建帧头 (不含CRC8)
    header = bytes([HEADER_SOF]) + struct.pack('<H', data_len) + bytes([seq])

    # 计算帧头CRC8
    crc8 = crc8_calc(header)

    # 完整帧头
    frame_header = header + bytes([crc8])

    # 命令码 (小端)
    cmd_bytes = struct.pack('<H', cmd_id)

    # 组合数据 (帧头 + 命令码 + 数据域)
    frame_without_crc = frame_header + cmd_bytes + data

    # 计算CRC16 (整包校验)
    crc16 = crc16_calc(frame_without_crc)
    crc_bytes = struct.pack('<H', crc16)

    # 完整帧
    return frame_without_crc + crc_bytes


def frame_to_hex_string(frame):
    """将帧转换为十六进制字符串 (用于串口调试助手)"""
    return ' '.join(f'{b:02X}' for b in frame)


# ================== 数据帧生成函数 ==================

def generate_game_state(game_type=1, game_progress=4, remain_time=180, timestamp=0):
    """
    生成0x0001比赛状态数据帧

    Args:
        game_type: 比赛类型 (1=RMUC, 2=RMUT, 3=RMUA, 4=RMUL 3V3, 5=RMUL 1V1)
        game_progress: 比赛阶段 (0=未开始, 1=准备, 2=自检, 3=5秒倒计时, 4=比赛中, 5=结算)
        remain_time: 剩余时间(秒)
        timestamp: UNIX时间戳
    """
    # bit 0-3: 比赛类型, bit 4-7: 比赛阶段
    byte0 = (game_type & 0x0F) | ((game_progress & 0x0F) << 4)
    data = bytes([byte0]) + struct.pack('<H', remain_time) + struct.pack('<Q', timestamp)
    return build_frame(CMD_GAME_STATE, data)


def generate_robot_state(robot_id=7, robot_level=1, current_hp=350, max_hp=350,
                          shooter_cooling=50, shooter_heat_limit=400,
                          chassis_power_limit=80, gimbal_out=1, chassis_out=1, shooter_out=1):
    """
    生成0x0201机器人性能体系数据帧 (底盘功率上限、热量等)

    Args:
        robot_id: 机器人ID (1=红英雄, 2=红工程, 3-5=红步兵, 6=红空中, 7=红哨兵, 101+=蓝方)
        robot_level: 机器人等级 (1-3)
        current_hp: 当前血量
        max_hp: 最大血量
        shooter_cooling: 射击热量冷却值
        shooter_heat_limit: 射击热量上限
        chassis_power_limit: 底盘功率上限 (W)
        gimbal_out: 云台输出 (0/1)
        chassis_out: 底盘输出 (0/1)
        shooter_out: 发射输出 (0/1)
    """
    data = bytes([
        robot_id,
        robot_level
    ])
    data += struct.pack('<H', current_hp)
    data += struct.pack('<H', max_hp)
    data += struct.pack('<H', shooter_cooling)
    data += struct.pack('<H', shooter_heat_limit)
    data += struct.pack('<H', chassis_power_limit)

    # 电源管理输出状态
    power_output = (gimbal_out & 0x01) | ((chassis_out & 0x01) << 1) | ((shooter_out & 0x01) << 2)
    data += bytes([power_output])

    return build_frame(CMD_ROBOT_STATE, data)


def generate_power_heat_data(chassis_volt=2400, chassis_current=5000,
                              chassis_power=120.0, buffer_energy=60,
                              shooter1_heat=10, shooter2_heat=20, shooter_42mm_heat=0):
    """
    生成0x0202实时功率热量数据帧 (缓冲能量、枪管热量)

    Args:
        chassis_volt: 底盘电压 (mV * 100, 如2400=24V)
        chassis_current: 底盘电流 (mA * 100, 如5000=50A)
        chassis_power: 底盘功率 (W)
        buffer_energy: 缓冲能量 (J)
        shooter1_heat: 17mm发射机构1热量
        shooter2_heat: 17mm发射机构2热量
        shooter_42mm_heat: 42mm发射机构热量
    """
    data = struct.pack('<H', chassis_volt)
    data += struct.pack('<H', chassis_current)
    data += struct.pack('<f', chassis_power)
    data += struct.pack('<H', buffer_energy)
    data += struct.pack('<H', shooter1_heat)
    data += struct.pack('<H', shooter2_heat)
    data += struct.pack('<H', shooter_42mm_heat)

    return build_frame(CMD_POWER_HEAT, data)


def generate_robot_pos(x=1.0, y=2.0, z=0.0, yaw=90.0):
    """
    生成0x0203机器人位置数据帧

    Args:
        x, y, z: 坐标 (米)
        yaw: 朝向角度 (度)
    """
    data = struct.pack('<ffff', x, y, z, yaw)
    return build_frame(CMD_ROBOT_POS, data)


def generate_buff_musk(recovery=0, cooling=0, defence=0, vulnerability=0,
                        attack=0, remaining_energy=0):
    """
    生成0x0204机器人增益和底盘能量数据帧

    Args:
        recovery: 回血增益
        cooling: 冷却增益
        defence: 防御增益
        vulnerability: 脆弱增益
        attack: 攻击增益
        remaining_energy: 底盘剩余能量
    """
    data = bytes([recovery, cooling, defence, vulnerability])
    data += struct.pack('<H', attack)
    data += bytes([remaining_energy])
    return build_frame(CMD_BUFF_MUSK, data)


def generate_robot_hurt(armor_type=0, hurt_type=0):
    """
    生成0x0206伤害状态数据帧

    Args:
        armor_type: 装甲板ID (0-3)
        hurt_type: 伤害类型 (0=装甲板, 1=模块离线, 2=超射速, 3=超热量, 4=超功率, 5=撞击)
    """
    byte0 = (armor_type & 0x0F) | ((hurt_type & 0x0F) << 4)
    return build_frame(CMD_ROBOT_HURT, bytes([byte0]))


def generate_shoot_data(bullet_type=1, shooter_id=1, bullet_freq=1, bullet_speed=15.0):
    """
    生成0x0207实时射击数据帧

    Args:
        bullet_type: 弹丸类型 (1=17mm, 2=42mm)
        shooter_id: 发射机构ID
        bullet_freq: 发射频率
        bullet_speed: 弹丸速度 (m/s)
    """
    data = bytes([bullet_type, shooter_id, bullet_freq])
    data += struct.pack('<f', bullet_speed)
    return build_frame(CMD_SHOOT_DATA, data)


def generate_bullet_remaining(bullet_17mm=100, bullet_42mm=10, coin=100):
    """
    生成0x0208子弹剩余发射数数据帧

    Args:
        bullet_17mm: 17mm弹丸剩余
        bullet_42mm: 42mm弹丸剩余
        coin: 剩余金币
    """
    data = struct.pack('<HHH', bullet_17mm, bullet_42mm, coin)
    return build_frame(CMD_BULLET_REMAINING, data)


# ================== 主程序 ==================

def print_menu():
    print("\n" + "="*60)
    print("    RoboMaster 2026 裁判系统串口帧生成工具")
    print("    协议版本: V1.2.0 (20260209)")
    print("="*60)
    print("\n可选帧类型:")
    print("  1. 0x0001 - 比赛状态数据")
    print("  2. 0x0201 - 机器人性能体系数据 (底盘功率上限等) ★推荐")
    print("  3. 0x0202 - 实时功率热量数据 (缓冲能量、枪管热量) ★推荐")
    print("  4. 0x0203 - 机器人位置数据")
    print("  5. 0x0204 - 机器人增益和底盘能量数据")
    print("  6. 0x0206 - 伤害状态数据")
    print("  7. 0x0207 - 实时射击数据")
    print("  8. 0x0208 - 子弹剩余发射数")
    print("  9. 自定义数据帧")
    print("  0. 退出")
    print("-"*60)


def main():
    while True:
        print_menu()
        choice = input("\n请选择帧类型 (输入数字): ").strip()

        if choice == '0':
            print("退出程序")
            break

        elif choice == '1':
            print("\n--- 0x0001 比赛状态数据 ---")
            frame = generate_game_state(game_type=1, game_progress=4, remain_time=180)

        elif choice == '2':
            print("\n--- 0x0201 机器人性能体系数据 ---")
            print("默认值: 红方哨兵7号, 1级, HP=350, 底盘功率=80W")
            frame = generate_robot_state(
                robot_id=7,
                robot_level=1,
                current_hp=350,
                max_hp=350,
                shooter_cooling=50,
                shooter_heat_limit=400,
                chassis_power_limit=80,
                gimbal_out=1,
                chassis_out=1,
                shooter_out=1
            )

        elif choice == '3':
            print("\n--- 0x0202 实时功率热量数据 ---")
            print("默认值: 24V, 50A, 120W, 缓冲能量=60J")
            frame = generate_power_heat_data(
                chassis_volt=2400,
                chassis_current=5000,
                chassis_power=120.0,
                buffer_energy=60,
                shooter1_heat=10,
                shooter2_heat=20,
                shooter_42mm_heat=0
            )

        elif choice == '4':
            print("\n--- 0x0203 机器人位置数据 ---")
            frame = generate_robot_pos(x=1.0, y=2.0, z=0.0, yaw=90.0)

        elif choice == '5':
            print("\n--- 0x0204 机器人增益和底盘能量数据 ---")
            frame = generate_buff_musk()

        elif choice == '6':
            print("\n--- 0x0206 伤害状态数据 ---")
            frame = generate_robot_hurt(armor_type=0, hurt_type=0)

        elif choice == '7':
            print("\n--- 0x0207 实时射击数据 ---")
            frame = generate_shoot_data(bullet_type=1, shooter_id=1, bullet_freq=1, bullet_speed=15.0)

        elif choice == '8':
            print("\n--- 0x0208 子弹剩余发射数 ---")
            frame = generate_bullet_remaining(bullet_17mm=100, bullet_42mm=10, coin=100)

        elif choice == '9':
            print("\n--- 自定义数据帧 ---")
            cmd = int(input("输入命令码 (如0x0201输入521): ") or "513", 16)
            hex_data = input("输入数据 (十六进制, 空格分隔, 如: 07 01 5E 01): ").strip()
            if hex_data:
                data = bytes([int(x, 16) for x in hex_data.split()])
            else:
                data = b''
            frame = build_frame(cmd, data)

        else:
            print("无效选择!")
            continue

        # 输出结果
        print("\n" + "="*60)
        print("生成的帧数据 (用于串口调试助手发送):")
        print("-"*60)
        print(frame_to_hex_string(frame))
        print("-"*60)
        print(f"帧长度: {len(frame)} 字节")
        print("="*60)

        # 显示帧结构解析
        print("\n帧结构解析:")
        print(f"  帧头: {frame[0]:02X} {frame[1]:02X} {frame[2]:02X} {frame[3]:02X} {frame[4]:02X}")
        print(f"    SOF = 0x{frame[0]:02X}")
        print(f"    data_length = {frame[1] | (frame[2] << 8)}")
        print(f"    seq = {frame[3]}")
        print(f"    CRC8 = 0x{frame[4]:02X}")
        print(f"  命令码: {frame[5]:02X} {frame[6]:02X} (0x{frame[5] | (frame[6] << 8):04X})")
        data_len = len(frame) - 9  # 总长度减去帧头5、命令码2、CRC16 2
        if data_len > 0:
            print(f"  数据域 ({data_len}字节): {' '.join(f'{b:02X}' for b in frame[7:7+data_len])}")
        print(f"  CRC16: {frame[-2]:02X} {frame[-1]:02X}")


if __name__ == "__main__":
    main()