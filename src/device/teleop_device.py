from abc import abstractmethod
import pose_openvr_wrapper
from serial import Serial
import struct
import numpy as np

class TeleoperationDevice:
    @abstractmethod
    def get_hand_pose():
        pass

    def get_gripper_state():
        return None
    
    def close(self):
        pass

class ViveTracker(TeleoperationDevice):

    def __init__(self, tracker, serial_port):
        """
        Initializes connection to VR Controller and gripper control device.
        """
        # Vive tracker initialization
        self.device_manager = pose_openvr_wrapper.OpenvrWrapper('config.json')
        self.tracker = tracker
        if self.tracker not in self.device_manager.devices.keys():
                raise RuntimeError(f'Hand tracker not connected to PC!')
        
        # gripper controller initialization
        self.serial_port = Serial(serial_port, 230400, timeout=1)
        if not self.serial_port.isOpen():
            raise RuntimeError('Gripper controller is not connected to COM port!')

    def get_device_pose(self):
        """
        Retrieves current position and orientation of device.
        Returns:
            np.array: A list of 7 values (x, y, z, qx, qy, qz ,qw) 

        """
        hand_pose = self.device_manager.sample(self.tracker, samples_count=1)
        return np.array([hand_pose["x"][0], hand_pose["y"][0], hand_pose["z"][0], 
                         hand_pose["r_x"][0], hand_pose["r_y"][0], hand_pose["r_z"][0], hand_pose["r_w"][0]])



    def get_gripper_state(self):
        """
        Retrieves target gripper position.
        Returns:
            int: value in range (0..255).
        """
        flag_message_error = False
        package = bytearray(b"i")
        self.port.flushInput()
        self.port.write(package)
        # Получаем данные из порта
        data = self.port.read(8)
        # Проверяем, что данные получены и их размер равен 8 байт
        if data and len(data) == 8:
            teleop_position = np.frombuffer(data[:4], dtype=np.uint32)[0]
            CRC_STM = np.frombuffer(data[4:], dtype=np.uint32)[0]
            # high-endian это очень важно (!)
            bytes_for_crc = bytearray(struct.pack(">I", teleop_position))
            crc32_python = self.__crc32mpeg2(bytes_for_crc)
            if crc32_python != CRC_STM:
                flag_message_error = True
            else:
                teleop_position = round(teleop_position / 3)
                if teleop_position >= 255:
                    teleop_position=254
                if teleop_position == 33:
                    teleop_position=0
                return teleop_position
        else:
            flag_message_error = True
            teleop_position=0
            return teleop_position
        
    def __crc32mpeg2(data):
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte << 24
            for _ in range(8):
                if crc & 0x80000000:
                    crc = (crc << 1) ^ 0x04C11DB7
                else:
                    crc <<= 1
                crc &= 0xFFFFFFFF
        return crc

