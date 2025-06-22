#!/usr/bin/env python3
# coding: utf-8

import struct
import time
import serial
import threading


class Rosmaster:
    __uart_state = 0

    def __init__(self, car_type=1, com="/dev/myserial", delay=0.002, debug=False):
        self.ser = serial.Serial(com, 115200)

        self.__delay_time = delay
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_IMU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_SET_MOTOR_PID = 0x13

        self.__vx = self.__vy = self.__vz = 0
        self.__ax = self.__ay = self.__az = 0
        self.__gx = self.__gy = self.__gz = 0
        self.__mx = self.__my = self.__mz = 0

        self.__encoder_m1 = self.__encoder_m2 = 0
        self.__encoder_m3 = self.__encoder_m4 = 0
        self.__yaw = self.__roll = self.__pitch = 0
        self.__battery_voltage = 0

        self.__kp1 = self.__ki1 = self.__kd1 = 0
        self.__pid_index = 0

        if self.__debug:
            print(f"cmd_delay={self.__delay_time}s")

        if self.ser.isOpen():
            print("Rosmaster Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")

        time.sleep(0.002)

    def __del__(self):
        self.ser.close()
        self.__uart_state = 0
        print("Serial Closed!")

    def __parse_data(self, ext_type, ext_data):
        raw = bytearray(ext_data)

        if ext_type == self.FUNC_REPORT_SPEED:
            self.__vx = struct.unpack('<h', raw[0:2])[0] / 1000.0
            self.__vy = struct.unpack('<h', raw[2:4])[0] / 1000.0
            self.__vz = struct.unpack('<h', raw[4:6])[0] / 1000.0
            self.__battery_voltage = raw[6]

        elif ext_type == self.FUNC_REPORT_IMU_RAW:
            gyro_ratio = 1 / 3754.9
            accel_ratio = 1 / 1671.84
            self.__gx = struct.unpack('<h', raw[0:2])[0] * gyro_ratio
            self.__gy = -struct.unpack('<h', raw[2:4])[0] * gyro_ratio
            self.__gz = -struct.unpack('<h', raw[4:6])[0] * gyro_ratio
            self.__ax = struct.unpack('<h', raw[6:8])[0] * accel_ratio
            self.__ay = struct.unpack('<h', raw[8:10])[0] * accel_ratio
            self.__az = struct.unpack('<h', raw[10:12])[0] * accel_ratio
            self.__mx = struct.unpack('<h', raw[12:14])[0]
            self.__my = struct.unpack('<h', raw[14:16])[0]
            self.__mz = struct.unpack('<h', raw[16:18])[0]

        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            self.__roll = struct.unpack('<h', raw[0:2])[0] / 10000.0
            self.__pitch = struct.unpack('<h', raw[2:4])[0] / 10000.0
            self.__yaw = struct.unpack('<h', raw[4:6])[0] / 10000.0

        elif ext_type == self.FUNC_REPORT_ENCODER:
            self.__encoder_m1 = struct.unpack('<i', raw[0:4])[0]
            self.__encoder_m2 = struct.unpack('<i', raw[4:8])[0]
            self.__encoder_m3 = struct.unpack('<i', raw[8:12])[0]
            self.__encoder_m4 = struct.unpack('<i', raw[12:16])[0]

        elif ext_type == self.FUNC_SET_MOTOR_PID:
            self.__pid_index = raw[0]
            self.__kp1 = struct.unpack('<h', raw[1:3])[0]
            self.__ki1 = struct.unpack('<h', raw[3:5])[0]
            self.__kd1 = struct.unpack('<h', raw[5:7])[0]

    def __receive_data(self):
        while True:
            try:
                if not self.ser.is_open:
                    print("[Rosmaster] Serial port closed. Attempting to reconnect...")
                    self.ser.open()
                    time.sleep(1)

                header = self.ser.read()
                if not header or header[0] != self.__HEAD:
                    continue

                device_id = self.ser.read()
                if not device_id or device_id[0] != self.__DEVICE_ID - 1:
                    continue

                ext_len_bytes = self.ser.read()
                if not ext_len_bytes:
                    continue
                ext_len = ext_len_bytes[0]

                ext_type_bytes = self.ser.read()
                if not ext_type_bytes:
                    continue
                ext_type = ext_type_bytes[0]

                ext_data = []
                check_sum = ext_len + ext_type

                while len(ext_data) < (ext_len - 2):
                    val_bytes = self.ser.read()
                    if not val_bytes:
                        break
                    val = val_bytes[0]
                    ext_data.append(val)
                    if len(ext_data) != ext_len - 2:
                        check_sum += val

                if len(ext_data) != (ext_len - 2):
                    continue

                rx_check = ext_data[-1]
                if (check_sum % 256) == rx_check:
                    self.__parse_data(ext_type, ext_data)

            except serial.SerialException as e:
                print(f"[Rosmaster] SerialException: {e}. Retrying in 2s...")
                try:
                    self.ser.close()
                except:
                    pass
                time.sleep(2)
                try:
                    self.ser = serial.Serial(self.ser.port, self.ser.baudrate, timeout=0.1)
                    print("[Rosmaster] Reconnected.")
                except Exception as e2:
                    print(f"[Rosmaster] Reconnect failed: {e2}")
                    time.sleep(2)
            except Exception as e:
                print(f"[Rosmaster] Unexpected error in receive thread: {e}")
                time.sleep(1)


    def create_receive_threading(self):
        if self.__uart_state == 0:
            task = threading.Thread(target=self.__receive_data, name="serial_rx")
            task.setDaemon(True)
            task.start()
            print("Started receive thread")
            self.__uart_state = 1

    def set_auto_report_state(self, enable=True, forever=False):
        state1 = 1 if enable else 0
        state2 = 0x5F if forever else 0x00
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
        checksum = (sum(cmd) + self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(bytearray(cmd))
        if self.__debug:
            print("Auto report:", cmd)
        time.sleep(self.__delay_time)

    def set_beep(self, duration_ms):
        value = struct.pack('<h', duration_ms)
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
        checksum = (sum(cmd) + self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(bytearray(cmd))
        if self.__debug:
            print("Beep:", cmd)
        time.sleep(self.__delay_time)

    def set_motor(self, s1, s2, s3, s4):
        speeds = [self.__limit_pwm(s) for s in (s1, s2, s3, s4)]
        speed_bytes = [struct.pack('b', s)[0] for s in speeds]
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR] + speed_bytes
        cmd[2] = len(cmd) - 1 
        checksum = (sum(cmd) + self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(bytearray(cmd))
        if self.__debug:
            print("Motor:", cmd)
        time.sleep(self.__delay_time)

    def set_pid_param(self, kp, ki, kd, forever=False):
        if not (0 <= kp <= 10 and 0 <= ki <= 10 and 0 <= kd <= 10):
            print("PID values must be in range [0, 10.00]")
            return
        state = 0x5F if forever else 0
        kp_bytes = struct.pack('<h', int(kp * 1000))
        ki_bytes = struct.pack('<h', int(ki * 1000))
        kd_bytes = struct.pack('<h', int(kd * 1000))
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID,
               kp_bytes[0], kp_bytes[1], ki_bytes[0], ki_bytes[1],
               kd_bytes[0], kd_bytes[1], state]
        checksum = (sum(cmd) + self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(bytearray(cmd))
        if self.__debug:
            print("Set PID:", cmd)
        time.sleep(self.__delay_time)
        if forever:
            time.sleep(0.1)

    def get_motion_pid(self):
        self.__kp1 = self.__ki1 = self.__kd1 = self.__pid_index = 0
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, 0x50, self.FUNC_SET_MOTOR_PID, 1]
        checksum = (sum(cmd) + self.__COMPLEMENT) & 0xFF
        cmd.append(checksum)
        self.ser.write(bytearray(cmd))
        for _ in range(20):
            if self.__pid_index > 0:
                return [self.__kp1 / 1000.0, self.__ki1 / 1000.0, self.__kd1 / 1000.0]
            time.sleep(0.001)
        return [-1, -1, -1]

    def __limit_pwm(self, val):
        return max(-100, min(100, int(val)))

    def get_accelerometer_data(self):
        return self.__ax, self.__ay, self.__az

    def get_gyroscope_data(self):
        return self.__gx, self.__gy, self.__gz

    def get_magnetometer_data(self):
        return self.__mx, self.__my, self.__mz

    def get_imu_attitude_data(self):
        RtA = 57.2957795
        return self.__roll * RtA, self.__pitch * RtA, self.__yaw * RtA

    def get_motion_data(self):
        return self.__vx, self.__vy, self.__vz

    def get_battery_voltage(self):
        return self.__battery_voltage / 10.0

    def get_motor_encoder(self):
        return self.__encoder_m1, self.__encoder_m2, self.__encoder_m3, self.__encoder_m4
