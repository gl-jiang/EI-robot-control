import serial

def clawop_crc(data):
        # 设置串口参数
    port = '/dev/serialclaw1'
    baudrate = 115200  # 根据你的硬件需求设置波特率

    datawcrc = append_crc16_to_data(data)
    print(datawcrc)
    try:
        # 打开串口
        ser = serial.Serial(port, baudrate)
        
        # 确保串口已打开
        if ser.is_open:
            # 发送数据
            ser.write(datawcrc)
            
            # 关闭串口
            ser.close()
    except serial.SerialException as e:
        print(f"Serial Exception: {e}")
    except Exception as e:
        print(f"General Exception: {e}")



def clawop(data):
        # 设置串口参数
    port = '/dev/serialclaw1'
    baudrate = 115200  # 根据你的硬件需求设置波特率

    try:
        # 打开串口
        ser = serial.Serial(port, baudrate)
        
        # 确保串口已打开
        if ser.is_open:
            # 发送数据
            ser.write(data)
            
            # 关闭串口
            ser.close()
    except serial.SerialException as e:
        print(f"Serial Exception: {e}")
    except Exception as e:
        print(f"General Exception: {e}")


import time

def rotswithch(data):
        # 设置串口参数
    port = '/dev/serialclaw1'
    baudrate = 115200  # 根据你的硬件需求设置波特率

  #  datawcrc = append_crc16_to_data(data)

    try:
        # 打开串口
        ser = serial.Serial(port, baudrate)
        
        # 确保串口已打开
        if ser.is_open:
            # 发送数据
            #ser.write(data)
             # 发送字符串
            ser.write(data.encode('ascii'))  # 将字符串编码为字节串发送

              # 给设备一些时间来处理命令
            time.sleep(0.1)  # 这个延迟取决于你的设备需要多少时间来响应
            
            # 读取响应
            response = ser.read(ser.in_waiting or 1)  # 读取所有可用的字节或至少一个字节
            print(f"Received: {response.decode('ascii')}")  # 解码并打印接收到的数据
          

            
            # 关闭串口
            ser.close()
    except serial.SerialException as e:
        print(f"Serial Exception: {e}")
    except Exception as e:
        print(f"General Exception: {e}")



def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x8005
            else:
                crc <<= 1
        crc &= 0xFFFF
    print(f"CRC-16-Modbus: {crc:04X}")
    return crc


def append_crc16_to_data(data: bytes) -> bytes:
    """
    Appends a 16-bit CRC calculated with polynomial 0x8005 to the end of the given data.
    
    :param data: The input data to which the CRC will be appended.
    :return: The data with the CRC appended as two additional bytes.
    """
    crc = crc16_modbus(data)
    # Convert the CRC result into bytes
    crc_bytes = crc.to_bytes(2, byteorder='little')  # Use 'big' if your system uses big-endian
    # Append the CRC bytes to the original data
    return data + crc_bytes


# 准备要发送的数据
data = b'\x01\x06\x01\x05\x00\x32\x19\xe2'  # 示例字节序列

#01 06 01 05003219E2

# 调用函数发送数据
#send_bytes_to_serial(port, baudrate, data)