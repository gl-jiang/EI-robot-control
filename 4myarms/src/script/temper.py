import serial
from readtemp import *


def read_target_temperature(serial_port, baud_rate):
    # 初始化串口
    ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=1)
    
    if not ser.is_open:
        ser.open()
    print("port open!")
    # 寄存器地址
    reg_addr = 0x0002
    
    # 功能码3用于读取保持寄存器
    func_code = 0x03
    
    # 事务处理标识符（通常可以固定）
    transaction_id = 0x0001
    
    # 协议标识符（对于MODBUS RTU总是0）
    protocol_id = 0x0000
    
    # 单元ID（设备ID）
    unit_id = 0x03  # 假设设备ID为1
    
    # 要读取的寄存器数量
    num_regs = 2  # 因为是32位有符号整数，需要读取2个寄存器
    
    # 构建MODBUS RTU请求报文
    request = bytes.fromhex(f'{transaction_id:04x}{protocol_id:04x}{func_code:02x}{unit_id:02x}{reg_addr:04x}{num_regs:04x}')

    #request = bytes.fromhex(f'{protocol_id:04x}{func_code:02x}{unit_id:02x}{reg_addr:04x}{num_regs:04x}')
  
    #b'\x03\x03\x00\x00\x00\02'

    request_pdu=b'\x03\x03\x00\x02\x00\x02\x64\x29'
    print(request)
    # 计算CRC校验码
    crc = crc16_temp(request)

    #print(crc)

    #request = request + crc.to_bytes(2, byteorder='big')

    print(request_pdu)
    # 发送请求
    ser.write(request_pdu)
    
    # 接收响应
    response = ser.read(5 + 2 * num_regs)  # 8字节头部信息加上2个寄存器的数据
    print("response")
    print(response.hex())
    print(len(response))
    print(5 + 2 * num_regs)
    data = response[3:7]
    print(data.hex())

    temp_32bit = int.from_bytes(data,byteorder='big', signed=True)
    temperature = temp_32bit / 100.0
        
    return temperature
    
   

# 使用示例
if __name__ == "__main__":
    serial_port = '/dev/serialTempreMem' # Windows下是COMx，Linux/Mac下是/dev/ttyUSBx或/dev/ttySx
    baud_rate = 57600      # 根据实际情况设置波特率
    temperature = read_target_temperature(serial_port, baud_rate)
    if temperature is not None:
        print(f"Target Temperature: {temperature} °C")