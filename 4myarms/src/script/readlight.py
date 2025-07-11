import time
import pymodbus
from pymodbus.client import ModbusSerialClient as ModbusClient
from pymodbus.exceptions import ModbusIOException
from pymodbus.transaction import ModbusRtuFramer
import time
from pymodbus.client import ModbusSerialClient
import struct

class LightModbusRTUClient:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            stopbits=1,
            bytesize=8,
            parity='N',
            baudrate=baudrate,
            timeout=timeout
        )
        self.connected = False

    def connect(self):
        if not self.connected:
            if self.client.connect():
                self.connected = True
                print("Connected to the device.")
            else:
                print("Failed to connect to the device.")
        return self.connected

    def disconnect(self):
        if self.connected:
            self.client.close()
            self.connected = False
            print("Disconnected from the device.")

    def read_spectrum_data(self, start_address):
        if self.connected:
            # 读取两个寄存器
            response = self.client.read_holding_registers(start_address, 2, unit=1)
            if not response.isError():
                # 将两个 16 位整数转换为 32 位浮点数
                value = struct.unpack('>f', bytes(response.registers))[0]
                return value
            else:
                print("Error reading registers:", response)
                return None
        else:
            print("Not connected to the device.")
            return None

    def read_all_spectrum_data(self):
        if self.connected:
            results = []
            for i in range(672):  # 假设您需要读取 672 组数据
                address = 0xA000 + i * 2  # 根据数据定义计算起始地址
                data = self.read_spectrum_data(address)
                if data is not None:
                    results.append(data)
                time.sleep(0.1)  # 等待一段时间以避免过快的读取速度
            return results
        else:
            print("Not connected to the device.")
            return []

if __name__ == '__main__':
    # 创建客户端实例
    modbus_client = LightModbusRTUClient 
      # 连接到设备
    if modbus_client.connect():
        # 读取所有光谱数据
        spectrum_data = modbus_client.read_all_spectrum_data()
        print(spectrum_data)

        # 断开连接
        modbus_client.disconnect()