from pymodbus.client import ModbusSerialClient
import struct

class LabTempController:
    def __init__(self, port, baudrate=19200, timeout=1):
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

    def set_temperature(self, temperature):
        """Set the temperature of the device.
        
        Args:
            temperature (float): The desired temperature (in degrees Celsius).
        """
        if not self.connected:
            print("Not connected to the device.")
            return
        
        # Scale the temperature by 100 as per the protocol
        scaled_temp = int(temperature * 100)
        
        # Prepare the write command
        result = self.client.write_register(2, scaled_temp, unit=1)
        if result.function_code < 0x80:
            print(f"Temperature set to {temperature}°C")
        else:
            print("Error setting temperature.")

    def read_temperature(self):
        """Read the current temperature from the device.
        
        Returns:
            float: The current temperature (in degrees Celsius).
        """
        if not self.connected:
            print("Not connected to the device.")
            return None
        
        # Read the temperature register
        response = self.client.read_holding_registers(0, count=1, unit=1)
        if not response.isError():
            # Scale the temperature back down by 100
            temp = response.registers[0] / 100.0
            print(f"Current temperature: {temp}°C")
            return temp
        else:
            print("Error reading temperature.")
            return None
        
    def set_stirrer_mode(self, mode):
        if not self.connected:
            print("Not connected to the device.")
            return
        # 寄存器地址为 108
        result = self.client.write_register(108, mode, unit=1)
        if result.function_code < 0x80:
            print(f"Stirrer mode set to {'independent' if mode == 1 else 'non-independent'}")
        else:
            print("Error setting stirrer mode.")

    def set_stirrer_channel_count(self, count):
        if not self.connected:
            print("Not connected to the device.")
            return
        # 寄存器地址为 109
        result = self.client.write_register(109, count, unit=1)
        if result.function_code < 0x80:
            print(f"Stirrer channel count set to {count}")
        else:
            print("Error setting stirrer channel count.")

    def set_stirrer_speed(self, channel, speed):
        if not self.connected:
            print("Not connected to the device.")
            return

        # 寄存器地址根据通道号确定
        address = 144 + (channel - 1)

        # 写入速度值
        result = self.client.write_register(address, speed, unit=1)
        if result.function_code < 0x80:
            print(f"Speed set to {speed} for channel {channel}")
        else:
            print("Error setting speed.")

        
    def set_stirrer_speed(self, channel, speed):
        if not self.connected:
            print("Not connected to the device.")
            return

        # 寄存器地址根据通道号确定
        address = 144 + (channel - 1)

        # 写入速度值
        result = self.client.write_register(address, speed, unit=1)
        if result.function_code < 0x80:
            print(f"Speed set to {speed} for channel {channel}")
        else:
            print("Error setting speed.")

if __name__ == '__main__':
    # Create a client instance
    labtemp_controller = LabTempController(port='COM1')
    
    # Connect to the device
    if labtemp_controller.connect():
        # Set the temperature
        labtemp_controller.set_temperature(25.5)
        
        # Read the current temperature
        labtemp_controller.read_temperature()
        
        # Disconnect from the device
        labtemp_controller.disconnect()