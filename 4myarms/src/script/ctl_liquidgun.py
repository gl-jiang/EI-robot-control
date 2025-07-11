import serial
import struct

def cal_crc(bytearray, nbyte):
    itemp = 0xFFFF
    while nbyte > 0:
        itemp ^= bytearray[0]
        bytearray = bytearray[1:]
        for _ in range(8):
            if itemp & 0x1:
                itemp >>= 1
                itemp ^= 0xA001
            else:
                itemp >>= 1
        nbyte -= 1
    return itemp

class PipetteController:
    def __init__(self, device_id=1, port='COM1'):
        self.device_id = device_id
        self.port = port
        self.baudrate = 115200
        self.bytesize = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 1  # Set timeout to 1 second

        self.serial_connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            stopbits=self.stopbits,
            timeout=self.timeout
        )

    def _send_command(self, command, data=b''):
        crc = cal_crc((f'>{self.device_id}{command}{data.hex().upper()}'.encode()), len(f'>{self.device_id}{command}{data.hex().upper()}'))
        message = f'>{self.device_id}{command}{data.hex().upper()}{crc:04X}'.encode()
        print(f'Sending: {message.decode()}')
        self.serial_connection.write(message + b'\r\n')  # Add end-of-line characters

    def _read_tip_specification(self):
        self._send_command('o')

    def _set_tip_specification(self, tip_volume, has_filter):
        # Assuming tip_volume is in ul and has_filter is a boolean
        tip_data = struct.pack('<I', (tip_volume << 1) | (1 if has_filter else 0))
        self._send_command('O', tip_data)

    def aspirate(self, volume_ul):
        # Assuming volume_ul is the volume in ul to aspirate
        aspirate_data = struct.pack('<I', volume_ul)
        self._send_command('n', aspirate_data)

    def dispense(self, volume_ul):
        # Assuming volume_ul is the volume in ul to dispense
        dispense_data = struct.pack('<I', volume_ul)
        self._send_command('p', dispense_data)

    def eject_tip(self):
        self._send_command('Q')

# Example usage
controller = PipetteController(port='COM3')  # Change COM3 to your actual serial port
controller._read_tip_specification()
controller._set_tip_specification(1000, True)
controller.aspirate(500)
controller.dispense(200)
controller.eject_tip()