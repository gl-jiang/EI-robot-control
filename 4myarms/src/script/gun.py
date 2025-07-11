import serial

class SerialController:
    def __init__(self, port='/dev/serialLqGun', baudrate=115200, timeout=1):
        # 初始化串口参数
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        if not self.ser.is_open:
            raise IOError("Serial port %s is not open." % port)

    def send_command(self, command):
        # 发送命令到串口
        if isinstance(command, str):
            command = command.encode('ascii')  # 确保是ASCII编码的字节串
        self.ser.write(command + b'\n')  # 假设每个命令以换行符结尾
        response = self.ser.readline().decode('utf-8').strip()
        return response

    def send_command_one(self): #iniital
        # 发送第一个命令
        print("Sending command one...")
        #for new gun can't use normal init because tip lose 2025 0102
        print("send: " + ">01p000061AC")
        #response = self.send_command('>01p000061AC')
        response = self.send_command('>01G6158') 
        print("Response: " + response)

    def send_command_two(self): #air init
        # 发送第二个命令
        print("Sending command two...")
        print("send: " + ">01M66D8")
        response = self.send_command('>01M66D8')
        print("Response: " + response)

    def send_command_three(self): # liquid 3ml 
        # 发送第三个命令
        print("Sending command three...3ml")
        response = self.send_command('>01n0BB81E81')
        print("Response: " + response)

    def send_command_three_08(self): # liquid#0.8
        # 发送第三个命令
        print("Sending command three...0.8ml")
        print("send: " + ">01n032003F5")
        response = self.send_command('>01n032003F5')
        print("Response: " + response)

    def send_command_three_12(self): # liquid#1.2
        # 发送第三个命令
        print("Sending command three...1.2ml")
        response = self.send_command('>01n04B00261')
        print("Response: " + response)

    def send_command_three_45(self): # liquid #4.5
        # 发送第三个命令
        print("Sending command three...4.5ml")
        response = self.send_command('>01n1194CC53')
        print("Response: " + response)

    def send_command_four(self): # tu liquid 
        # 发送第三个命令
        print("Sending command four...")
        print("send: " + ">01p000061AC")
        response = self.send_command('>01p000061AC')
        print("Response: " + response)

    
    def send_command_setSpeed(self,spd): # tu liquid 
        # 发送第三个命令
        print("Sending command set spd...")
        #print("send: " + ">01p000061AC")
        cmd=""
        if spd==24000:
            cmd=">01B5DC043F0"
            response = self.send_command(cmd)
        elif spd==1000:
            cmd=">01B03E8F342"
            response = self.send_command(cmd)
        elif spd==500:
            cmd=">01B01F4C6E3"
            response = self.send_command(cmd)
        else:
            print("unsupport speed!")
        print("Response: " + response)

    def close(self):
        # 关闭串口
        self.ser.close()