import serial

class SerialComsSingle():
    def __init__( self, dev_name = '/dev/ttyACM0', baudrate = 9600 ):
        self.baudrate = baudrate
        self.dev_name = dev_name
        self.serialCon = serial.Serial(self.dev_name, self.baudrate)

    def send_serial_msgs(self,  msg):
        self.serialCon.write(str(msg))





