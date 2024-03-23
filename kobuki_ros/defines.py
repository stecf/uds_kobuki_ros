from typing import List

class TRawGyroData:
    def __init__(self, x=0, y=0, z=0):
        self.x = x  # unsigned short
        self.y = y  # unsigned short
        self.z = z  # unsigned short

class TExtraRequestData:
    def __init__(self):
        self.HardwareVersionMajor = 0  # unsigned char
        self.HardwareVersionMinor = 0  # unsigned char
        self.HardwareVersionPatch = 0  # unsigned char
        self.FirmwareVersionMajor = 0  # unsigned char
        self.FirmwareVersionMinor = 0  # unsigned char
        self.FirmwareVersionPatch = 0  # unsigned char
        self.UDID0 = 0  # unsigned int
        self.UDID1 = 0  # unsigned int
        self.UDID2 = 0  # unsigned int
        self.PIDtype = 0  # unsigned char
        self.PIDgainP = 0  # unsigned int
        self.PIDgainI = 0  # unsigned int
        self.PIDgainD = 0  # unsigned int

class TKobukiData:
    def __init__(self):
        self.timestamp = 0  # unsigned short
        self.BumperLeft = False  # bool
        self.BumperCenter = False  # bool
        self.BumperRight = False  # bool
        self.CliffLeft = False  # bool
        self.CliffCenter = False  # bool
        self.CliffRight = False  # bool
        self.WheelDropLeft = False  # bool
        self.WheelDropRight = False  # bool
        self.EncoderRight = 0  # unsigned short
        self.EncoderLeft = 0  # unsigned short
        self.PWMright = 0  # unsigned char
        self.PWMleft = 0  # unsigned char
        self.ButtonPress = 0  # unsigned char
        self.Charger = 0  # unsigned char
        self.Battery = 0  # unsigned char
        self.overCurrent = 0  # unsigned char
        self.IRSensorRight = 0  # unsigned char
        self.IRSensorCenter = 0  # unsigned char
        self.IRSensorLeft = 0  # unsigned char
        self.GyroAngle = 0  # signed short
        self.GyroAngleRate = 0  # unsigned short
        self.CliffSensorRight = 0  # unsigned short
        self.CliffSensorCenter = 0  # unsigned short
        self.CliffSensorLeft = 0  # unsigned short
        self.wheelCurrentLeft = 0  # unsigned char
        self.wheelCurrentRight = 0  # unsigned char
        self.frameId = 0  # unsigned char
        self.gyroData = []  # List of TRawGyroData
        self.digitalInput = 0  # unsigned short
        self.analogInputCh0 = 0  # unsigned short
        self.analogInputCh1 = 0  # unsigned short
        self.analogInputCh2 = 0  # unsigned short
        self.analogInputCh3 = 0  # unsigned short
        self.extraInfo = TExtraRequestData()  # TExtraRequestData

class LaserData:
    def __init__(self):
        self.scanQuality: int = 0
        self.scanAngle: float = 0.0
        self.scanDistance: float = 0.0
        
    def __str__(self):
        return f'Quality: {self.scanQuality}, Angle: {self.scanAngle}, Distance: {self.scanDistance}'

class TLaserMeasurement:
    def __init__(self):
        self.numberOfScans: int = 0
        self.Data: List[LaserData] = []


# Define the IP address and port number
UDP_IP = "127.0.0.1"  # Loopback address for local testing
ROBOT_UDP_PORT = 5300
LIDAR_UDP_PORT = 5299
