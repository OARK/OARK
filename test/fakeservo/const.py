#Contains important constants needed for interpreting AX12
#packets.


WHEEL_MODE = 0
POSITION_MODE = 1


#Instruction packets
INST_PING = 0x01
INST_READDATA = 0x02
INST_WRITEDATA = 0x03
INST_REGWRITE = 0x04
INST_ACTION = 0x05
INST_RESET = 0x06
INST_SYNCWRITE = 0x83


#Addresses
ADDR_MODELNUMBER_L = 0x0
ADDR_MODELNUMBER_H = 0x1
ADDR_VERSIONOFFIRMWARE = 0x2
ADDR_ID = 0x3
ADDR_BAUDRATE = 0x4
ADDR_RETURNDELAYTIME = 0x5
ADDR_CWANGLELIMIT_L = 0x6
ADDR_CWANGLELIMIT_H = 0x7
ADDR_CCWANGLELIMIT_L = 0x8
ADDR_CCWANGLELIMIT_H = 0x9
ADDR_HIGHESTLIMITTEMPERATURE = 0xB
ADDR_LOWESTLIMITVOLTAGE = 0xC
ADDR_HIGHESTLIMITVOLTAGE = 0xD
ADDR_MAXTORQUE_L = 0xE
ADDR_MAXTORQUE_H = 0xF
ADDR_STATUSRETURNLEVEL = 0x10
ADDR_ALARMLED = 0x11
ADDR_ALARMSHUTDOWN = 0x12
ADDR_TORQUEENABLE = 0x18
ADDR_LED = 0x19
ADDR_CWCOMPLIANCEMARGIN = 0x1A
ADDR_CCWCOMPLIANCEMARGIN = 0x1B
ADDR_CWCOMPLIANCESLOPE = 0x1C
ADDR_CCWCOMPLIANCESLOPE = 0x1D
ADDR_GOALPOSITION_L = 0x1E
ADDR_GOALPOSITION_H = 0x1F
ADDR_MOVINGSPEED_L = 0x20
ADDR_MOVINGSPEED_H = 0x21
ADDR_TORQUELIMIT_L = 0x22
ADDR_TORQUELIMIT_H = 0x23
ADDR_PRESENTPOSITION_L = 0x24
ADDR_PRESENTPOSITION_H = 0x25
ADDR_PRESENTSPEED_L = 0x26
ADDR_PRESENTSPEED_H = 0x27
ADDR_PRESENTLOAD_L = 0x28
ADDR_PRESENTLOAD_H = 0x29
ADDR_PRESENTVOLTAGE = 0x2A
ADDR_PRESENTTEMPERATURE = 0x2B
ADDR_REGISTERED = 0x2C
ADDR_MOVING = 0x2E
ADDR_LOCK = 0x2F
ADDR_PUNCH_L = 0x30
ADDR_PUNCH_H = 0x31


#Special ID(s)
ID_BROADCAST = 0xFE


#Status return levels
SRL_NONE = 0x0
SRL_READ = 0x1
SRL_ALL = 0x2


#Dynamixel metrics
AX12_WEIGHT = 53.5 #grams
AX12_RESOLUTION = 0.29 #degrees
AX12_GEARREDUCTIONRATIO = 254 # 254 : 1
AX12_STALLTORQUE = 1.5 #Nm
AX12_NOLOADSPEED = 59 #rpm
AX12_NORMALVOLTAGE = 11.1 #V
