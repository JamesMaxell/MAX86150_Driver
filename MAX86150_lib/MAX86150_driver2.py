#基于Max30102的microPython驱动和Max86150的Arduino驱动编写的Max86150的microPython驱动
from machine import I2C, Pin
from ustruct import unpack
from utime import sleep,sleep_ms, ticks_diff, ticks_ms
import time
import builtins
import binascii

from MAX86150_lib.circular_buffer import CircularBuffer   #循环缓冲区的简单实现


# 定义I2C设备地址
#Max30102_I2C_ADDR = 0x57   #这是max30102的I2C地址
Max86150_I2C_ADDR = 0x5E

MAX86150_SLAVE_ADDRESS_WRITE = 0xBC
MAX86150_SLAVE_ADDRESS_READ =0xBD

# 定义寄存器地址
#----------------------------------------

#状态寄存器
MAX86150_INTSTAT1  =         0x00 
MAX86150_INTSTAT2  =         0x01
MAX86150_INTENABLE1  =       0x02
MAX86150_INTENABLE2  =       0x03
#FIFO寄存器
MAX86150_FIFOWRITEPTR  = 0x04
MAX86150_FIFOOVERFLOW  = 0x05
MAX86150_FIFOREADPTR   = 0x06
MAX86150_FIFODATA      =          0x07
# FIFO Data Control
MAX86150_FIFOCONFIG  =       0x08
MAX86150_FIFOCONTROL1 = 0x09
MAX86150_FIFOCONTROL2 = 0x0A
# PPG Configuration
MAX86150_SYSCONTROL =   0x0D
MAX86150_PPGCONFIG1 =        0x0E
MAX86150_PPGCONFIG2 =        0x0F
MAX86150_LED_PROX_AMP = 0x10
# LED Pulse Amplitude
MAX86150_LED1_PULSEAMP =0x11
MAX86150_LED2_PULSEAMP =0x12
MAX86150_LED_RANGE     =         0x14
MAX86150_LED_PILOT_PA  =     0x15
#ECG Configuration
MAX86150_ECG_CONFIG1  =      0x3C
MAX86150_ECG_CONFIG3  =      0x3E
MAX86150_PROXINTTHRESH =0x10
# Part ID
MAX86150_PARTID  =           0xFF

#MAX86150 Commands
MAX86150_INT_A_FULL_MASK =     0x7F  #0X80 OK
MAX86150_INT_A_FULL_ENABLE =   0x80
MAX86150_INT_A_FULL_DISABLE =   0x00

MAX86150_INT_DATA_RDY_MASK = 0xBF  #b'\x40'
MAX86150_INT_DATA_RDY_ENABLE =  0x40
MAX86150_INT_DATA_RDY_DISABLE = 0x00

MAX86150_INT_ALC_OVF_MASK = 0xDF   #b'\x20'
MAX86150_INT_ALC_OVF_ENABLE =   0x20
MAX86150_INT_ALC_OVF_DISABLE = 0x00

MAX86150_INT_PROX_INT_MASK = 0xEF  #b'\x10'
MAX86150_INT_PROX_INT_ENABLE = 0x10
MAX86150_INT_PROX_INT_DISABLE = 0x00

MAX86150_SAMPLEAVG_MASK =  0xF8    #b'\xE0' #OK
MAX86150_SAMPLEAVG_1 =   0x00
MAX86150_SAMPLEAVG_2 =   0x01
MAX86150_SAMPLEAVG_4 =   0x02
MAX86150_SAMPLEAVG_8 =   0x03
MAX86150_SAMPLEAVG_16 =   0x04
MAX86150_SAMPLEAVG_32 =   0x05 #0x06和0x07都代表32次采样取平均

MAX86150_ROLLOVER_MASK =   0xEF  #OK
MAX86150_ROLLOVER_ENABLE = 0x10
MAX86150_ROLLOVER_DISABLE = 0x00

MAX86150_EN_FIFO_MASK = 0xFB  #OK
MAX86150_EN_FIFO_ENABLE = 0x04
MAX86150_EN_FIFO_DISABLE =0x00

MAX86150_A_FULL_MASK =  0xF0  #OK


MAX86150_SHUTDOWN_MASK =   0xFD  #0x7F OK
MAX86150_SHUTDOWN =    0x02
MAX86150_WAKEUP =      0x00

MAX86150_RESET_MASK =    0xFE#ok
MAX86150_RESET =         0x01

#配置FIFO数据格式的四个element(元素)，即FIFO数据按照什么格式顺序填入队列缓冲，例如先填入red的数据，接着是ir的数据，最后是ecg的数据
#只读取一个元素或者两个元素，需要配置的寄存器地址为MAX86150_FIFOCONTROL1 = 0x09
#当需要读取三个及以上元素的数据时候，除了配置0x09地址外，还需要配置MAX86150_FIFOCONTROL2 = 0x0A
#============================================================
MAX86150_MODE_MASK =    0xFF
MAX86150_MULTIMODE_MASK =    0xF0
MAX86150_MODE_NONE =    0x00
MAX86150_MODE_REDONLY =   0x02
MAX86150_MODE_IRONLY =   0x01
MAX86150_MODE_ECGONLY =   0x09

MAX86150_MODE_RED_IRONLY =   0x12
MAX86150_MODE_IR_REDONLY =   0x21
MAX86150_MODE_RED_ECGONLY =   0x92
MAX86150_MODE_ECG_REDONLY =   0x29
MAX86150_MODE_IR_ECGONLY =   0x91
MAX86150_MODE_ECG_REDONLY =   0x19

RED_MODE = 1
RED_IR_MODE = 2
RED_IR_ECG_MODE = 3
#===========================================

MAX86150_ADCRANGE_MASK =   0x3F  #0x9F  ok
MAX86150_ADCRANGE_4096 =   0x00
MAX86150_ADCRANGE_8192 =   0x40
MAX86150_ADCRANGE_16384 =   0x80
MAX86150_ADCRANGE_32768 =   0xC0

MAX86150_SAMPLERATE_MASK = 0xC3 #0xE3  ok
MAX86150_SAMPLERATE_50 =   0x08
MAX86150_SAMPLERATE_100 =   0x10
MAX86150_SAMPLERATE_200 =   0x14
MAX86150_SAMPLERATE_400 =   0x18
MAX86150_SAMPLERATE_800 =   0x1C
MAX86150_SAMPLERATE_1000 = 0x20
MAX86150_SAMPLERATE_1600 = 0x24
MAX86150_SAMPLERATE_3200 = 0x28

MAX86150_PULSEWIDTH_MASK = 0xFC  #OK
MAX86150_PULSEWIDTH_50 =   0x00
MAX86150_PULSEWIDTH_100 =   0x01
MAX86150_PULSEWIDTH_200 =   0x02
MAX86150_PULSEWIDTH_400 =   0x03

MAX86150_ECG_SAMPLE_RATE_MASK= 0xF8   #OK #配置ECG寄存器MAX86150_ECG_CONFIG1  = 0x3C
MAX86150_ECG_SAMPLE_200_0 =   0x03
MAX86150_ECG_SAMPLE_400_0 =   0x02
MAX86150_ECG_SAMPLE_800_0 =   0x01
MAX86150_ECG_SAMPLE_1600_0 = 0x00
MAX86150_ECG_SAMPLE_400_1 =   0x07
MAX86150_ECG_SAMPLE_800_1 =   0x06
MAX86150_ECG_SAMPLE_1600_1 = 0x05
MAX86150_ECG_SAMPLE_3200_1 = 0x04

MAX86150_ECG_PGA_GAIN_MASK= 0xF3  #OK#配置ECG寄存器MAX86150_ECG_CONFIG3  =  0x3E, 配置芯片PGA放大器增益（倍数）
MAX86150_ECG_PGA_GAIN_1= 0x00     #默认PGA增益为1
MAX86150_ECG_PGA_GAIN_2= 0x04
MAX86150_ECG_PGA_GAIN_4= 0x08
MAX86150_ECG_PGA_GAIN_8= 0x0C
MAX86150_ECG_IA_GAIN_MASK= 0xFC  #OK#配置ECG寄存器MAX86150_ECG_CONFIG3  =  0x3E ，配置Instrumentation Amplifier 放大器增益（倍数）
MAX86150_ECG_IA_GAIN_5= 0x00
MAX86150_ECG_IA_GAIN_9_5= 0x01
MAX86150_ECG_IA_GAIN_20= 0x02
MAX86150_ECG_IA_GAIN_50= 0x03

MAX86150_SLOT1_MASK =    0xF0  #OK
MAX86150_SLOT2_MASK =    0x0F
MAX86150_SLOT3_MASK =    0xF0
MAX86150_SLOT4_MASK =    0x0F

# LED brightness level. It affects the distance of detection.
MAX86150_PULSE_AMP_LOWEST = 0x02  # 0.4mA  - Presence detection of ~4 inch
MAX86150_PULSE_AMP_LOW = 0x1F  # 6.4mA  - Presence detection of ~8 inch
MAX86150_PULSE_AMP_MEDIUM = 0x7F  # 25.4mA - Presence detection of ~8 inch
MAX86150_PULSE_AMP_HIGH = 0xFF  # 50.0mA - Presence detection of ~12 inch

SLOT_NONE       =      0x00    #ok
SLOT_RED_LED    =      0x01
SLOT_IR_LED      =      0x02
SLOT_RED_PILOT   =      0x05  #0x09
SLOT_IR_PILOT    =      0x06    #0x0A
SLOT_ECG        =      0x09   #0x0D



MAX_86150_EXPECTEDPARTID = 0x1E #b'\x1e' 30




I2C_BUFFER_LENGTH = 32

# Size of the queued readings
STORAGE_QUEUE_SIZE = 4


# Data structure to hold the last readings
class SensorData:
    def __init__(self):
        self.red = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.IR = CircularBuffer(STORAGE_QUEUE_SIZE)
        self.ecg = CircularBuffer(STORAGE_QUEUE_SIZE)

# 定义Max86150类
class Max86150:
    
    def __init__(self,
                 i2c,
                 addr = Max86150_I2C_ADDR,
                 ):
        self.i2c = i2c
        self.address = addr
      
        self.activeDevices = None   #self._active_leds = None
        self.pulseWidth = None      #self._pulse_width = None
        self.toGet = None           #self._multi_led_read_mode = None
        
        # Store current config values to compute acquisition frequency
        self.sampleRate = None       #self._sample_rate = None
        self.sampleAverage = None    #self._sample_avg = None
        self._acq_frequency = None
        self._acq_frequency_inv = None
        # Circular buffer of readings from the sensor
        self.sense = SensorData()        
        
# #        self.sense = {'red': [0]*STORAGE_QUEUE_SIZE, 'IR': [0]*STORAGE_QUEUE_SIZE, 'ecg': [0]*STORAGE_QUEUE_SIZE, 'head': 0, 'tail': 1}
#          if self.readPartID() != MAX_86150_EXPECTEDPARTID:
#             print("MAX86150 was not found. Please check wiring/power.")
#             while True:
#                 pass
#         else:
#             print(self.readPartID())
            
        self.setup()    # sampleAverage = 8, ledMode = 3, sampleRate = 400, pulseWidth = 411, adcRange = 16384
           
    def setup(self,
              powerLevel= MAX86150_PULSE_AMP_MEDIUM,
              sampleAverage = 8,
              ledMode = RED_IR_ECG_MODE,
              sampleRate = 400,
              pulseWidth = 400,
              adcRange = 16384,
              ECGsampleRate=200,
              ECG_PGA_gain=8,
              ECG_IA_gain=9.5,
              ):
        #sampleAverage = 8, ledMode = 3, sampleRate = 400, pulseWidth = 411, adcRange = 16384
        self.softReset()
        sleep_ms(100)
        
        self.setFIFOAverage(sampleAverage)
        self.enableFIFORollover()

        # Set the LED mode to the default value of 2 (RED + IR)
        # Note: the 3rd mode is available only with MAX30105
        self.setLEDMode(ledMode)
        
        # Set the ADC range to default value of 16384       =0x80
        self.setADCRange(adcRange)
        
        # Set the sample rate to the default value of 400
        self.setSampleRate(sampleRate)

        # Set the Pulse Width to the default value of 411
        self.setPulseWidth(pulseWidth)
        
        # Set the LED brightness to the default value of 'low'
        self.writeRegister8(self.address,MAX86150_LED_RANGE,0x00)   #  LED_RED的电流范围设置为50mA档.LED_IR的电流范围设置为50mA档   
        self.setActiveLedsAmplitude(powerLevel)  #  设置LED_RED的电流为25.4mA档.LED_IR的电流设置为25.4mA，  0x7F   51*127/255=25.4
#         self.setPulseAmplitudeRed(powerLevel)
#         self.setPulseAmplitudeIR(powerLevel)
#         self.setPulseAmplitudeProximity(powerLevel)
        
        self.set_ECG_SampleRate(ECGsampleRate)
        self.set_ECG_PGA_GAIN(ECG_PGA_gain)
        self.set_ECG_IA_GAIN(ECG_IA_gain)
        
        self.bitMask(MAX86150_SYSCONTROL, MAX86150_EN_FIFO_MASK, MAX86150_EN_FIFO_ENABLE) #start FIFO

        
#         #FIFO Control 1 = FD2|FD1, slot1=LED_RED数据,slot2=LED_IR数据； FIFO Control 2 = FD4|FD3，slot3=ECG数据,slot4=无效
#         self.writeRegister8(self.address,MAX86150_FIFOCONTROL1,0x21)#b'\x21'
#         self.writeRegister8(self.address,MAX86150_FIFOCONTROL2,0x09)#b'\x09'
        
#         self.writeRegister8(self.address,MAX86150_FIFOCONTROL1, (char)(FIFOCode & 0x00FF) );
#         self.writeRegister8(self.address,MAX86150_FIFOCONTROL2, (char)(FIFOCode >>8) );
        
#        self.writeRegister8(self.address,MAX86150_PPGCONFIG1,0xD1)#b'\xD1'
#        self.writeRegister8(self.address,MAX86150_PPGCONFIG2,0x06)#b'\x06'
#        self.writeRegister8(self.address,MAX86150_LED_RANGE,0x00)#b'\x00'         #PPG_ADC_RGE: 32768nA
#        self.writeRegister8(self.address,MAX86150_SYSCONTROL,0x04)#b'\x04'        #start FIFO
#         self.writeRegister8(self.address,MAX86150_ECG_CONFIG1,0x03)#b'\x03'    # SR: 200
#         self.writeRegister8(self.address,MAX86150_ECG_CONFIG3,0x0D)#b'\x0D'    # IA Gain: 9.5 / PGA Gain: 8
        
#         self.setPulseAmplitudeRed(0xFF)
#         self.setPulseAmplitudeIR(0xFF)
        
#         FIFOCode = 0x00
#         FIFOCode = FIFOCode<<4 | 0x0009   #insert ECG front of ETI in FIFO
#         FIFOCode = FIFOCode<<8 | 0x0021   #insert Red(2) and IR (1) in front of ECG in FIFO        
        #   Multi-LED Mode Configuration, Enable the reading of the three LEDs
        #   -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        #   enableSlot(1, SLOT_RED_LED);
        #   if (ledMode > 1)
        #   enableSlot(2, SLOT_IR_LED);
        #   if (ledMode > 2)
        #   enableSlot(3, SLOT_ECG);
        #   enableSlot(1, SLOT_RED_PILOT);
        #   enableSlot(2, SLOT_IR_PILOT);
        #   enableSlot(3, SLOT_GREEN_PILOT);
        #   -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        self.clearFIFO()


    #配置Max86150的寄存器
    #=====================================================================
    
    #中断配置（Interrupt configuration）=============================================================
    def getINT1(self):
        return self.readRegister8(self.address, MAX86150_INTSTAT1)
        
    def getINT2(self):
        return self.readRegister8(self.address, MAX86150_INTSTAT2)
        
    def enableAFULL(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_A_FULL_MASK, MAX86150_INT_A_FULL_ENABLE)

    def disableAFULL(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_A_FULL_MASK, MAX86150_INT_A_FULL_DISABLE)
        
    def enableDATARDY(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_DATA_RDY_MASK, MAX86150_INT_DATA_RDY_ENABLE)

    def disableDATARDY(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_DATA_RDY_MASK, MAX86150_INT_DATA_RDY_DISABLE)
        
    def enableALCOVF(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_ENABLE)

    def disableALCOVF(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_DISABLE)
        
    def enablePROXINT(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_PROX_INT_MASK, MAX86150_INT_PROX_INT_ENABLE)

    def disablePROXINT(self):
        self.bitMask(MAX86150_INTENABLE1, MAX86150_INT_PROX_INT_MASK, MAX86150_INT_PROX_INT_DISABLE)
    #End Interrupt configuration =======================================================================
        
          
    # 软件重启Max86150
    def softReset(self):
        # When the RESET bit is set to one, all configuration, threshold,
        # and data registers are reset to their power-on-state through
        # a power-on reset. The RESET bit is cleared automatically back to zero
        # after the reset sequence is completed. (datasheet pag. 19)

        self.bitMask(MAX86150_SYSCONTROL, MAX86150_RESET_MASK, MAX86150_RESET)
        curr_status = -1
        while not ((curr_status & MAX86150_RESET) == 0):
            sleep_ms(10)
            curr_status = ord(self.readRegister8(self.address,MAX86150_SYSCONTROL))

    # 从设备中读取设备ID
    def readPartID(self):
#self.i2c.writeto(self.address, bytearray([MAX86150_FIFODATA]))
        #self.i2c.writeto(self.address, bytearray([MAX86150_FIFODATA]))
        #ID = self.readRegister8(self.address, MAX86150_PARTID)#self.i2c.readfrom_mem(self.address,MAX86150_PARTID, 1)
        return self.readRegister8(self.address, MAX86150_PARTID)
    
    def check_part_id(self):
    # Checks the correctness of the Device ID
        part_id = ord(self.readPartID())
        return part_id == MAX_86150_EXPECTEDPARTID
        

    #Put IC into low power mode (datasheet pg. 28)
    # During shutdown the IC will continue to respond to I2C commands but will
    # not update with or take new readings (such as temperature)
    def shutDown(self):
        self.bitMask(MAX86150_SYSCONTROL, MAX86150_SHUTDOWN_MASK, MAX86150_SHUTDOWN)

    # Pull IC out of low power mode (datasheet pg. 28)
    def wakeUp(self):
        self.bitMask(MAX86150_SYSCONTROL, MAX86150_SHUTDOWN_MASK, MAX86150_WAKEUP)

    # Set which LEDs are used for sampling -- Red only, IR only, or RED+IR（Multi-LED mode）.
    # See datasheet, page 30
    def setLEDMode(self,LED_mode):
#        self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_MODE_MASK, mode)
        # Set LED mode: select which LEDs are used for sampling RED_MODE = 1 , RED_IR_MODE = 2 , RED_IR_ECG_MODE = 3
        # Options: RED only, RED + IR only, or RED + IR+ EEG (datasheet pag. 19)
        if LED_mode == RED_MODE:
            self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_MODE_MASK, MAX86150_MODE_REDONLY)
            self.bitMask(MAX86150_FIFOCONTROL2, MAX86150_MULTIMODE_MASK, MAX86150_MODE_NONE)
        elif LED_mode == RED_IR_MODE:
            self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_MODE_MASK, MAX86150_MODE_RED_IRONLY)
            self.bitMask(MAX86150_FIFOCONTROL2, MAX86150_MULTIMODE_MASK, MAX86150_MODE_NONE)
        elif LED_mode == RED_IR_ECG_MODE:
            self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_MODE_MASK, MAX86150_MODE_RED_IRONLY)
            self.bitMask(MAX86150_FIFOCONTROL2, MAX86150_MULTIMODE_MASK, MAX86150_MODE_ECGONLY)
        else:
            raise ValueError('Wrong LED mode:{0}!'.format(LED_mode))

        # Multi-LED Mode Configuration: enable the reading of the LEDs
#         # depending on the chosen mode
#         self.enable_slot(1, SLOT_RED_LED)
#         if LED_mode > 1:
#             self.enable_slot(2, SLOT_IR_LED)
#         if LED_mode > 2:
#             self.enable_slot(3, SLOT_GREEN_LED)

        # Store the LED mode used to control how many bytes to read from
        # FIFO buffer in multiLED mode: a sample is made of 3 bytes
        self.activeDevices = LED_mode
        self.toGet  = LED_mode * 3    #_multi_led_read_mode


    # adcRange: one of MAX86150_ADCRANGE_4096, _8192, _16384, _32768
    def setADCRange(self, adcRange):
        # ADC range: set the range of the conversion
        # Options: 4096, 8192, 16384, 32768
        # Current draw: 7.81pA. 15.63pA, 31.25pA, 62.5pA per LSB.
        if adcRange == 4096:
            ADC_range = MAX86150_ADCRANGE_4096
        elif adcRange == 8192:
            ADC_range = MAX86150_ADCRANGE_8192
        elif adcRange == 16384:
            ADC_range = MAX86150_ADCRANGE_16384
        elif adcRange == 32768:
            ADC_range = MAX86150_ADCRANGE_32768
        else:
            raise ValueError('Wrong ADC range:{0}!'.format(adcRange))

        self.bitMask(MAX86150_PPGCONFIG1, MAX86150_ADCRANGE_MASK, ADC_range)

    # sampleRate: one of MAX86150_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
    # Sample Rate Configuration
    def setSampleRate(self, sample_rate):
        # Sample rate: select the number of samples taken per second.
        # Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
        # Note: in theory, the resulting acquisition frequency for the end user
        # is sampleRate/sampleAverage. However, it is worth testing it before
        # assuming that the sensor can effectively sustain that frequency
        # given its configuration.
        if sample_rate == 50:
            sr = MAX86150_SAMPLERATE_50
        elif sample_rate == 100:
            sr = MAX86150_SAMPLERATE_100
        elif sample_rate == 200:
            sr = MAX86150_SAMPLERATE_200
        elif sample_rate == 400:
            sr = MAX86150_SAMPLERATE_400
        elif sample_rate == 800:
            sr = MAX86150_SAMPLERATE_800
        elif sample_rate == 1000:
            sr = MAX86150_SAMPLERATE_1000
        elif sample_rate == 1600:
            sr = MAX86150_SAMPLERATE_1600
        elif sample_rate == 3200:
            sr = MAX86150_SAMPLERATE_3200
        else:
            raise ValueError('Wrong sample rate:{0}!'.format(sample_rate))

        self.bitMask(MAX86150_PPGCONFIG1, MAX86150_SAMPLERATE_MASK, sr)

        # Store the sample rate and recompute the acq. freq.
        self.sampleRate = sample_rate
        self.update_acquisition_frequency()
        
    # pulseWidth: one of MAX86150_PULSEWIDTH_50, _100, _200, _400
    def setPulseWidth(self, pulseWidth):
        # Pulse width of LEDs: The longer the pulse width the longer range of
        # detection. At 50us and 0.4mA it's about 2 inches,
        # at 411us and 0.4mA it's about 6 inches.
        if pulseWidth == 50:
            pulse_width = MAX86150_PULSEWIDTH_50
        elif pulseWidth == 100:
            pulse_width = MAX86150_PULSEWIDTH_100
        elif pulseWidth == 200:
            pulse_width = MAX86150_PULSEWIDTH_200
        elif pulseWidth == 400:
            pulse_width = MAX86150_PULSEWIDTH_400
        else:
            raise ValueError('Wrong pulse width:{0}!'.format(pulseWidth))
        self.bitMask(MAX86150_PPGCONFIG1, MAX86150_PULSEWIDTH_MASK, pulse_width)

        # Store the pulse width
        self.pulseWidth = pulse_width

    # LED Pulse Amplitude Configuration methods
    def setActiveLedsAmplitude(self, amplitude):
        if self.activeDevices > 0:
            self.setPulseAmplitudeRed(amplitude)
        if self.activeDevices > 1:
            self.setPulseAmplitudeIR(amplitude)
        if self.activeDevices > 2:
            self.setPulseAmplitudeProximity(amplitude)

    #  NOTE: Red LED Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical),See datasheet, page 31
    def setPulseAmplitudeRed(self,amplitude):
        self.writeRegister8(self.address,MAX86150_LED2_PULSEAMP,amplitude)
        
    #  NOTE: IR LED Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical),See datasheet, page 31
    def setPulseAmplitudeIR(self,amplitude):
        self.writeRegister8(self.address,MAX86150_LED1_PULSEAMP,amplitude)

    #  NOTE: 设置接近模式Proximity
    def setPulseAmplitudeProximity(self,amplitude):
        self.writeRegister8(self.address,MAX86150_LED_PROX_AMP,amplitude)
        
    #  The threshMSB signifies only the 8 most significant-bits of the ADC count.
    def setProximityThreshold(self,threshMSB):
        self.writeRegister8(self.address,MAX86150_PROXINTTHRESH,amplitude)
        
    #  Given a slot number assign a thing to it
    # Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
    # Assigning a SLOT_RED_LED will pulse LED
    #Assigning a SLOT_RED_PILOT will ??
    def enableSlot(self,slotNumber,threshMSB):
        if slotNumber==1: 
            self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_SLOT1_MASK, threshMSB)
            #break;
        elif slotNumber==2:
            self.bitMask(MAX86150_FIFOCONTROL1, MAX86150_SLOT2_MASK, threshMSB << 4)
            #break;
        elif slotNumber==3:
            self.bitMask(MAX86150_FIFOCONTROL2, MAX86150_SLOT3_MASK, threshMSB)
            #break;
        elif slotNumber==4:
            self.bitMask(MAX86150_FIFOCONTROL2, MAX86150_SLOT4_MASK, threshMSB << 4)
            #break;
        else:
            raise ValueError('Wrong slot number:{0}!'.format(slot_number))

    #Clears all slot assignments
    def disableSlots(self):
        self.writeRegister8(self.address,MAX86150_FIFOCONTROL1,0)
        self.writeRegister8(self.address,MAX86150_FIFOCONTROL2,0)
        
    def set_ECG_SampleRate(self, ECG_sample_rate):
        # ECG Sample rate: select the number of ECG samples taken per second.
        # Options:  200, 400, 800, 1600, 3200
        # Note: in theory, the resulting acquisition frequency for the end user
        # is sampleRate/sampleAverage. However, it is worth testing it before
        # assuming that the sensor can effectively sustain that frequency
        # given its configuration.
        if ECG_sample_rate == 200:
            ECG_sr = MAX86150_ECG_SAMPLE_200_0 
        elif ECG_sample_rate == 400:
            ECG_sr = MAX86150_ECG_SAMPLE_400_0 
        elif ECG_sample_rate == 800:
            ECG_sr = MAX86150_ECG_SAMPLE_800_0 
        elif ECG_sample_rate == 1600:
            ECG_sr = MAX86150_ECG_SAMPLE_1600_0 
        elif ECG_sample_rate == 3200:
            ECG_sr = MAX86150_ECG_SAMPLE_3200_1 
        else:
            raise ValueError('Wrong ECG sample rate:{0}!'.format(ECG_sample_rate))

        self.bitMask(MAX86150_ECG_CONFIG1 , MAX86150_ECG_SAMPLE_RATE_MASK, ECG_sr)
        
    def set_ECG_PGA_GAIN(self, ECG_PGA_GAIN):
        # ECG PGA GAIN: select the number of ECG PGA Gain times.
        # Options:  1 ,2 , 4 , 8
        # given its configuration.
        if ECG_PGA_GAIN == 1:
            ECG_PGA_gain = MAX86150_ECG_PGA_GAIN_1 
        elif ECG_PGA_GAIN == 2:
            ECG_PGA_gain = MAX86150_ECG_PGA_GAIN_2 
        elif ECG_PGA_GAIN == 4:
            ECG_PGA_gain = MAX86150_ECG_PGA_GAIN_4 
        elif ECG_PGA_GAIN == 8:
            ECG_PGA_gain = MAX86150_ECG_PGA_GAIN_8 
        else:
            raise ValueError('Wrong ECG PGA GAIN:{0}!'.format(ECG_PGA_GAIN))

        self.bitMask(MAX86150_ECG_CONFIG3 , MAX86150_ECG_PGA_GAIN_MASK, ECG_PGA_gain)
        
    def set_ECG_IA_GAIN(self, ECG_IA_GAIN):
        # ECG IA GAIN: select the number of ECG Instrumentation Amplifier Gain times.
        # Options:  5 ,9.5 , 20 , 50
        # given its configuration.
        if ECG_IA_GAIN == 5:
            ECG_IAgain = MAX86150_ECG_IA_GAIN_5 
        elif ECG_IA_GAIN == 9.5:
            ECG_IAgain = MAX86150_ECG_IA_GAIN_9_5 
        elif ECG_IA_GAIN == 20:
            ECG_IAgain = MAX86150_ECG_IA_GAIN_20 
        elif ECG_IA_GAIN == 50:
            ECG_IAgain = MAX86150_ECG_IA_GAIN_50 
        else:
            raise ValueError('Wrong ECG IA GAIN:{0}!'.format(ECG_IA_GAIN))

        self.bitMask(MAX86150_ECG_CONFIG3 , MAX86150_ECG_IA_GAIN_MASK, ECG_IAgain)

    #FIFO Configuration
    def setFIFOAverage(self,numberOfSamples):
        #清空原有寄存器里的值
        self.writeRegister8(self.address,MAX86150_FIFOCONFIG,0x7F)#b'\x7F' 清空FIFO CONFIG寄存器
        
        #FIFO设置：
        if numberOfSamples == 1:
            ns = MAX86150_SAMPLEAVG_1
        elif numberOfSamples == 2:
            ns = MAX86150_SAMPLEAVG_2
        elif numberOfSamples == 4:
            ns = MAX86150_SAMPLEAVG_4
        elif numberOfSamples == 8:
            ns = MAX86150_SAMPLEAVG_8
        elif numberOfSamples == 16:
            ns = MAX86150_SAMPLEAVG_16
        elif numberOfSamples == 32:
            ns = MAX86150_SAMPLEAVG_32
        else:
            raise ValueError(
                'Wrong number of samples:{0}!'.format(numberOfSamples))
#         else:
#             self.setFIFOAverage(MAX86150_SAMPLEAVG_8)
        self.bitMask(MAX86150_PPGCONFIG2, MAX86150_SAMPLEAVG_MASK, numberOfSamples)
        
        # Store the number of averaged samples and recompute the acq. freq.
        self.sampleAverage = numberOfSamples
        self.update_acquisition_frequency()

    def update_acquisition_frequency(self):
        if None in [self.sampleRate, self.sampleAverage]:
            return
        else:
            self._acq_frequency = self.sampleRate / self.sampleAverage
            from math import ceil

            # Compute the time interval to wait before taking a good measure
            # (see note in setSampleRate() method)
            self._acq_frequency_inv = int(ceil(1000 / self._acq_frequency))   #ceil函数的功能是向上取整

    def get_acquisition_frequency(self):
        return self._acq_frequency

    #Resets all points to start in a known state
    def clearFIFO(self):
        self.writeRegister8(self.address,MAX86150_FIFOWRITEPTR,0)
        self.writeRegister8(self.address,MAX86150_FIFOOVERFLOW,0)
        self.writeRegister8(self.address,MAX86150_FIFOREADPTR,0)
        
    #Enable roll over if FIFO over flows
    def enableFIFORollover(self):
        self.bitMask(MAX86150_FIFOCONFIG, MAX86150_ROLLOVER_MASK, MAX86150_ROLLOVER_ENABLE)

    #Disable roll over if FIFO over flows
    def disableFIFORollover(self):
        self.bitMask(MAX86150_FIFOCONFIG, MAX86150_ROLLOVER_MASK, MAX86150_ROLLOVER_DISABLE)

    #Power on default is 32 samples ; Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
    def setFIFOAlmostFull(self,numberOfSamples):
        self.bitMask(MAX86150_FIFOCONFIG, MAX86150_A_FULL_MASK, numberOfSamples)

    # Read the FIFO Write Pointer
    def getWritePointer(self):
        return self.readRegister8(self.address, MAX86150_FIFOWRITEPTR)

    # Read the FIFO Read Pointer
    def getReadPointer(self):
        return self.readRegister8(self.address, MAX86150_FIFOREADPTR)

    #Set the PROX_INT_THRESHold
    def setPROXINTTHRESH(self,val):
        self.writeRegister8(self.address,MAX86150_PROXINTTHRESH,val)
   
    
    #Tell caller how many samples are available
#     def available(self):
#         # Read the status register
# #         status = self.i2c.readfrom_mem(self.address,MAX86150_INTSTAT1, 1)[0]
# #         # Check if new data is available
# #         if (status & (5 << MAX86150_FIFOREADPTR)) == 0:
# #             return 0
# #         else:            
#             numberOfSamples = (self.sense['head'] - self.sense['tail']) % STORAGE_QUEUE_SIZE
#             if (numberOfSamples < 0):
#                 numberOfSamples += STORAGE_QUEUE_SIZE
#             return numberOfSamples   

#         # Limit the number of samples to read
#         numberOfSamples = min(numberOfSamples, STORAGE_QUEUE_SIZE)


    #Report the most recent red value
    def getRed(self):
        if(self.safeCheck(250)):    #Check the sensor for new data for 250ms
            return self.sense.red.pop_head()
        else:
            return 0

    #Report the most recent IR value
    def getIR(self):
        if(self.safeCheck(250)):    #Check the sensor for new data for 250ms
            return self.sense.IR.pop_head()
        else:
            return 0
    
    #Report the most recent ECG value
    def getECG(self):
        if(self.safeCheck(250)):    #Check the sensor for new data for 250ms
            return self.sense.ecg.pop_head()
        else:
            return 0
    
    #Report the next Red value in the FIFO
    def getFIFORed(self):
        if len(self.sense.red) == 0:
            return 0
        else:
            return self.sense.red.pop()
       
    #Report the next IR value in the FIFO
    def getFIFOIR(self):
        if len(self.sense.IR) == 0:
            return 0
        else:
            return self.sense.IR.pop()
           
    #Report the next ECG value in the FIFO
    def getFIFOECG(self):
        if len(self.sense.ecg) == 0:
            return 0
        else:
            return self.sense.ecg.pop()
    
    #update the tail
    def nextSample(self):
        if self.available():      # Only advance the tail if new data is available
            self.sense['tail'] +=1 
            self.sense['tail'] %= STORAGE_QUEUE_SIZE  # Wrap condition
        
    # //Polls the sensor for new data
    # //Call regularly
    # //If new data is available, it updates the head and tail in the main struct
    # //Returns number of new samples obtained        
    def check(self):          #//Read register FIDO_DATA in (3-byte * number of active LED) chunks   //Until FIFO_RD_PTR = FIFO_WR_PTR
        
        I2C_BUFFER_LENGTH=32
        
        readPointer =ord(self.getReadPointer())# self.getReadPointer()    # Read the FIFO read pointer
        writePointer = ord(self.getWritePointer())  # Read the FIFO write pointer
        
        
        if readPointer != writePointer:     #Do we have new data?
            numberOfSamples = writePointer - readPointer   #Calculate the number of readings we need to get from sensor
#             print(readPointer, writePointer, numberOfSamples)
            if numberOfSamples < 0:
                numberOfSamples += 32
                
            for i in range(numberOfSamples):
                get_FIFO_data = self.readRegister8(self.address,MAX86150_FIFODATA, self.toGet)
                temp = bytearray(4) # Array of 4 bytes that we will convert into long
                
                if self.activeDevices > 0:
                    # Burst read three bytes - RED
                    temp[3] = 0
                    temp[2] = get_FIFO_data[0]
                    temp[1] = get_FIFO_data[1]
                    temp[0] = get_FIFO_data[2]
#                     print(temp[2],temp[1],temp[0],self.fifo_bytes_to_int(get_FIFO_data[0:3]) & 0x7FFFF)
                    self.sense.red.append(self.fifo_bytes_to_int(get_FIFO_data[0:3]) & 0x7FFFF)
                    
                if self.activeDevices > 1:
                    # Burst read three bytes - RED
                    temp[3] = 0
                    temp[2] = get_FIFO_data[3]
                    temp[1] = get_FIFO_data[4]
                    temp[0] = get_FIFO_data[5]
                    
                    self.sense.IR.append(self.fifo_bytes_to_int(get_FIFO_data[3:6]) & 0x7FFFF)
                if self.activeDevices > 2:
                    # Burst read three bytes - RED
                    temp[3] = 0
                    temp[2] = get_FIFO_data[6]
                    temp[1] = get_FIFO_data[7]
                    temp[0] = get_FIFO_data[8]
                    self.sense.ecg.append(self.fifo_bytes_to_int(get_FIFO_data[6:9]) & 0x3FFFF)
                return True 
                
#             # Limit the number of samples to read
#             numberOfSamples = min(numberOfSamples, STORAGE_QUEUE_SIZE)
#             
#             bytesLeftToRead = numberOfSamples * self.activeDevices * 3    #doing Red and IR (3 bytes each)
# 
#             while bytesLeftToRead > 0:
#                 # Read one sample from the sensor
#                 self.toGet = bytesLeftToRead
#                 if self.toGet > I2C_BUFFER_LENGTH:
#                     self.toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (self.activeDevices * 3))
#                 bytesLeftToRead -= self.toGet
#                 #Request self.toGet number of bytes from sensor
#                 while self.toGet > 0:


#                     # Read one sample from the sensor
# #                     data = self.i2c.readfrom_mem(self.address,MAX86150_FIFODATA, self.toGet)
#                     get_FIFO_data = self.readRegister8(self.address,MAX86150_FIFODATA, self.toGet)
#                     if self.activeDevices > 0:
#                     # Advance the head of the storage struct
#                         self.sense.red.append(self.fifo_bytes_to_int(get_FIFO_data[0:3]))
# 
#                     if self.activeDevices > 1:
#                         self.sense.IR.append(self.fifo_bytes_to_int(get_FIFO_data[3:6]))
# 
# 
#                     if self.activeDevices > 2:
#                         self.sense.green.append(self.fifo_bytes_to_int(get_FIFO_data[6:9]))
# 
#                     self.toGet -= self.activeDevices * 3
#                     return True                        #print(self.sense['ecg'][self.sense['head']])
#                     #numberOfSamples -= 1
                        
                    
        else:
            return False   #Let the world know how much new data we found
    
    
    #Check for new data but give up after a certain amount of time
    def safeCheck(self, maxTimeToCheck):
        markTime = time.ticks_ms()

        while True:
            if time.ticks_ms() - markTime > maxTimeToCheck:
                return False

            if self.check() == True: # We found new data!
                return True
        time.sleep_ms(10)
                
    #Given a register, read it, mask it, and then set the thing
    def bitMask(self, reg, mask, thing):
#         reg=bytes([reg])
#         mask=bytes([mask])
#         thing=bytes([thing])
#         print(reg,mask,thing)
#         originalContents = self.readRegister8(self.address, reg)
#         originalContents = bytes([originalContents[i] & mask[i] for i in range(len(originalContents))])
#         self.writeRegister8(self.address,reg, bytes([originalContents[j] | thing[j] for j in range(len(originalContents))]))
#
        originalContents = ord(self.readRegister8(self.address, reg))
#         print(originalContents,reg,mask,thing)
        originalContents = originalContents & mask
#         print(originalContents,thing,originalContents | thing)
        self.writeRegister8(self.address,reg, originalContents | thing)
#         print(reg,mask,thing,originalContents)

    
#     def bitMask(self,reg, mask, thing):
#         # Grab current register context
#         originalContents = self.readRegister8(reg)
# 
#         # Zero-out the portions of the register we're interested in
#         originalContents = originalContents & mask
# 
#         # Change contents
#         self.writeRegister8(self.address, reg, originalContents | thing)
# 
    def readRegister8(self, address, reg, n_bytes=1):
#         # 写入寄存器地址，开始I2C传输
        self.i2c.writeto(address, bytearray([reg]))
        return self.i2c.readfrom(address, n_bytes)
# #         self.i2c.start()
#         self.i2c.writeto(address,  bytes([MAX86150_SLAVE_ADDRESS_WRITE]), False)
#         self.i2c.writeto(address,  bytes([reg]), False)
#         self.i2c.writeto(address,  bytes([MAX86150_SLAVE_ADDRESS_READ]), False)
#         # 从内存地址reg后开始，读取1个字节数据
#         result = self.i2c.readfrom(address,1)
# #         self.i2c.stop()
# #         print(type(result[0]),result,result[0])
#         # 如果有可用数据
#         if self.available()>0 :
#             # 返回读取的数据,读取的result为byte类型，而取result[0]则转为int类型
# #             print(result[0],'tiaoshi')
#             return result[0]
#         else:
#             # 失败返回0
#             return 0

#
    def writeRegister8(self, address, reg, value):
#         self.i2c.start()
        self.i2c.writeto(address,  bytearray([reg, value]))
        return
#         self.i2c.stop()

    # Given a register, read it, mask it, and then set the thing
#     def set_bitmask(self, REGISTER, MASK, NEW_VALUES):
#         newCONTENTS = (ord(self.i2c_read_register(REGISTER)) & MASK) | NEW_VALUES
#         self.i2c_set_register(REGISTER, newCONTENTS)
        return

    def fifo_bytes_to_int(self, fifo_bytes):
        value = unpack(">i", b'\x00' + fifo_bytes)
        return value[0] 

    # Returns how many samples are available
    def available(self):
        number_of_samples = len(self.sense.red)
        return number_of_samples
    
#      def writeRegister8(self, reg, value):
#          self.i2c.writeto(self.address, bytes([reg, value]))


    # 写入寄存器函数
#     def write_register(self, reg_addr, value):
#         self.i2c.writeto_mem(self.i2c_address, reg_addr, bytes([value]))



    # 读取 FIFO 数据
#     def read_fifo(self):
#         # 读取 FIFO 数据计数器
#         fifo_count = self.read_register(REG_FIFO_DATA_COUNT)
#         data = []
#         # 逐个读取 FIFO 数据
#         for i in range(fifo_count):
#             data.append(self.read_register(REG_FIFO_READ))
#         return data

