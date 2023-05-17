from machine import sleep, I2C, Pin, Timer
from utime import ticks_diff, ticks_us
import time
from MAX86150_lib import MAX86150_driver2
from hrcalc import calc_hr_and_spo2

#四个Max86150模块的I2C中断标志引脚（未用）
i2c_INTB1 = Pin(27, Pin.IN)
i2c_INTB2 = Pin(13, Pin.IN)
i2c_INTB3 = Pin(14, Pin.IN)
i2c_INTB4 = Pin(15, Pin.IN)

#四个Max86150模块的Digi_VDD上电选通控制引脚
A0 = Pin(23, Pin.OUT)
A1 = Pin(25, Pin.OUT)
A2 = Pin(26, Pin.OUT)
# #选通控制Max86150模块1上电
# A0.value(0)
# A1.value(0)
# A2.value(0) 

SENSOR1_RED = 0  # 存储心率
FINGER_FLAG = False  # 默认表示未检测到手指
SENSOR2_RED = 0  # 存储血氧
SENSOR3_RED = 0  # 存储温度
SENSOR4_RED = 0
# ------------ 添加新代码 -----------
SENSOR1_LIST = []  # 存储心率的最新10次数据
SENSOR2_LIST = []  # 存储血氧的最新10次数据
SENSOR3_LIST = []  # 存储温度的最新10次数据
SENSOR4_LIST = []  # 存储温度的最新10次数据


def display_info(t):
    # ------------ 添加新代码 -----------
    global SENSOR1_LIST, SENSOR2_LIST, SENSOR3_LIST, SENSOR4_LIST
    
    # 如果没有检测到手指，那么就不显示
    if FINGER_FLAG is False:
        # ------------ 添加新代码 -----------
        SENSOR1_LIST.clear()
        SENSOR2_LIST.clear()
        SENSOR3_LIST.clear()
        SENSOR4_LIST.clear()
        return

#     print('心率: ', SENSOR1_RED, " 血氧：", SENSOR2_RED)

    # ------------ 添加新代码 -----------
    if len(SENSOR1_LIST) < 10:
        SENSOR1_LIST.append(SENSOR1_RED)
        SENSOR1_LIST = SENSOR1_LIST[-10:]  # 保证列表最多30个元素
#         print("正在检测【心率】...", 30 - len(SENSOR1_LIST))
    elif len(SENSOR2_LIST) < 10:
        SENSOR2_LIST.append(SENSOR2_RED)
        SENSOR2_LIST = SENSOR2_LIST[-10:]  # 保证列表最多10个元素
#         print("正在检测【血氧】...", 10 - len(SENSOR2_RED_LIST))
    elif len(SENSOR3_LIST) < 10:
        SENSOR3_LIST.append(SENSOR3_RED)
        SENSOR3_LIST = SENSOR3_LIST[-10:]  # 保证列表最多10个元素
#         print("正在检测【温度】...", 10 - len(TEMP_LIST))
    elif len(SENSOR4_LIST) < 10:
        SENSOR4_LIST.append(SENSOR4_RED)
        SENSOR4_LIST = SENSOR4_LIST[-10:]  # 保证列表最多10个元素
#         print("正在检测【温度】...", 10 - len(TEMP_LIST))
    else:
        print("----------已完成检测----------")
        print('传感器1: ', sum(SENSOR1_LIST[5:])/len(SENSOR1_LIST[5:]),
              ' 传感器2：', sum(SENSOR2_LIST[5:])/len(SENSOR2_LIST[5:]),
              '传感器3: ', sum(SENSOR3_LIST[5:])/len(SENSOR3_LIST[5:]),
              ' 传感器4：', sum(SENSOR4_LIST[5:])/len(SENSOR4_LIST[5:]))


def main():
    global SENSOR1_RED, FINGER_FLAG, SENSOR2_RED, SENSOR3_RED , SENSOR4_RED  # 如果需要对全局变量修改，则需要global声明
    

#     time.sleep_ms(200)   

    t_start = ticks_us()  # Starting time of the acquisition

    MAX_HISTORY = 32
    history = []
    beats_history = []
    beat = False
    red_list = []
    ir_list = []

    red1_reading=0
    red2_reading=0
    red3_reading=0
    red4_reading=0
    while True:
        # ------------ 修改代码 -----------
        if len(SENSOR1_LIST) < 10:
            # 读取第一个传感器的数据
            #选通控制Max86150模块1上电(0 1 2 3)
            A0.value(0)
            A1.value(0)
            A2.value(0)

            # 创建I2C对象(检测MAX86150)
            #定义I2C引脚
            i2c = I2C(1, sda=Pin(4), scl=Pin(5),freq=400000)  # Fast: 400kHz, slow: 100kHz

            # 创建传感器对象
            max86150Sensor_1 = MAX86150_driver2.Max86150(i2c)
            max86150Sensor_1.check()
            time.sleep_ms(100) 
            if max86150Sensor_1.available():
                # FIFO 先进先出，从队列中取数据。都是整形int
                red1_reading = max86150Sensor_1.getFIFORed() & 0x7FFFF
                ir1_reading = max86150Sensor_1.getFIFOIR() & 0x7FFFF
                
                if red1_reading < 1000:
                    print('No finger1',red1_reading)
                    FINGER_FLAG = False  # 表示没有放手指
                    continue
                else:
                    FINGER_FLAG = True  # 表示手指已放
                if 0x7FFFF > red1_reading > 1000:
                    SENSOR1_RED = red1_reading

        if len(SENSOR2_LIST) < 10:
            # 读取第一个传感器的数据
            #选通控制Max86150模块1上电(0 1 2 3)
            A0.value(1)
            A1.value(0)
            A2.value(0)

            # 创建I2C对象(检测MAX86150)
            #定义I2C引脚
            i2c = I2C(1, sda=Pin(4), scl=Pin(5),freq=400000)  # Fast: 400kHz, slow: 100kHz

            # 创建传感器对象
            max86150Sensor_2 = MAX86150_driver2.Max86150(i2c)            
            max86150Sensor_2.check()
            time.sleep_ms(100)
            if max86150Sensor_2.available():
                # FIFO 先进先出，从队列中取数据。都是整形int
                red2_reading = max86150Sensor_2.getFIFORed() & 0x7FFFF
                ir2_reading = max86150Sensor_2.getFIFOIR() & 0x7FFFF
                
                if red2_reading < 1000:
                    print('No finger2',red2_reading)
                    FINGER_FLAG = False  # 表示没有放手指
                    continue
                else:
                    FINGER_FLAG = True  # 表示手指已放
                if 0x7FFFF > red2_reading > 1000:
                    SENSOR2_RED = red2_reading

        if len(SENSOR3_LIST) < 10:
            # 读取第一个传感器的数据
            #选通控制Max86150模块1上电(0 1 2 3)
            A0.value(0)
            A1.value(1)
            A2.value(0)

            # 创建I2C对象(检测MAX86150)
            #定义I2C引脚
            i2c = I2C(1, sda=Pin(4), scl=Pin(5),freq=400000)  # Fast: 400kHz, slow: 100kHz

            # 创建传感器对象
            max86150Sensor_3 = MAX86150_driver2.Max86150(i2c)            
            max86150Sensor_3.check()
            time.sleep_ms(100)
            if max86150Sensor_3.available():
                # FIFO 先进先出，从队列中取数据。都是整形int
                red3_reading = max86150Sensor_3.getFIFORed() & 0x7FFFF
                ir3_reading = max86150Sensor_3.getFIFOIR() & 0x7FFFF
                
                if red3_reading < 1000:
                    print('No finger3',red3_reading)
                    FINGER_FLAG = False  # 表示没有放手指
                    continue
                else:
                    FINGER_FLAG = True  # 表示手指已放
                if 0x7FFFF > red3_reading > 1000:
                    SENSOR3_RED = red3_reading

        if len(SENSOR4_LIST) < 10:
            # 读取第一个传感器的数据
            #选通控制Max86150模块1上电(0 1 2 3)
            A0.value(1)
            A1.value(1)
            A2.value(0)

            # 创建I2C对象(检测MAX86150)
            #定义I2C引脚
            i2c = I2C(1, sda=Pin(4), scl=Pin(5),freq=400000)  # Fast: 400kHz, slow: 100kHz

            # 创建传感器对象
            max86150Sensor_4 = MAX86150_driver2.Max86150(i2c)             
            max86150Sensor_4.check()
            time.sleep_ms(100)
            if max86150Sensor_4.available():
                # FIFO 先进先出，从队列中取数据。都是整形int
                red4_reading = max86150Sensor_4.getFIFORed() & 0x7FFFF
                ir4_reading = max86150Sensor_4.getFIFOIR() & 0x7FFFF
                
                if red4_reading < 1000:
                    print('No finger4',red4_reading)
                    FINGER_FLAG = False  # 表示没有放手指
                    continue
                else:
                    FINGER_FLAG = True  # 表示手指已放
                if 0x7FFFF > red4_reading > 1000:
                    SENSOR4_RED = red4_reading
# 
#         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(1)
#         A1.value(0)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_2 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         while red2_reading == 0:
#             if max86150Sensor_2.check():
#                 # FIFO 先进先出，从队列中取数据。都是整形int
#                 red2_reading = max86150Sensor_2.getFIFORed() & 0x7FFFF
#                 ir2_reading = max86150Sensor_2.getFIFOIR() & 0x7FFFF
# #         print(red2_reading,ir2_reading)
#             
# #         time.sleep_ms(1000)
#         
# #         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(0)
#         A1.value(1)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_3 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         while red3_reading == 0:
#             if max86150Sensor_3.check():
#                 # FIFO 先进先出，从队列中取数据。都是整形int
#                 red3_reading = max86150Sensor_3.getFIFORed() & 0x7FFFF
#                 ir3_reading = max86150Sensor_3.getFIFOIR() & 0x7FFFF
# #         print(red3_reading,ir3_reading)
#             
# #         time.sleep_ms(1000)
# 
#         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(1)
#         A1.value(1)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_4 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         while red4_reading == 0:
#             if max86150Sensor_4.check():
#                 # FIFO 先进先出，从队列中取数据。都是整形int
#                 red4_reading = max86150Sensor_4.getFIFORed() & 0x7FFFF
#                 ir4_reading = max86150Sensor_4.getFIFOIR() & 0x7FFFF
# #         print(red4_reading,ir4_reading)
#             
#         time.sleep_ms(1000)        
#         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(1)
#         A1.value(0)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_2 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         max86150Sensor_2.check()
#         if max86150Sensor_2.check() & max86150Sensor_2.available():
#             # FIFO 先进先出，从队列中取数据。都是整形int
#             red2_reading = max86150Sensor_2.getFIFORed() & 0x7FFFF
#             ir2_reading = max86150Sensor_2.getFIFOIR() & 0x7FFFF
#             print(red2_reading,ir2_reading)
#         
#         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(0)
#         A1.value(1)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_3 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         
#         if max86150Sensor_3.check() & max86150Sensor_3.available():
#             # FIFO 先进先出，从队列中取数据。都是整形int
#             red3_reading = max86150Sensor_3.getFIFORed() & 0x7FFFF
#             ir3_reading = max86150Sensor_3.getFIFOIR() & 0x7FFFF
#             print(red3_reading,ir3_reading)
#         
#         #选通控制Max86150模块1上电(0 1 2 3)
#         A0.value(1)
#         A1.value(1)
#         A2.value(0)
#         
#         # 创建传感器对象
#         max86150Sensor_4 = MAX86150_driver2.Max86150(i2c)
#         time.sleep_ms(200)
#         
#         if max86150Sensor_4.check() & max86150Sensor_4.available():
#             # FIFO 先进先出，从队列中取数据。都是整形int
#             red4_reading = max86150Sensor_4.getFIFORed() & 0x7FFFF
#             ir4_reading = max86150Sensor_4.getFIFOIR() & 0x7FFFF
#             print(red4_reading,ir4_reading)
        
#         print(red1_reading, ir1_reading, red2_reading, ir2_reading, red3_reading, ir3_reading, red4_reading, ir4_reading)
#             print('tiaoshi')
#             
#             if red_reading < 1000:
# #                 print('No finger')
#                 FINGER_FLAG = False  # 表示没有放手指
#                 continue
#             else:
#                 FINGER_FLAG = True  # 表示手指已放
# 

                
#                 # 为了防止列表过大，这里取列表的后32个元素
#                 history = history[-MAX_HISTORY:]
#                 
#                 # 提取必要数据
#                 minima, maxima = min(history), max(history)
#                 threshold_on = (minima + maxima * 3) // 4   # 3/4
#                 threshold_off = (minima + maxima) // 2      # 1/2
#                 
#                 if not beat and red_reading > threshold_on:
#                     beat = True                    
#                     t_us = ticks_diff(ticks_us(), t_start)
#                     t_s = t_us/1000000
#                     f = 1/t_s
#                     bpm = f * 60
#                     if bpm < 500:
#                         t_start = ticks_us()
#                         beats_history.append(bpm)                    
#                         beats_history = beats_history[-MAX_HISTORY:]   # 只保留最大30个元素数据
#                         SENSOR1_RED = round(sum(beats_history)/len(beats_history), 2)  # 四舍五入
#                 if beat and red_reading < threshold_off:
#                     beat = False
#             # ------------ 修改代码 -----------
#             elif len(SENSOR2_RED_LIST) < 10:
#                 # 计算血氧
#                 red_list.append(red_reading)
#                 ir_list.append(ir_reading)
#                 # 最多 只保留最新的100个
#                 red_list = red_list[-100:]
#                 ir_list = ir_list[-100:]
#                 # 计算血氧值
#                 if len(red_list) == 100 and len(ir_list) == 100:
#                     hr, hrb, sp, spb = calc_hr_and_spo2(red_list, ir_list)
#                     if hrb is True and spb is True:
#                         if sp != -999:
#                             SENSOR2_RED = int(sp)
#             # ------------ 修改代码 -----------
# #             elif len(TEMP_LIST) < 10:
# #                 # 计算温度
# #                 SENSOR3_RED = max86150Sensor_1.read_temperature()


if __name__ == '__main__':
    # 1. 创建定时器
    timer = Timer(1)
    # 2. 设置定时器的回调函数，每1秒钟调用1次display_info函数（用来显示数据）
    timer.init(period=500, mode=Timer.PERIODIC, callback=display_info)
    # 3. 调用主程序，用来检测数据
    main()

