#kannsuuwo method ni suru (ituka) Cnogakakiyasuiyone 

import gpiod
import spidev
import time
import math
import struct
import numpy as np

# Define GPIO chip and lines #################################################################################################################################
CHIP = "gpiochip4"
#CHIP = "gpiochip4" #addict 4 env 0 or 4
LINE_SVON = 2
LINE_ALARM = 1
LINE_RESET = 0

LINE_SS1 = 25
LINE_SS2 = 3

NUM_OF_MOTOR = 2

# Initialize SPI #############################################################################################################################################
spi = spidev.SpiDev()
spi.open(1, 1)  # Bus 1, Device 1  (gpio12)
spi.mode = 3 #definetly 3 kijyutuniburegaaruga 3gayoi
spi.max_speed_hz = 1000000

# Open GPIO lines ############################################################################################################################################
chip = gpiod.Chip(CHIP)#gpio settings
lines = chip.get_lines([LINE_SVON, LINE_ALARM])#gpio settings
#motors = chip.get_lines([ LINE_SS1, LINE_SS2])

# Set lines for output (except for ALARM, S1, and S2, which are inputs)
#lines.request(consumer="sv_control", type=gpiod.LINE_REQ_DIR_OUT)
#motors.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)


# Motor class to hold parameters ###################################################################################################################################
class Motor:
    #servo_mode = 0x00#class val
    
    def __init__(self, pin, SPIid, setting_pulse, pos_max, pos_min, torque_constant = 0.1):
        self.pinNumber = pin
        self.SPIid = 0
        self.setting_Pulse = setting_pulse
        self.Kp = 1
        self.Kd = 0.3
        self.rad = 0
        self.pulse = 0
        self.initialPosition = 0
        self.pos_max = pos_max
        self.pos_min = pos_min
        self.error = 0
        self.error_old = 0
        self.cd = 0
        self.cd_filtered = 0
        self.c_threshold = 1.0
        
        self.current = 0
        self.torque_constant = torque_constant
        self.alpha = 0.1  # For filtering
        self.data_packet = [0x00 for i in range(12)]
        self.check = False
        
# Initialize motors ##########################################################################################################################################
motor1 = Motor(pin=LINE_SS1, SPIid = 0, setting_pulse=1048576, pos_max=np.pi/4, pos_min=-np.pi/4)
motor2 = Motor(pin=LINE_SS2, SPIid = 1, setting_pulse=2097152, pos_max=0, pos_min=0)

servo_mode = 0x00 

def initialize_motor(motor,SPI):
    """ Initialize motor using SPI communication. """
    global servo_mode
    servo_mode = 0
    SPI.max_speed_hz = 100000
    i= 0
    while i<20:
        print(f"failed {i} time")
        #create_data_packet_current(motor,0.0)
        create_data_packet_position(motor,0.0)
        print(motor.data_packet)
        transfer_spi_data(motor, spi)  # Send zero value
        print(motor.data_packet)
        time.sleep(0.01)
        if motor.check ==True:
            i+=1
        
    motor.initialPosition = motor.pulse
    print("initial Position is ",end = "")
    print(motor.initialPosition)
        

def create_data_packet_current(motor, value):
    """ Prepare a data packet for SPI transfer. """
    value = 0.0
    SPI_RESET = 0
    DIN4  = 0
    DIN3 = 0
    DIN2 = 0
    DIN1 = 0
    # コマンドバイトを生成
    #command_byte = 0x08  # t =8 f = 0
    command_byte = 0x00  + servo_mode # t =8 f = 0

    # 送信する値を4バイトのデータとして変換
    # valueはモーターの制御対象の値（例: 電流、位置）。ここではfloatを整数に変換し、バイト列にする
    value = min(max(value, -motor.c_threshold), motor.c_threshold)  # Limit to [-1, 1]
    current_data_bytes = struct.pack('<f',value)

    # 予約バイト
    unused_data12 = [0x00, 0x00]
    unused_data89 = [0x00, 0x00]
    mode_bytes = SPI_RESET <<4 + DIN4 <<3  + DIN3 <<2 + DIN2 <<1 + DIN1

    # バイト列を生成
    data_packet = [command_byte] + unused_data12 + list(current_data_bytes) + [mode_bytes] + unused_data89

    # チェックサムの計算
    sum_value = 0
    for i in range(5):
        # 各2バイト（16ビット）ごとに合計する
        sum_value += (data_packet[2 * i + 1] << 8) + data_packet[2 * i]

    # チェックサムの下位8ビットを10バイト目に格納
    checksum_low = sum_value & 0x00FF
    data_packet.append(checksum_low)

    # チェックサムの上位7ビットと固定ビットを11バイト目に格納
    checksum_high = ((sum_value & 0x7F00) >> 8) | 0x80
    data_packet.append(checksum_high)


    #データパケットが12バイトになることを確認（もし不足している場合はゼロで埋める）
    while len(data_packet) < 12:
        data_packet = []
        for i in range(10):
            data_packet.append(0x00)

    motor.data_packet = data_packet
    # 結果を表示
    # binary_representation = ' '.join(f'{byte:08b}' for byte in data_packet)
    # print(binary_representation)


    return data_packet  # Placeholder for a 12-byte data packet

def create_data_packet_position(motor,value):
    """ Prepare a data packet for SPI transfer. pos are incriment or decriment from current position"""
    value = 0.0
    SPI_RESET = 0
    DIN4  = 0
    DIN3 = 0
    DIN2 = 0
    DIN1 = 0
    # コマンドバイトを生成
    #command_byte = 0x08  # t =8 f = 0
    command_byte = 0x00 + servo_mode # t =8 f = 0

    # 送信する値を4バイトのデータとして変換
    # valueはモーターの制御対象の値（例: 電流、位置）。ここではfloatを整数に変換し、バイト列にする
    #value = min(max(value, motor.pos_min), motor.pos_max) 
    position = motor.initialPosition #+ value #* motor.setting_Pulse / (2 * math.pi)
    pos_data_bytes = struct.pack('<i',(int)(position))

    # 予約バイト
    #unused_data12 = [0x00, 0x00]
    #unused_data89 = [0x00, 0x00]
    mode_bytes = SPI_RESET <<4 + DIN4 <<3  + DIN3 <<2 + DIN2 <<1 + DIN1

    # バイト列を生成
    data_packet = [command_byte] + [0x00, 0x00] + list(pos_data_bytes) + [mode_bytes] + [0x00, 0x00]

    # チェックサムの計算
    sum_value = 0
    for i in range(4):
        # 各2バイト（16ビット）ごとに合計する
        sum_value += (data_packet[2 * i + 1] << 8) + data_packet[2 * i]

    # チェックサムの下位8ビットを10バイト目に格納
    checksum_low = sum_value & 0x00FF
    data_packet.append(checksum_low)

    # チェックサムの上位7ビットと固定ビットを11バイト目に格納
    checksum_high = ((sum_value & 0x7F00) | 0x8000 ) >> 8
    data_packet.append(checksum_high)


    #データパケットが12バイトになることを確認（もし不足している場合はゼロで埋める）
    while len(data_packet) < 12:
        data_packet = []
        for i in range(10):
            data_packet.append(0x00)

    motor.data_packet = data_packet
    # 結果を表示
    # binary_representation = ' '.join(f'{byte:08b}' for byte in data_packet)
    # print(binary_representation)

    return data_packet  # Placeholder for a 12-byte data packet


def transfer_spi_data(motor,SPI):
    """ xfer motor data over SPI. """
    motor.check = False
    #time.sleep(0.020)
    response = SPI.xfer2(motor.data_packet)  # xfer 12 bytes
    #time.sleep(0.020)
    # Extract encoder pulses and current values from the response
    
    # チェックサムの計算
    sum_value = 0
    for i in range(5):
        # 各2バイト（16ビット）ごとに合計する
        sum_value += (response[2 * i + 1] << 8) + response[2 * i]
    # チェックサムの下位8ビットを10バイト目に格納
    sumvalue = (sum_value & 0x7FFF) | 0x8000
    checksum = response[11]<< 8 | response[10]
    if checksum ==sumvalue:
        motor.check = True
        motor.pulse = struct.unpack('<l',bytes(response[1:5]))[0]
        motor.current = struct.unpack('<f', bytes(response[5:9]))[0]
        motor.rad = (2 * math.pi * (motor.pulse - motor.initialPosition)) / motor.setting_Pulse
        
        print("success!")
        
    return response

def PDcurrent_control_motor(motor, target_angle, dt):
    """ Perform PD control for the motor. """
    motor.error_old = motor.error
    motor.error = target_angle - motor.rad
    motor.cd = motor.error * motor.Kp + (motor.error - motor.error_old) / dt * motor.Kd
    motor.cd_filtered = motor.cd * (1 - motor.alpha) + motor.cd_filtered * motor.alpha
    motor.cd_filtered = min(max(motor.cd_filtered, -1.0), 1.0)  # Limit to [-1, 1]

def control_position(motor,SPI,value):
    create_data_packet_position(motor,(motor.pulse - motor.initialPosition))
    transfer_spi_data(motor,SPI)#SERVO with current position
    
    val_pulse = value/2/np.pi * motor.setting_Pulse
    error_pulse = (motor.pulse - val_pulse)
    while error_pulse   > 100 or error_pulse < -100:
        if error_pulse  >0:
            create_data_packet_position(motor,10)# order with pulse+10
        elif error_pulse <0:
            create_data_packet_position(motor,-10)# order with pulse-10
            
        print(motor.data_packet)
        transfer_spi_data(motor,SPI)
        print(motor.data_packet)
        
    create_data_packet_position(motor,val_pulse)
    transfer_spi_data(motor,SPI)#SERVO with current position


def loop():
    """ Main control loop. """
    pre_time = time.time()
    global servo_mode
    while True:
        i= 0
        while(i<50):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            #PDcurrent_control_motor(motor1, target_angle=math.pi / 4, dt=dt)
            #PDcurrent_control_motor(motor2, target_angle=math.pi / 2, dt=dt)
            
            # Send SPI data to motors
            servo_mode = 0x08
            data1 = create_data_packet_position(motor1,0)
            #data2 = create_data_packet(motor2,motor2.cd_filtered)
            print(data1)
            data1_receive = transfer_spi_data(motor1, spi)
            #data2_receive = transfer_spi_data(motor2, data2)
            print(data1_receive )
            print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            print(motor1.initialPosition)
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            time.sleep(0.1)
            i+=1
        i=0
        while(i<50):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            #PDcurrent_control_motor(motor1, target_angle=math.pi / 4, dt=dt)
            #PDcurrent_control_motor(motor2, target_angle=math.pi / 2, dt=dt)
            
            # Send SPI data to motors
            servo_mode= 0x00
            position_d = motor1.rad * 0.9 + 0 * 0.1
            data1 = create_data_packet_position(motor1,0)
            #data2 = create_data_packet(motor2,motor2.cd_filtered)
            print(data1)
            data1_receive = transfer_spi_data(motor1, spi)
            #data2_receive = transfer_spi_data(motor2, data2)
            print(data1_receive )
            print(motor1.pulse ,end = " ")
            print(motor1.initialPosition)
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            time.sleep(0.1)
            i+=1

def end_process(motor,SPI):
    global servo_mode
    servo_mode = 0x00
    while not motor.data_packet[0] == 0:
        initialize_motor(motor,SPI)
        print(motor.data_packet)
        time.sleep(0.1)
        if motor.data_packet[0] == 4:
            break
    lines.release()
    #motors.release()
    chip.close()
    spi.close()
        
# main #######################################################################################################################################################
if __name__ == "__main__":
    # Run initialization
    initialize_motor(motor1,spi)
    #initialize_motor(motor2)
    time.sleep(3)
    
    # Start the control loop
    try:
        servo_mode = 0x00#already global
        
        loop()
    except KeyboardInterrupt:
        servo_mode = 0x00
        end_process(motor1,spi)
