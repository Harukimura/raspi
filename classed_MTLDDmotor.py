#kannsuuwo method ni suru (ituka) Cnogakakiyasuiyone 

import gpiod
import spidev
import time
import math
import struct
import numpy as np
import csv
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
#spi = spidev.SpiDev()
#spi.open(1, 1)  # Bus 1, Device 1  (gpio12)
#spi.mode = 3 #definetly 3 kijyutuniburegaaruga 3gayoi
#spi.max_speed_hz = 1000000

# Open GPIO lines ############################################################################################################################################
chip = gpiod.Chip(CHIP)#gpio settings
lines = chip.get_lines([LINE_SVON, LINE_ALARM])#gpio settings
#motors = chip.get_lines([ LINE_SS1, LINE_SS2])

# Set lines for output (except for ALARM, S1, and S2, which are inputs)
#lines.request(consumer="sv_control", type=gpiod.LINE_REQ_DIR_OUT)
#motors.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)

xfererror =0

# Motor class to hold parameters ###################################################################################################################################
class Motor:
    servo_mode = 0x00#class val
    g = [0.0,-9.81]
    def __init__(self, pin, SPIid, setting_pulse, pos_max, pos_min, spi_bus,spi_device,torque_constant = 0.1):
        self.pinNumber = pin
        self.SPIid = 0
        self.SPI = spidev.SpiDev()
        self.SPI.open(spi_bus, spi_device)  # Bus 1, Device 1  (gpio12)
        self.SPI.mode = 3 #definetly 3 kijyutuniburegaaruga 3gayoi
        self.SPI.max_speed_hz = 300000
        
        self.setting_Pulse = setting_pulse
        self.Kp = 1
        self.Kd = 0.3
        self.rad = 0
        self.old_rad = 0
        self.radxy =0
        self.pulse = 0
        self.initialPosition = 0
        self.initialPosition_rad = 0
        self.pos_max = pos_max
        self.pos_min = pos_min
        self.error = 0
        self.error_old = 0
        self.c_old = 0
        self.cd = 0
        self.cd_filtered = 0
        self.c_threshold = 1.0
        
        self.current = 0
        self.torque_constant = torque_constant
        self.alpha = 0.7 # For filtering
        self.data_packet = [0x00 for i in range(12)]
        self.check = False
        
    def create_data_packet_current(self, value):
        """ Prepare a data packet for SPI transfer. """
        #value = 0.0
        SPI_RESET = 0
        DIN4  = 0
        DIN3 = 0
        DIN2 = 0
        DIN1 = 0
        # コマンドバイトを生成
        #command_byte = 0x08  # t =8 f = 0
        command_byte = 0x00  + Motor.servo_mode # t =8 f = 0

        # 送信する値を4バイトのデータとして変換
        # valueはモーターの制御対象の値（例: 電流、位置）。ここではfloatを整数に変換し、バイト列にする
        value = min(max(value, -self.c_threshold), self.c_threshold)  # Limit to [-1, 1]
        current_data_bytes = struct.pack('<f',np.float32(value))

        # 予約バイト
        unused_data12 = [0x00, 0x00]
        unused_data89 = [0x00, 0x00]
        mode_bytes = SPI_RESET <<4 + DIN4 <<3  + DIN3 <<2 + DIN2 <<1 + DIN1

        # バイト列を生成
        data_packet_temp = [command_byte] + unused_data12 + list(current_data_bytes) + [mode_bytes] + unused_data89

        # チェックサムの計算
        sum_value = 0
        for i in range(5):
            # 各2バイト（16ビット）ごとに合計する
            sum_value += (data_packet_temp[2 * i + 1] << 8) + data_packet_temp[2 * i]

        # チェックサムの下位8ビットを10バイト目に格納
        checksum_low = sum_value & 0x00FF
        data_packet_temp.append(checksum_low)

        # チェックサムの上位7ビットと固定ビットを11バイト目に格納
        checksum_high = ((sum_value & 0x7F00) >> 8) | 0x80
        data_packet_temp.append(checksum_high)


        #データパケットが12バイトになることを確認（もし不足している場合はゼロで埋める）
        while len(data_packet_temp) < 12:
            data_packet_temp = []
            for i in range(10):
                data_packet_temp.append(0x00)

        self.data_packet = data_packet_temp
        # 結果を表示
        # binary_representation = ' '.join(f'{byte:08b}' for byte in data_packet)
        # print(binary_representation)


        return self.data_packet # Placeholder for a 12-byte data packet

    def create_data_packet_position(self,value):
        """ Prepare a data packet for SPI transfer. pos are incriment or decriment from current position"""
        #value = 10.0
        SPI_RESET = 0
        DIN4  = 0
        DIN3 = 0
        DIN2 = 0
        DIN1 = 0
        # コマンドバイトを生成
        #command_byte = 0x08  # t =8 f = 0
        command_byte = 0x00 + Motor.servo_mode # t =8 f = 0

        # 送信する値を4バイトのデータとして変換
        # valueはモーターの制御対象の値（例: 電流、位置）。ここではfloatを整数に変換し、バイト列にする
        #value = min(max(value, motor.pos_min), motor.pos_max) 
        position = value #+ value #* motor.setting_Pulse / (2 * math.pi)
        print(type(position),end = "")
        print(position,end = "")
        pos_data_bytes = struct.pack('<i',(int)(position))
        print(pos_data_bytes)
        
        #position = (int)(motor.initialPosition + value)
        #print(type(position),end = "")
        #print(position,end = "")
        #pos_data_bytes = struct.pack('<i',(int)(position))
        #print(pos_data_bytes)
        
        # 予約バイト
        #unused_data12 = [0x00, 0x00]
        #unused_data89 = [0x00, 0x00]
        mode_bytes = SPI_RESET <<4 + DIN4 <<3  + DIN3 <<2 + DIN2 <<1 + DIN1

        # バイト列を生成
        data_packet_temp = [command_byte] + [0x00, 0x00] + list(pos_data_bytes) + [mode_bytes] + [0x00, 0x00]

        # チェックサムの計算
        sum_value = 0
        for i in range(4):
            # 各2バイト（16ビット）ごとに合計する
            sum_value += (data_packet_temp[2 * i + 1] << 8) + data_packet_temp[2 * i]

        # チェックサムの下位8ビットを10バイト目に格納
        checksum_low = sum_value & 0x00FF
        data_packet_temp.append(checksum_low)

        # チェックサムの上位7ビットと固定ビットを11バイト目に格納
        checksum_high = ((sum_value & 0x7F00) | 0x8000 ) >> 8
        data_packet_temp.append(checksum_high)


        #データパケットが12バイトになることを確認（もし不足している場合はゼロで埋める）
        while len(data_packet_temp) < 12:
            data_packet_temp = []
            for i in range(10):
                data_packet_temp.append(0x00)

        self.data_packet = data_packet_temp
        # 結果を表示
        # binary_representation = ' '.join(f'{byte:08b}' for byte in data_packet)
        # print(binary_representation)

        return data_packet_temp  # Placeholder for a 12-byte data packet
        
    def transfer_spi_data(self):
        """ xfer motor data over SPI. """
        self.check = False
        #time.sleep(0.020)
        response = self.SPI.xfer2(self.data_packet,300000,10)  # xfer 12 bytes bss138 <300kHz
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
            self.check = True
            self.pulse = struct.unpack('<l',bytes(response[1:5]))[0]
            self.current = struct.unpack('<f', bytes(response[5:9]))[0]
            self.old_rad = self.rad
            self.rad = self.alpha * (2 * math.pi * self.pulse) / self.setting_Pulse  + (1-self.alpha) * self.old_rad#filtered
            #self.rad = (2 * math.pi * (self.pulse - self.initialPosition)) / self.setting_Pulse
            #print("xfer success!")
        else:
            #print(f" {self.pinNumber} xfer failed!")
            global xfererror
            xfererror=xfererror+1
        return response    
        
    def initialize_motor_p(self):
        """ Initialize motor using SPI communication. """
        Motor.servo_mode = 0x00
        self.SPI.max_speed_hz = 100000
        i= 0
        while i<100:
            i+=1
            #print(f"failed {i} time")
            #create_data_packet_current(motor,0.0)
            value = self.pulse
            self.create_data_packet_position(value)
            print(self.data_packet)
            self.transfer_spi_data()  # Send zero value
            print(self.data_packet)
            time.sleep(0.01)
            if self.check == False:
                print(f"failed {i} time")
                i-=1
            
        self.initialPosition = self.pulse
        self.initialPosition_rad = self.rad
        print("initial Position is ",end = "")
        print(self.initialPosition)
        print("initial Position is ",end = "")
        print(self.initialPosition_rad)
        
    def initialize_motor_c(self):
        """ Initialize motor using SPI communication. """
        Motor.servo_mode = 0x00
        self.SPI.max_speed_hz = 1000000
        i= 0
        while i<100:
            i+=1
            #print(f"failed {i} time")
            #create_data_packet_current(motor,0.0)
            self.create_data_packet_current(0.0)
            print(self.data_packet)
            self.transfer_spi_data()  # Send zero value
            print(self.data_packet)
            #time.sleep(0.01)
            if self.check == False:
                print(f"failed {i} time")
                i-=1
            
        self.initialPosition = self.pulse
        self.initialPosition_rad = self.rad
        print("initial Position is ",end = "")
        print(self.initialPosition)
        print("initial Position is ",end = "")
        print(self.initialPosition_rad)
        
    def end_process_c(self):
        self.servo_mode = 0x00
        while not self.data_packet[0] == 0:
            Motor.servo_mode = 0x00
            self.SPI.max_speed_hz = 100000
            i= 0
            while i<5:
                #print(f"end command {i} time")
                #create_data_packet_current(motor,0.0)
                self.create_data_packet_current(0.0)
                #print(self.data_packet)
                self.transfer_spi_data()  # Send zero value
                #print(self.data_packet)
                time.sleep(0.01)
                if self.check == True:
                    i+=1
            print(self.data_packet)
            time.sleep(0.1)
            if self.data_packet[0] == 4:
                break
        self.SPI.close()
        
    def end_process_p(self):
        self.servo_mode = 0x00
        while not self.data_packet[0] == 0:
            Motor.servo_mode = 0x00
            self.SPI.max_speed_hz = 100000
            i= 0
            while i<5:
                #print(f"failed {i} time")
                #create_data_packet_current(motor,0.0)
                value = self.pulse
                self.create_data_packet_position(value)
                #print(self.data_packet)
                self.transfer_spi_data()  # Send zero value
                #print(self.data_packet)
                time.sleep(0.01)
                if self.check == True:
                    i+=1
            print(self.data_packet)
            time.sleep(0.1)
            if self.data_packet[0] == 4:
                break
        self.SPI.close()
        
class adoptiveModel:
    def __init__(self,dimension,model_zero,threshold):
        self.dim = dimension
        self.model = np.array(model_zero)
        self.threshold = np.array(threshold)
    def modelUpdate(self,val):
        for i in range(self.dim):
            if val[i] < self.threshold[i][0]:
                self.model[i] = self.threshold[i][0]
            elif val[i] > self.threshold[i][1]:
                self.model[i] = self.threshold[i][1]
            else:
                self.model[i] = val[i]
    # 動作確認
    #M = adoptiveModel(dimension=2, model_zero=[0.4, 0.4], threshold=[[0.3, 0.5], [0.3, 0.5]])
    #M.modelUpdate([0.2, 0.6])  # 更新後 self.model は [0.3, 0.5]
    #print(M.model)
    
# Initialize motors ##########################################################################################################################################
motor1 = Motor(pin=18, SPIid = 0, setting_pulse=1048576, pos_max=np.pi/4, pos_min=-np.pi/4,spi_bus =1,spi_device = 0)#
motor2 = Motor(pin=17, SPIid = 1, setting_pulse=1048576, pos_max=np.pi/4, pos_min=-np.pi/4,spi_bus =1,spi_device = 1)#


def rad2radxy(motor_a,motor_b):
    motor_a.radxy =  motor_a.initialPosition_rad - motor_a.rad + np.pi/2#CW minus CCW plus
    motor_b.radxy = motor_b.rad - motor_b.initialPosition_rad
    return 0

def gravity_compensation(motor_a,motor_b):
    #memo トルクの次元ではモデル通りに考えて良い　トルクから電流に変換するときに回転方向は直す
    M = [[0.0576, 0.0], [0.0, 0.0570]]
    l1 = 0.1
    lee = 0.1
    l2 = 0.03
    rad_a =  motor_a.initialPosition_rad - motor_a.rad + np.pi/2#CW minus
    rad_b = motor_b.rad - motor_b.initialPosition_rad
    #rad_a = q[0]
    #rad_b = q[1]
    jt1 = np.array([[0,-1/2*l2*np.sin(rad_b)],[0,1/2*l2*np.cos(rad_b)]])
    jt2 = np.array([[-l1*np.sin(rad_a),-1/2*l2*np.sin(rad_b)],[l1*np.cos(rad_a),1/2*l2*np.cos(rad_b)]])
    jt3 = np.array([[-1/2*l1*np.sin(rad_a),-l2*np.sin(rad_b)],[1/2*l1*np.sin(rad_a),l2*np.sin(rad_b)]])
    jt4 = np.array([[-1/2*l1*np.sin(rad_a),0],[1/2*l1*np.sin(rad_a),0]])
    Jte = [[l1*np.sin(rad_a), l1*np.cos(rad_a)],[lee*np.sin(rad_b), -lee*np.cos(rad_b)]]
    #TauConstants = [[1/motor_a.torque_constant,0],[0,1/motor_b.torque_constant]]
    #return - (np.dot( Jte,np.dot(M,Motor.g)) )#only mass point model
    #CWplus with current
    return  -(np.dot( Jte,np.dot(M,Motor.g)) +np.dot( jt1,np.dot(0.001,Motor.g))+np.dot( jt2,np.dot(0.001,Motor.g)) +np.dot( jt3,np.dot(0.0036+0.001,Motor.g)) +np.dot( jt4,np.dot(0.003,Motor.g)) )
    #return  -np.dot(fixer,(np.dot( Jte,np.dot(M,Motor.g)) +np.dot( jt1,np.dot(0.001,Motor.g))+np.dot( jt2,np.dot(0.001,Motor.g)) +np.dot( jt3,np.dot(0.0036+0.001,Motor.g)) +np.dot( jt4,np.dot(0.003,Motor.g)) ))

def adoptive_gravity_compensation(motor_a,motor_b,model,errors,target_angles):
    #print(model)
    rad2radxy(motor_a, motor_b)
    Ginv = np.array([[0.001, 0],[0, 0.001]])
    #Yqd = np.array([[np.cos(target_angles[0]), 0],[0, np.cos(target_angles[1])]])#target?
    Yqd = np.array([[np.cos(motor_a.radxy), 0],[0, np.cos(motor_b.radxy)]])
    #model = model  - Ginv @ Yqd.T @ errors
    model.modelUpdate(model.model  - Ginv @ Yqd.T @ errors)
    
    
    #4 dimension
    #Ginv = np.array([[0.001, 0,0,0],[0, 0.001, 0,0],[0,0, 0.001, 0],[0,0,0, 0.001]])
    #errors2to4 = errors @ np.array([[1,1,0,0],[0,0,1,1]])
    #selectionMatrix = np.array([[1,1,0,0],[0,0,1,1]])
    #Yqd = np.array([[np.sin(target_angles[0]), 0,0,0],[0,np.cos(target_angles[0]), 0,0],[0,0,np.sin(target_angles[1]), 0],[0,0,0, np.cos(target_angles[1])]])#target?
    #model = model  - Ginv @ Yqd.T @ errors2to4
    
    #print(model)
    return  np.dot(Yqd,model.model.T),model #normal
    #return  np.dot(selectionMatrix,np.dot(Yqd,model.T)),model #4dmodel

def joint_impedance_control(motor,target_angle,target_velocity,theta_d_ddot,dt):
    
    error = target_angle - motor.rad
    error_dot = target_velocity - (motor.rad - motor.old_rad)/dt
    
    M = [[0.00018, 0.0], [0.0, 0.00018]]  #慣性mll 0.018*0.1*0.1
    B = [[0.05, 0.0], [0.0, 0.05]]  #粘性
    K = [[1.0, 0.0], [0.0, 1.0]]  #剛性
    #theta_d_ddot = [0,0]
    tau = (np.dot(M, theta_d_ddot) + np.dot(B, error_dot) + np.dot(K, error))
    #print(f"tau:{tau}")
    #print(f"error:{error}")
    #print(f"ed:{error_dot}")
    cd = tau/motor.torque_constant#current
    #cdf = min((max(1-0.9) * motor.current + 0.9*cd , -motor.c_threshold), motor.c_threshold)
    #cdf = 0.7 * motor.c_old + (1-0.7)*cd
    cdf = cd
    cdf_cut = min(max(cdf[0,0],-1 * motor.c_threshold),motor.c_threshold)
    return cdf_cut
    
def joint_impedance_control_2DOF(motor_a, motor_b, target_angles, target_velocities, dt):
    #memo a-initial = default b-ini = default
    errors = [(motor_a.initialPosition_rad -motor_a.rad) +np.pi/2  -target_angles[0] , (motor_b.rad - motor_b.initialPosition_rad) - target_angles[1]]
    error_dots =[ -(motor_a.rad - motor_a.old_rad)/dt -target_velocities[0], (motor_b.rad - motor_b.old_rad)/dt -target_velocities[1]]
    #theta_d_ddots = [0.0,0.0]
    #M = [[0.00018, 0.0], [0.0, 0.00018]]  #慣性mll 0.018*0.1*0.1
    C = [[0.02, 0.0], [0.0, 0.02]]  #粘性
    K = [[0.5, 0.0], [0.0, 0.5]]  #剛性
    #theta_d_ddot = [0,0]
    tau =  -np.dot(C, error_dots) -np.dot(K, errors)  + gravity_compensation(motor_a, motor_b)#+ (np.dot(M, theta_d_ddots)
    #print(f"tau:{tau}")
    #print(f"error:{error}")
    #print(f"ed:{error_dot}")
    cd1 = -tau[0]/motor_a.torque_constant#torque2 current motor setting CW = current>0
    cd2 = tau[1]/motor_b.torque_constant#torque2current motor setting CW = current<0
    #cdf = min((max(1-0.9) * motor.current + 0.9*cd , -motor.c_threshold), motor.c_threshold)
    #cdf = 0.7 * motor.c_old + (1-0.7)*cd
    cd1f_cut = min(max(cd1,-1 * motor_a.c_threshold),motor_a.c_threshold)
    cd2f_cut = min(max(cd2,-1 * motor_b.c_threshold),motor_b.c_threshold)
    return cd1f_cut,cd2f_cut
    
def adoptive_joint_impedance_control_2DOF(motor_a, motor_b, target_angles, target_velocities, dt, model):
    #memo a-initial = default b-ini = default
    errors = [(motor_a.initialPosition_rad -motor_a.rad) +np.pi/2  -target_angles[0] , (motor_b.rad - motor_b.initialPosition_rad) - target_angles[1]]
    error_dots =[ -(motor_a.rad - motor_a.old_rad)/dt -target_velocities[0], (motor_b.rad - motor_b.old_rad)/dt -target_velocities[1]]
    #theta_d_ddots = [0.0,0.0]
    #M = [[0.00018, 0.0], [0.0, 0.00018]]  #慣性mll 0.018*0.1*0.1
    C = [[0.02, 0.0], [0.0, 0.02]]  #粘性
    K = [[0.5, 0.0], [0.0, 0.5]]  #剛性
    #theta_d_ddot = [0,0]
    ga, model= adoptive_gravity_compensation(motor_a, motor_b, model,errors,target_angles)
    tau =  -np.dot(C, error_dots) -np.dot(K, errors)  + ga #+ (np.dot(M, theta_d_ddots)
    #print(f"tau:{tau}")
    #print(f"error:{error}")
    #print(f"ed:{error_dot}")
    cd1 = -tau[0]/motor_a.torque_constant#current CCW to CW
    cd2 = tau[1]/motor_b.torque_constant#current
    #cdf = min((max(1-0.9) * motor.current + 0.9*cd , -motor.c_threshold), motor.c_threshold)
    #cdf = 0.7 * motor.c_old + (1-0.7)*cd
    cd1f_cut = min(max(cd1,-1 * motor_a.c_threshold),motor_a.c_threshold)
    cd2f_cut = min(max(cd2,-1 * motor_b.c_threshold),motor_b.c_threshold)
    return [cd1f_cut,cd2f_cut,ga,errors,model]

def workspace_impedance_control_2DOF(motor_a, motor_b, target_position, target_velocities, dt):
    
    l1 = 0.1
    lee = 0.1
    rad_a =  motor_a.initialPosition_rad - motor_a.rad + np.pi/2#CW minus
    rad_b = motor_b.rad - motor_b.initialPosition_rad
    #Jte = [[-0.1*np.sin(target_position[0]), 0.1*np.cos(target_position[0])],[0.1*np.sin(target_position[1]), -0.1*np.cos(target_position[1])]]
    Jte = [[-0.1*np.sin(rad_a), 0.1*np.cos(rad_a)],[0.1*np.sin(rad_b), -0.1*np.cos(rad_b)]]
    pos = [l1*np.cos(rad_a) - lee*np.cos(rad_b), l1*np.sin(rad_a) - lee*np.sin(rad_b)]
    errors = [pos[0] - target_position[0],  pos[1] - target_position[1]]
    error_dots =[ -(motor_a.rad - motor_a.old_rad)/dt - target_velocities[0] , (motor_b.rad - motor_b.old_rad)/dt - target_velocities[1]]
    #theta_d_ddots = [0.0,0.0]
    #M = [[0.00018, 0.0], [0.0, 0.00018]]  #慣性mll 0.018*0.1*0.1
    C = [[0.0005, 0.0], [0.0, 0.0005]]  #粘性
    K = [[30, 0.0], [0.0, 30]]  #剛性
    #theta_d_ddot = [0,0]
    tau =   -np.dot(C, error_dots) - np.dot( Jte,np.dot(K, errors))  + gravity_compensation(motor_a, motor_b) #+ (np.dot(M, theta_d_ddots)
    #print(f"tau:{tau}")
    #print(f"error:{errors}")
    #print(f"ed:{error_dot}")
    cd1 = -tau[0]/motor_a.torque_constant#current
    cd2 = tau[1]/motor_b.torque_constant#current
    #cdf = min((max(1-0.9) * motor.current + 0.9*cd , -motor.c_threshold), motor.c_threshold)
    #cdf = 0.7 * motor.c_old + (1-0.7)*cd
    cd1f_cut = min(max(cd1,-1 * motor_a.c_threshold),motor_a.c_threshold)
    cd2f_cut = min(max(cd2,-1 * motor_b.c_threshold),motor_b.c_threshold)
    return cd1f_cut,cd2f_cut,errors[0],errors[1]

def PDcurrent_control_motor(motor, target_angle, dt):
    """ Perform PD control for the motor. """
    motor.error_old = motor.error
    motor.error = target_angle - motor.rad
    motor.cd = motor.error * motor.Kp + (motor.error - motor.error_old) / dt * motor.Kd
    motor.cd_filtered = motor.cd * (1 - motor.alpha) + motor.cd_filtered * motor.alpha
    motor.cd_filtered = min(max(motor.cd_filtered, -1.0), 1.0)  # Limit to [-1, 1]

def control_position(motor,value):
    create_data_packet_position(motor,(motor.pulse))
    transfer_spi_data(motor)#SERVO with current position
    
    val_pulse = value/2/np.pi * motor.setting_Pulse
    error_pulse = (motor.pulse - val_pulse)
    while error_pulse   > 100 or error_pulse < -100:
        if error_pulse  >0:
            create_data_packet_position(motor,int(motor.pulse +10))# order with pulse+10
        elif error_pulse <0:
            create_data_packet_position(motor,int(motor.pulse -10))# order with pulse-10
            
        print(motor.data_packet)
        transfer_spi_data(motor)
        print(motor.data_packet)
        
    create_data_packet_position(motor,val_pulse)
    transfer_spi_data(motor)#SERVO with current position

########################################################################
def loop():
    """ Main control loop. """
    pre_time = time.time()
    time.sleep(0.1)
    while True:
        for i in range(50):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            #PDcurrent_control_motor(motor1, target_angle=math.pi / 4, dt=dt)
            #PDcurrent_control_motor(motor2, target_angle=math.pi / 2, dt=dt)
            
            # Send SPI data to motors
            Motor.servo_mode = 0x00
            data1 = motor1.create_data_packet_current(0.0)
            #data2 = create_data_packet(motor2,motor2.cd_filtered)
            #print(data1)
            data1_receive = motor1.transfer_spi_data()
            #data2_receive = transfer_spi_data(motor2, data2)
            #print(data1_receive )
            print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            print(motor1.initialPosition)
            print(joint_impedance_control(motor1,motor1.initialPosition_rad,0.0,dt))
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            time.sleep(0.1)
        for i in range(1):
            # Calculate loop duration
            
            # Update motor control
            #PDcurrent_control_motor(motor1, target_angle=math.pi / 4, dt=dt)
            #PDcurrent_control_motor(motor2, target_angle=math.pi / 2, dt=dt)
            
            # Send SPI data to motors
            servo_mode= 0x00
            data1 = motor1.create_data_packet_position(motor1.pulse -200)
            #data2 = create_data_packet(motor2,motor2.cd_filtered)
            #print(data1)
            data1_receive = motor1.transfer_spi_data()
            #data2_receive = transfer_spi_data(motor2, data2)
            #print(data1_receive )
            print(motor1.pulse ,end = " ")
            print(motor1.initialPosition)
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            time.sleep(0.1)
            
def loop_impedance():
    """ Main control loop. """
    pre_time = time.time()
    time.sleep(0.1)
    tim = time.time()
    datalist1 = []
    datalist2 = []
    while True:
        tim = time.time()
        datalist = []
        
        for i in range(1000):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            Motor.servo_mode = 0x08
            
            val1 = joint_impedance_control(motor1,motor1.initialPosition_rad,0.0,0.0,dt)
            data1 = motor1.create_data_packet_current(val1)
            data1_receive = motor1.transfer_spi_data()
            
            val2 = joint_impedance_control(motor2,motor2.initialPosition_rad,0.0,9.81,dt)
            data2 = motor2.create_data_packet_current(val2)
            data2_receive = motor2.transfer_spi_data()
            #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
            datalist.append([pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition_rad,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition_rad,val2])
            """
            print(data1)
            print(data2)
            """
            #print(data1_receive )
            #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            #print(motor1.initialPosition)
            #print(val1)
            
            #print(data2_receive )
            #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
            #print(motor2.initialPosition)
            #print(val2)
            
            #print("")
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            #time.sleep(0.01)
        print(tim -pre_time)
        with open(fname,"a",newline = "") as file:
            writer = csv.writer(file)
            writer.writerows(datalist)

def loop_gravity_compensation():
    """ Main control loop. """
    pre_time = time.time()
    time.sleep(0.1)
    tim = time.time()
    datalist1 = []
    datalist2 = []
    while True:
        tim = time.time()
        datalist = []

        for i in range(1000):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            Motor.servo_mode = 0x08
            
            val = gravity_compensation(motor1, motor2)
            #print(val)
            val = np.dot([[-1/motor1.torque_constant,0],[0,1/motor2.torque_constant]],val)
            data1 = motor1.create_data_packet_current(val[0])
            data1_receive = motor1.transfer_spi_data()
            
            data2 = motor2.create_data_packet_current(val[1])
            data2_receive = motor2.transfer_spi_data()
            #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
            datalist.append([pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition_rad,val[0],motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition_rad,val[1]])
            """
            print(data1)
            print(data2)
            """
            #print(data1_receive )
            #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            #print(motor1.initialPosition)
            #print(val1)
            
            #print(data2_receive )
            #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
            #print(motor2.initialPosition)
            #print(val2)
            
            #print("")
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            #time.sleep(0.01)
        #print(gravity_compensation(motor1,motor2))
        print(tim -pre_time)
        """
        with open(fname,"a",newline = "") as file:
            writer = csv.writer(file)
            writer.writerows(datalist)
        """
        

def loop_impedance2():
    """ Main control loop. """
    data_label = ['time','dt','pulse','rad','current','rad_target','val_target','pulse','rad','current','rad_target','val_target','Gc0','Gc1',"E1","E2"]
    userNamedf= "40b50gjpdgC"
    fname = "datas/" +time.strftime("%Y-%m-%d %H：%M：%S" + userNamedf ,time.localtime()) + ".csv"
    with open(fname,"w",newline = "") as file:
        writer = csv.writer(file)
        writer.writerow(data_label)
        
    pre_time = time.time()#initialize
    time.sleep(0.1)
    tim = time.time()
    datalist = []
    motions = [[np.pi/2,0],[np.pi/4+np.pi/2,0],[+np.pi,np.pi/2],[np.pi/4+np.pi/2, np.pi/4],[np.pi/4, -np.pi/4],[0,-np.pi/2]]
    for i in range(6):
        tim = time.time()
        datalist = []
        targetAngle = [+ motions[i][0], motions[i][1]]
        targetVelocities = [0.0, 0.0]
        for i in range(10000):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            Motor.servo_mode = 0x08
            valg = np.dot([[1/motor1.torque_constant, 0],[0, 1/motor2.torque_constant]], gravity_compensation(motor1, motor2))
            errors = [(motor1.initialPosition_rad +np.pi/2 -motor1.rad) -targetAngle[0] , (motor2.rad - motor2.initialPosition_rad) - targetAngle[1]]#4 observation
            val = joint_impedance_control_2DOF(motor1, motor2, targetAngle, targetVelocities, dt)
            data1 = motor1.create_data_packet_current(val[0])
            data2 = motor2.create_data_packet_current(val[1])
            
            data1_receive = motor1.transfer_spi_data()
            #time.sleep(0.00001)
            
            data2_receive = motor2.transfer_spi_data()
            #time.sleep(0.00001)
            
            rad2radxy(motor1,motor2)
            #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
            datalist.append([pre_time,dt,motor1.pulse,motor1.radxy,motor1.current,targetAngle[0],-val[0],motor2.pulse,motor2.radxy,motor2.current,targetAngle[1],val[1],valg[0],valg[1], errors[0], errors[1] ])
            
            """
            print(data1)
            print(data2)
            """
            #print(data1_receive )
            #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            #print(motor1.initialPosition)
            #print(val1)
            
            #print(data2_receive )
            #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
            #print(motor2.initialPosition)
            #print(val2)
            
            #print("")
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            #time.sleep(0.01)
        print(tim -pre_time)
        with open(fname,"a",newline = "") as file:
            writer = csv.writer(file)
            writer.writerows(datalist)
            
def loop_impedance_adoptive():
    """ Main control loop. """
    data_label = ['time','dt','rad','rad_target','current','val_target','rad','rad_target','current','val_target','adaptive1','adaptive2',"E1","E2","model1","model2"]
    userNamedf= "40adoptivem1"
    fname = "datas/" +time.strftime("%Y-%m-%d %H：%M：%S" + userNamedf ,time.localtime()) + ".csv"
    with open(fname,"w",newline = "") as file:
        writer = csv.writer(file)
        writer.writerow(data_label)
        
    Model = np.array( [-0.04*Motor.g[1]*0.1, 0.04* Motor.g[1]*0.1 ]) #normal 
    #Model = np.array( [0,-0.04*Motor.g[1]*0.1, 0, 0.04* Motor.g[1]*0.1 ])
    pre_time = time.time()#initialize
    tim = time.time()
    datalist = []
    #motions = [[np.pi/2,0],[np.pi/4+np.pi/2,0],[+np.pi,np.pi/2],[np.pi/4+np.pi/2, np.pi/4],[np.pi/4, -np.pi/4],[0,-np.pi/2],[0,-np.pi/2]
    motions = [[np.pi/4, 0], [np.pi/4,0],[np.pi/20.0*6 , 0], [np.pi/20.0*7 , 0],[np.pi/20.0*8 , 0],[np.pi/20.0*9 , 0],[np.pi/20.0*10 , 0],[np.pi/20.0*11 , 0],[np.pi/20.0*12 , 0],[np.pi/20.0*13 , 0],[np.pi/20.0*14 , 0],[np.pi/20.0*15 , 0]]
    targetVelocities = [0.0, 0.0]
    Motor.servo_mode = 0x08
    num_of_loop =100000
    
    for i in range(12):
        tim = time.time()
        targetAngle = [motions[i][0],motions[i][1]]#choose one 
        
        for j in range(num_of_loop):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            errors = [(motor1.initialPosition_rad +np.pi/2 -motor1.rad) -targetAngle[0] , (motor2.rad - motor2.initialPosition_rad) - targetAngle[1]]#4 observation            
            val1,val2,gc,Model = adoptive_joint_impedance_control_2DOF(motor1, motor2, targetAngle, targetVelocities, dt, Model)
            
            data1 = motor1.create_data_packet_current(val1)
            data1_receive = motor1.transfer_spi_data()
            data2 = motor2.create_data_packet_current(val2)
            data2_receive = motor2.transfer_spi_data()
            
            rad2radxy(motor1,motor2)
            #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
            datalist.append([pre_time,dt,motor1.radxy,targetAngle[0],motor1.current,val1,motor2.radxy,targetAngle[1],motor2.current,val2,-gc[0]*10,gc[1]*10,errors[0],errors[1],Model[0],Model[1]])
            #datalist.append([pre_time,dt,motor1.radxy,targetAngle[0],motor1.current,val1,motor2.radxy,targetAngle[1],motor2.current,val2,-gc[0]*10,gc[1]*10,Model[0],Model[1],Model[2],Model[3]])#4d
            """
            print(data1)
            print(data2)
            """
            #print(data1_receive )
            #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            #print(motor1.initialPosition)
            #print(val1)
            
            #print(data2_receive )
            #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
            #print(motor2.initialPosition)
            #print(val2)
            
            #print("")
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            #time.sleep(0.01)
    print(tim -pre_time)
    with open(fname,"a",newline = "") as file:
        writer = csv.writer(file)
        writer.writerows(datalist)
        
        
def loop_impedance_adoptive_slow():
    """ Main control loop. """
    data_label = ['time','dt','rad','rad_target','current','val_target','rad','rad_target','current','val_target','adaptive1','adaptive2',"E1","E2","model1","model2"]
    userNamedf= "40adoptivem1"
    fname = "datas/" +time.strftime("%Y-%m-%d %H：%M：%S" + userNamedf ,time.localtime()) + ".csv"
    with open(fname,"w",newline = "") as file:
        writer = csv.writer(file)
        writer.writerow(data_label)
        
    Model = adoptiveModel(dimension = 2,model_zero = [-0.04*Motor.g[1]*0.1, 0.04* Motor.g[1]*0.1 ], threshold = [[0.0, 0.1],[-0.1, -0.01]])
    #np.array( [-0.04*Motor.g[1]*0.1, 0.04* Motor.g[1]*0.1 ]) #normal 
    #Model = np.array( [0,-0.04*Motor.g[1]*0.1, 0, 0.04* Motor.g[1]*0.1 ])
    pre_time = time.time()#initialize
    tim = time.time()
    datalist = []
    #motions = [[np.pi/2,0],[np.pi/4+np.pi/2,0],[+np.pi,np.pi/2],[np.pi/4+np.pi/2, np.pi/4],[np.pi/4, -np.pi/4],[0,-np.pi/2],[0,-np.pi/2]
    motions = [[np.pi/2,0], [np.pi/4, 0], [np.pi/2,0], [np.pi/4 +np.pi/2, 0], [np.pi/2, 0]]
    targetVelocities = [0.0, 0.0]
    Motor.servo_mode = 0x08
    num_of_loop =30000
    
    for i in range(2):
        tim = time.time()
        nowtargetAngle = [+ motions[i][0], motions[i][1]]
        nexttargetAngle = [+ motions[i+1][0], motions[i+1][1]]
        #targetAngle = [motions[i][0],motions[i][1]]#choose one 
        dt = time.time() - pre_time
        pre_time = time.time()
        
        for j in range(num_of_loop):
            # Calculate loop duration
            targetAngle = [ nowtargetAngle[0]*(float)((num_of_loop -j)/num_of_loop) + nexttargetAngle[0]*(float)(j/num_of_loop), nowtargetAngle[1]*(float)((num_of_loop -j)/num_of_loop) + nexttargetAngle[1]*(float)(j/num_of_loop)]#choose one 
            print(pre_time-tim)
            
            for k in range(5):
                #for target Angle of j loop
                dt = time.time() - pre_time
                pre_time = time.time()
                val1, val2, gc, errors, Model = adoptive_joint_impedance_control_2DOF(motor1, motor2, targetAngle, targetVelocities, dt, Model)
                
                data1 = motor1.create_data_packet_current(val1)
                data1_receive = motor1.transfer_spi_data()
                data2 = motor2.create_data_packet_current(val2)
                data2_receive = motor2.transfer_spi_data()
                
                rad2radxy(motor1,motor2)
                #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
                datalist.append([pre_time,dt,motor1.radxy,targetAngle[0],motor1.current,val1,motor2.radxy,targetAngle[1],motor2.current,val2,-gc[0]*10,gc[1]*10,errors[0],errors[1],Model.model[0],Model.model[1]])
                #datalist.append([pre_time,dt,motor1.radxy,targetAngle[0],motor1.current,val1,motor2.radxy,targetAngle[1],motor2.current,val2,-gc[0]*10,gc[1]*10,Model[0],Model[1],Model[2],Model[3]])#4d
                """
                print(data1)
                print(data2)
                """
                #print(data1_receive )
                #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
                #print(motor1.initialPosition)
                #print(val1)
                
                #print(data2_receive )
                #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
                #print(motor2.initialPosition)
                #print(val2)
                
                #print("")
                #print(motor1.current )
                # Delay for a short while (example loop rate)
                #time.sleep(0.01)
        
    with open(fname,"a",newline = "") as file:
        writer = csv.writer(file)
        writer.writerows(datalist)            
            
def loop_workspace():
    data_label = ['time','dt','xd','yd','Ex','Ey','current1','current2','val_target1','val_target2','GC0','GC1']
    userNamedf= "40b50gWSpd"
    fname = "datas/" +time.strftime("%Y-%m-%d %H：%M：%S" + userNamedf ,time.localtime()) + ".csv"

    with open(fname,"w",newline = "") as file:
        writer = csv.writer(file)
        writer.writerow(data_label)
    """ Main control loop. """
    pre_time = time.time()#initialize
    time.sleep(0.1)
    tim = time.time()
    datalist1 = []
    datalist2 = []
    positions = [[-0.1,0.1],[-0.1707,0.0707],[-0.1,-0.1],[-0.1414,0.0],[0,0.1414],[0.1,0.1]]
    for i in range(6):
        tim = time.time()
        datalist = []
        targetPosition = [positions[i][0], positions[i][1]]
        targetVelocities = [0.0, 0.0]
        print("go"  + str(targetPosition))
        Jte = [[-0.1*np.sin(targetPosition[0]), 0.1*np.cos(targetPosition[0])],[0.1*np.sin(targetPosition[1]), -0.1*np.cos(targetPosition[1])]]
        for i in range(20000):
            # Calculate loop duration
            dt = time.time() - pre_time
            pre_time = time.time()
            
            # Update motor control
            Motor.servo_mode = 0x08
            valg = np.dot([[1/motor1.torque_constant,0],[0,1/motor2.torque_constant]],gravity_compensation(motor1, motor2))
            val = workspace_impedance_control_2DOF(motor1, motor2, targetPosition, targetVelocities, dt)
            data1 = motor1.create_data_packet_current(val[0])
            data1_receive = motor1.transfer_spi_data()
            
            data2 = motor2.create_data_packet_current(val[1])
            data2_receive = motor2.transfer_spi_data()
            #databuffer = [pre_time,dt,motor1.pulse,motor1.rad,motor1.current,motor1.initialPosition,val1,motor2.pulse,motor2.rad,motor2.current,motor2.initialPosition,val2]
            datalist.append([pre_time,dt,targetPosition[0],targetPosition[1],val[2],val[3],motor1.current,motor2.current,val[0],val[1],valg[0],valg[1] ])
            """
            print(data1)
            print(data2)
            """
            #print(data1_receive )
            #print(f"\033[31m{motor1.pulse }\033[0m",end = " ")
            #print(motor1.initialPosition)
            #print(val1)
            
            #print(data2_receive )
            #print(f"\033[31m{motor2.pulse }\033[0m",end = " ")
            #print(motor2.initialPosition)
            #print(val2)
            
            #print("")
            #print(motor1.current )
            # Delay for a short while (example loop rate)
            #time.sleep(0.01)
        print(tim -pre_time)
        with open(fname,"a",newline = "") as file:
            writer = csv.writer(file)
            writer.writerows(datalist)
        
# main #######################################################################################################################################################
if __name__ == "__main__":
    #print(loop_impedance2())
    # Run initialization
    motor1.initialize_motor_c()
    motor2.initialize_motor_c()
    #initialize_motor(motor2)
    time.sleep(3)
    
    # Start the control loop
    try:
        Motor.servo_mode = 0x00#already global
        #loop_impedance2()
        loop_impedance_adoptive_slow()
        
        print(xfererror)
        loop_gravity_compensation()
        servo_mode = 0x00
        motor1.end_process_c()
        motor2.end_process_c()
        lines.release()
        #motors.release()
        chip.close()
        #print(fname)
        print("closed!")
    except KeyboardInterrupt:
        servo_mode = 0x00
        motor1.end_process_c()
        motor2.end_process_c()
        lines.release()
        #motors.release()
        chip.close()
        #print(fname)
        print("closed w keyboard!")
