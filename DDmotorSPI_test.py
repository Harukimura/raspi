import gpiod
import spidev
import time
import math
import struct

# Define GPIO chip and lines
CHIP = "gpiochip0"
#CHIP = "gpiochip4"#addict 4 env
LINE_SVON = 2
LINE_ALARM = 1
LINE_RESET = 0

LINE_SS1 = 25
LINE_SS2 = 3

NUM_OF_MOTOR = 2
# Open GPIO lines
chip = gpiod.Chip(CHIP)
lines = chip.get_lines([LINE_SVON, LINE_ALARM, LINE_RESET])
motors = chip.get_lines([ LINE_SS1, LINE_SS2])

# Set lines for output (except for ALARM, S1, and S2, which are inputs)
lines.request(consumer="sv_control", type=gpiod.LINE_REQ_DIR_OUT)
motors.request(consumer="motor_control", type=gpiod.LINE_REQ_DIR_OUT)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (SS1)
spi.mode = 3
#spi.no_cs = True
spi.max_speed_hz = 50000

# Motor class to hold parameters
class Motor:
    def __init__(self, pin,SPIid, setting_pulse, Kp, Kd):
        self.pinNumber = pin
        self.SPIid = 0
        self.setting_Pulse = setting_pulse
        self.Kp = Kp
        self.Kd = Kd
        self.rad = 0
        self.initialPosition = 0
        self.error = 0
        self.error_old = 0
        self.cd = 0
        self.cd_filtered = 0
        self.alpha = 0.1  # For filtering
        self.current = 0
        self.pulse = 0
        self.data_packet = [0x00 for i in range(12)]

# Initialize motors
motor1 = Motor(pin=LINE_SS1, SPIid = 0, setting_pulse=1048576, Kp=1.5, Kd=0.2)
motor2 = Motor(pin=LINE_SS2, SPIid = 1, setting_pulse=2097152, Kp=1.5, Kd=0.2)

def initialize_motor(motor):
    """ Initialize motor using SPI communication. """
    for _ in range(5):
        transfer_spi_data(motor, create_data_packet(motor,0.0))  # Send zero value
        

def create_data_packet(motor, value):
    """ Prepare a data packet for SPI transfer. """
    value = 0.0
    SPI_RESET = 0
    DIN4  = 0
    DIN3 = 0
    DIN2 = 0
    DIN1 = 0
    # コマンドバイトを生成
    #command_byte = 0x08  # t =8 f = 0
    command_byte = 0x00  # t =8 f = 0

    # 送信する値を4バイトのデータとして変換
    # valueはモーターの制御対象の値（例: 電流、位置）。ここではfloatを整数に変換し、バイト列にする
    current_data_bytes = struct.pack('<f',value)

    # 予約バイト
    unused_data12 = [0x00, 0x00]
    unused_data89 = [0x00, 0x00]
    mode_bytes = SPI_RESET <<4 + DIN4 <<3  + DIN3 <<2 + DIN2 <<1 + DIN1

    # バイト列を生成
    data_packet = [command_byte] + unused_data12 + list(current_data_bytes) + [mode_bytes] + unused_data89

    # チェックサムの計算
    sum_value = 0
    for i in range(4):
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


    # 結果を表示
    # binary_representation = ' '.join(f'{byte:08b}' for byte in data_packet)
    # print(binary_representation)


    return data_packet  # Placeholder for a 12-byte data packet

def transfer_spi_data(motor,data):
    """ xfer motor data over SPI. """
    motorCSlist = [1 for i in range(NUM_OF_MOTOR)]
    motorCSlist[motor.SPIid] = 0
    motors.set_values(motorCSlist)
    time.sleep(0.020)
    response = spi.xfer2(data)  # xfer 12 bytes
    time.sleep(0.020)
    motors.set_values([1 for i in range(NUM_OF_MOTOR)])
    # Extract encoder pulses and current values from the response
    motor.pulse = struct.unpack('<l',bytes(response[1:5]))[0]
    motor.current = struct.unpack('<f', bytes(response[5:9]))[0]
    motor.rad = (2 * math.pi * motor.pulse) / motor.setting_Pulse
    return response

def PDcurrent_control_motor(motor, target_angle, dt):
    """ Perform PD control for the motor. """
    motor.error_old = motor.error
    motor.error = target_angle - motor.rad
    motor.cd = motor.error * motor.Kp + (motor.error - motor.error_old) / dt * motor.Kd
    motor.cd_filtered = motor.cd * (1 - motor.alpha) + motor.cd_filtered * motor.alpha
    motor.cd_filtered = min(max(motor.cd_filtered, -1.0), 1.0)  # Limit to [-1, 1]

def loop():
    """ Main control loop. """
    pre_time = time.time()
    
    while True:
        # Calculate loop duration
        dt = time.time() - pre_time
        pre_time = time.time()
        
        # Update motor control
        #PDcurrent_control_motor(motor1, target_angle=math.pi / 4, dt=dt)
        #PDcurrent_control_motor(motor2, target_angle=math.pi / 2, dt=dt)
        
        # Send SPI data to motors
        data1 = create_data_packet(motor1,motor1.cd_filtered)
        #data2 = create_data_packet(motor2,motor2.cd_filtered)
        print(data1)
        data1_receive = transfer_spi_data(motor1, data1)
        #data2_receive = transfer_spi_data(motor2, data2)
        print(data1_receive )
        print(motor1.pulse )
        #print(motor1.current )
        # Delay for a short while (example loop rate)
        time.sleep(0.1)

# main
# Run initialization
initialize_motor(motor1)
initialize_motor(motor2)
time.sleep(3)
# Start the control loop
try:
    loop()
except KeyboardInterrupt:
    lines.release()
    motors.release()
    chip.close()
    spi.close()
