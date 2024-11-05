import pickle

from LPF import LowPassFilter
from PID import PIDController
from controller import Robot

global current_time


def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)


# 用于存储变量的数组
time_data = []
pitch_data = []
velocity_data = []
balance_torque_data = []
turn_torque_data = []
walk_torque_data = []


def collect_data():
    # 将每次迭代的数据存入数组
    time_data.append(current_time)
    pitch_data.append(pitch)
    velocity_data.append(speed)
    balance_torque_data.append(balance_torque)
    turn_torque_data.append(turn_torque)
    walk_torque_data.append(walk_torque)

    with open('data.pkl', 'wb') as f:
        pickle.dump([time_data, pitch_data, velocity_data, balance_torque_data, turn_torque_data, walk_torque_data], f)


# create the Robot instance.
robot = Robot()
print(robot.devices)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
start_time = robot.getTime()

# create keyboard instance
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# 初始化左右轮
leftMotor = robot.getDevice('left_motor')
leftMotor.setVelocity(0.0)
leftSensor = leftMotor.getPositionSensor()
leftSensor.enable(timestep)
rightMotor = robot.getDevice('right_motor')
rightMotor.setVelocity(0.0)
rightSensor = rightMotor.getPositionSensor()
rightSensor.enable(timestep)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.enableTorqueFeedback(True)
rightMotor.enableTorqueFeedback(True)

# 初始化IMU
IMU = robot.getDevice("IMU")
IMU.enable(timestep)

# 初始化陀螺仪
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# 初始化GPS
gps = robot.getDevice("gps")
gps.enable(timestep)

# PID Controllers
angle_loop = PIDController(30, 0, 0, 1e6, 10, start_time)
gyro_loop = PIDController(5, 0, 0, 1e6, 10, start_time)
loop_angle_yaw = PIDController(5, 0, 0, 1e6, 10, start_time)
loop_gyro_yaw = PIDController(1, 0, 0, 1e6, 10, start_time)
loop_speed = PIDController(1, 1e-3, 0, 1e6, 10, start_time)

# Low Pass Filter
gps_speed_filter = LowPassFilter(0.1, start_time)
pitch_filter = LowPassFilter(0.1, start_time)
speed_target_filter = LowPassFilter(0.1, start_time)

# 小车的物理参数
m_car = 3.97
m_wheel = 0.507
r_wheel = 0.085
I_Wheel = 0.5 * m_wheel * r_wheel ** 2

current_time = 0
left_position = leftSensor.getValue()
right_position = rightSensor.getValue()

# Main loop:
while robot.step(timestep) != -1:
    # Read the sensors:
    time_prev = current_time
    current_time = robot.getTime()
    print(f"current_time:{current_time}")

    # IMU角度
    rpy = IMU.getRollPitchYaw()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    print(f"pitch:{pitch}")

    # 陀螺仪角速度
    gyro_values = gyro.getValues()
    pitch_speed = gyro_values[1]
    yaw_speed = gyro_values[2]
    # print(f"pitch_speed:{pitch_speed}")

    # 通过PositionSensor获取速度
    left_position_prev = left_position
    right_position_prev = right_position
    left_position = leftSensor.getValue()
    right_position = rightSensor.getValue()
    speed = (left_position - left_position_prev + right_position - right_position_prev) / 2 / (
                current_time - time_prev)
    print(f"speed:{speed}")

    # Process sensor data here.

    # 计算控制量
    userkey = keyboard.getKey()
    if userkey == 65:

        speed_target = speed_target_filter(-5, current_time)
    elif userkey == 68:
        speed_target = speed_target_filter(5, current_time)
    else:
        speed_target = speed_target_filter(0, current_time)
    print(f"speed_target:{speed_target}")

    angle_control = angle_loop(pitch, current_time)
    gyro_control = gyro_loop(pitch_speed, current_time)
    angle_yaw_control = loop_angle_yaw(yaw, current_time)
    gyro_yaw_control = loop_gyro_yaw(yaw_speed, current_time)
    speed_control = loop_speed(speed_target - speed, current_time)

    balance_torque = -angle_control - gyro_control
    # balance_torque = 0
    turn_torque = angle_yaw_control + gyro_yaw_control
    turn_torque = 0
    walk_torque = speed_control
    print(f"balance_torque:{balance_torque}, turn_torque:{turn_torque}, walk_torque:{walk_torque}")

    torque_left = - balance_torque + turn_torque - walk_torque
    torque_right = - balance_torque - turn_torque - walk_torque

    # 执行控制
    # 控制量为正，车往左
    # 左上扬，pitch>0
    # 车往右，gps_speed>0
    leftMotor.setTorque(torque_left)
    # leftMotor.setTorque(3)
    rightMotor.setTorque(torque_right)
    # rightMotor.setTorque(3)
    print(f"set Torque:{torque_left}, {torque_right}")

    collect_data()

    print("=====================================")
