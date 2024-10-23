import pickle

from LPF import LowPassFilter
from PID import PIDController
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
start_time = robot.getTime()

# 初始化左右轮
leftMotor = robot.getDevice('left wheel')
leftMotorSensor = leftMotor.getPositionSensor()
leftMotor.setVelocity(0.0)
leftMotorSensor.enable(timestep)
rightMotor = robot.getDevice('right wheel')
rightMotor.setVelocity(0.0)
rightMotorSensor = rightMotor.getPositionSensor()
rightMotorSensor.enable(timestep)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# 初始化IMU
IMU = robot.getDevice("IMU")
IMU.enable(timestep)

# 初始化陀螺仪
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# PID Controllers
pid_angle = PIDController(2, 0, 0, 1e6, 8, start_time)
pid_gyro = PIDController(0.06, 0, 0, 1e6, 8, start_time)
pid_distance = PIDController(0.2, 0, 0, 1e6, 2, start_time)
pid_speed = PIDController(0.3, 0, 0, 1e6, 8, start_time)
# pid_yaw_angle = PIDController(1.0, 0, 0, 100000, 8, start_time)
# pid_yaw_gyro = PIDController(0.04, 0, 0, 100000, 8, start_time)
# pid_lqr_u = PIDController(1, 15, 0, 100000, 8, start_time)
pid_zeropoint = PIDController(0.002, 0, 0, 1e6, 4, start_time)
# pid_roll_angle = PIDController(8, 0, 0, 100000, 450, start_time)

# Low pass filters
lpf_joyy = LowPassFilter(0.2, start_time)
lpf_zeropoint = LowPassFilter(0.1, start_time)
lpf_roll = LowPassFilter(0.3, start_time)
lpf_speed = LowPassFilter(0.1, start_time)
lpf_gyro = LowPassFilter(0.1, start_time)

# LQR parameters
angle_zeropoint = 0
distance_zeropoint = -256.0
LQR_u = 0

#小车的物理参数
m_car = 3.97
m_wheel = 0.507
r_wheel = 0.085
I_Wheel = 0.5*m_wheel*r_wheel**2

# 用于存储变量的数组
time_data = []
LQR_distance_data = []
LQR_speed_data = []
LQR_angle_data = []
LQR_gyro_data = []
angle_control_data = []
gyro_control_data = []
distance_control_data = []
speed_control_data = []
LQR_u_data = []
angle_zeropoint_data = []

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    current_time = robot.getTime()

    # 轮子的角度
    left_shaft_angle = leftMotorSensor.getValue()
    right_shaft_angle = rightMotorSensor.getValue()
    # print(f"left_shaft_angle:{left_shaft_angle}, right_shaft_angle:{right_shaft_angle}")

    # 轮子的角速度
    left_shaft_speed = leftMotor.getVelocity()
    right_shaft_speed = rightMotor.getVelocity()
    # print(f"left_shaft_speed:{left_shaft_speed}, right_shaft_speed:{right_shaft_speed}")

    # IMU角度
    rpy = IMU.getRollPitchYaw()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    # print(f"pitch:{pitch}")

    # 陀螺仪角速度
    gyro_values = gyro.getValues()
    roll_speed = gyro_values[0]
    pitch_speed = gyro_values[1]
    yaw_speed = gyro_values[2]
    # print(f"pitch_speed:{pitch_speed}")

    # Process sensor data here.
    # LQR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro
    #         + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;
    LQR_distance = 0.5 * (left_shaft_angle + right_shaft_angle)
    print(f"LQR_distance:{LQR_distance}")
    LQR_speed = 0.5 * (left_shaft_speed + right_shaft_speed)
    LQR_speed = lpf_speed(LQR_speed, current_time)
    print(f"LQR_speed:{LQR_speed}")
    LQR_angle = pitch
    print(f"LQR_angle:{LQR_angle}")
    LQR_gyro = pitch_speed
    LQR_gyro = lpf_gyro(LQR_gyro, current_time)
    print(f"LQR_gyro:{LQR_gyro}")

    angle_control = pid_angle(LQR_angle - angle_zeropoint, current_time)
    print(f"angle_control:{angle_control}")
    gyro_control = pid_gyro(LQR_gyro, current_time)
    print(f"gyro_control:{gyro_control}")

    if abs(LQR_speed) < 0.5:
        distance_zeropoint = LQR_distance

    distance_control = pid_distance(LQR_distance - distance_zeropoint, current_time)
    print(f"distance_control:{distance_control}")
    speed_control = pid_speed(LQR_speed, current_time)
    print(f"speed_control:{speed_control}")

    LQR_u = - angle_control - gyro_control - distance_control - speed_control
    # LQR_u = pid_lqr_u(LQR_u, current_time)
    print(f"LQR_u:{LQR_u}")
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control, current_time), current_time)
    print(f"angle_zeropoint:{angle_zeropoint}")

    # 执行控制
    YAW_output = 0

    # 正是往右
    leftMotor.setVelocity(0.5 * (LQR_u + YAW_output) / I_Wheel)
    rightMotor.setVelocity(0.5 * (LQR_u - YAW_output) / I_Wheel)
    print(f"set speed:{0.5 * LQR_u / I_Wheel}")

    # 将每次迭代的数据存入数组
    time_data.append(current_time)
    LQR_distance_data.append(LQR_distance)
    LQR_speed_data.append(LQR_speed)
    LQR_angle_data.append(LQR_angle)
    LQR_gyro_data.append(LQR_gyro)
    angle_control_data.append(angle_control)
    gyro_control_data.append(gyro_control)
    distance_control_data.append(distance_control)
    speed_control_data.append(speed_control)
    LQR_u_data.append(LQR_u)
    angle_zeropoint_data.append(angle_zeropoint)

    with open('data.pkl', 'wb') as f:
        pickle.dump([time_data, LQR_distance_data, LQR_speed_data, LQR_angle_data, LQR_gyro_data, angle_control_data, gyro_control_data, distance_control_data, speed_control_data, LQR_u_data, angle_zeropoint_data], f)


    print("=====================================")