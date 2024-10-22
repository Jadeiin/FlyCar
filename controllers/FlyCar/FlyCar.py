from LPF import LowPassFilter
from PID import PIDController
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
start_time = robot.getTime()

# 初始化左右轮
leftMotor = robot.getDevice('left hip')
leftMotorSensor = leftMotor.getPositionSensor()
leftMotorSensor.enable(timestep)
rightMotor = robot.getDevice('right hip')
rightMotorSensor = rightMotor.getPositionSensor()
rightMotorSensor.enable(timestep)

# 初始化IMU
IMU = robot.getDevice("IMU")
IMU.enable(timestep)

# 初始化陀螺仪
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# PID Controllers
pid_angle = PIDController(1, 0, 0, 100000, 8, start_time)
pid_gyro = PIDController(0.06, 0, 0, 100000, 8, start_time)
pid_distance = PIDController(0.5, 0, 0, 100000, 8, start_time)
pid_speed = PIDController(0.7, 0, 0, 100000, 8, start_time)
pid_yaw_angle = PIDController(1.0, 0, 0, 100000, 8, start_time)
pid_yaw_gyro = PIDController(0.04, 0, 0, 100000, 8, start_time)
pid_lqr_u = PIDController(1, 15, 0, 100000, 8, start_time)
pid_zeropoint = PIDController(0.002, 0, 0, 100000, 4, start_time)
pid_roll_angle = PIDController(8, 0, 0, 100000, 450, start_time)

# Low pass filters
lpf_joyy = LowPassFilter(0.2, start_time)
lpf_zeropoint = LowPassFilter(0.1, start_time)
lpf_roll = LowPassFilter(0.3, start_time)

# LQR parameters
angle_zeropoint = -2.25
distance_zeropoint = -256.0
LQR_u = 0

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

    # 轮子的角速度
    left_shaft_speed = leftMotor.getVelocity()
    right_shaft_speed = rightMotor.getVelocity()

    # IMU角度
    rpy = IMU.getRollPitchYaw()
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    # 陀螺仪角速度
    gyro_values = gyro.getValues()
    roll_speed = gyro_values[0]
    pitch_speed = gyro_values[1]
    yaw_speed = gyro_values[2]

    # Process sensor data here.
    # LQR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro
    #         + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;
    LQR_distance = 0.5 * (left_shaft_angle + right_shaft_angle)
    LQR_speed = 0.5 * (left_shaft_speed + right_shaft_speed)
    LQR_angle = roll
    LQR_gyro = roll_speed

    angle_control = pid_angle(LQR_angle - angle_zeropoint, current_time)
    gyro_control = pid_gyro(LQR_gyro, current_time)
    distance_control = pid_distance(LQR_distance - distance_zeropoint, current_time)
    speed_control = pid_speed(LQR_speed, current_time)

    LQR_u = angle_control + gyro_control + distance_control + speed_control
    LQR_u = pid_lqr_u(LQR_u, current_time)
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control, current_time), current_time)

    # 执行控制
    YAW_output = 0
    leftMotor.setTorque((-0.5) * (LQR_u + YAW_output))
    rightMotor.setTorque((-0.5) * (LQR_u - YAW_output))
# Enter here exit cleanup code.
