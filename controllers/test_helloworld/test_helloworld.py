"""test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
leftMotor = robot.getDevice('left wheel')
rightMotor  = robot.getDevice('right wheel')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
leftSensor = robot.getDevice('left wheel_sensor')
leftSensor.enable(timestep)
rightSensor = robot.getDevice('right wheel_sensor')

Inertial = robot.getInertialUnit("imu")
Inertial.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    val = leftSensor.getValue()
    print(val)
    
    # Read and print RPY data
    rpy = Inertial.getRollPitchYaw()
    print("Roll: {}, Pitch: {}, Yaw: {}".format(rpy[0], rpy[1], rpy[2]))
    
    # Process sensor data here.
    leftMotor.setVelocity(10.0)
    rightMotor.setVelocity(10.0)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.