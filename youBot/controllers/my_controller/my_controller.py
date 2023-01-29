"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor

def move_forward(rf,lf,rb,lb):
    print("set forward")
    rf.setVelocity(6.28)
    lf.setVelocity(6.28)
    rb.setVelocity(6.28)
    lb.setVelocity(6.28)

def move_backward(rf,lf,rb,lb):
    print("set back")
    rf.setVelocity(-6.28)
    lf.setVelocity(-6.28)
    rb.setVelocity(-6.28)
    lb.setVelocity(-6.28)
    
def turn_right(rf,lf,rb,lb):
    print("set right")
    rf.setVelocity(-6.28)
    lf.setVelocity(6.28)
    rb.setVelocity(-6.28)
    lb.setVelocity(6.28)

def turn_left(rf,lf,rb,lb):
    print("set left")
    rf.setVelocity(6.28)
    lf.setVelocity(-6.28)
    rb.setVelocity(6.28)
    lb.setVelocity(-6.28)
    
def stop(rf,lf,rb,lb):
    print("set stop")
    rf.setVelocity(0)
    lf.setVelocity(0)
    rb.setVelocity(0)
    lb.setVelocity(0)


def is_Xcoordinate_correct(targetX, gps_value):
    if targetX-gps_value[0]<0.2 and targetX-gps_value[0] >-0.2:
        return True
    else:
        return False
        
def is_Zcoordinate_correct(targetZ, gps_value):
    if targetZ-gps_value[2]<0.2 and targetZ-gps_value[2] >-0.2:
        return True
    else:
        return False


# create the Robot instance.
# TIME_STEP = 32
# robot = Robot()
def run_robot(robot):
    timestep=32
    max_speed=6.28
    targetX=1.0
    targetZ=-1.0
    turning_coordinate=0.7 # number of coordinate needeed for the rebot to make a turn

    right_front_motor=robot.getDevice('wheel1')
    left_front_motor=robot.getDevice('wheel2')
    right_back_motor=robot.getDevice('wheel3')
    left_back_motor=robot.getDevice('wheel4')
        
    right_front_motor.setPosition(float('inf'))
    left_front_motor.setPosition(float('inf'))
    right_back_motor.setPosition(float('inf'))
    left_back_motor.setPosition(float('inf'))
        
    right_front_motor.setVelocity(0.0)
    left_front_motor.setVelocity(0.0)
    right_back_motor.setVelocity(0.0)
    left_back_motor.setVelocity(0.0)
    
    gps=robot.getDevice('gps')
    gps.enable(timestep)
    
    compass=robot.getDevice('compass')
    compass.enable(timestep)

    frontSensor1 = robot.getDevice("frontDS1")
    frontSensor2 = robot.getDevice("frontDS2")
    frontSensor3 = robot.getDevice("frontDS3")
    
    backSensor1 = robot.getDevice("backDS1")
    backSensor2 = robot.getDevice("backDS2")
    backSensor3 = robot.getDevice("backDS3")
    
    leftSensor1 = robot.getDevice("leftDS1")
    leftSensor2 = robot.getDevice("leftDS2")
    leftSensor3 = robot.getDevice("leftDS3")
    
    rightSensor1 = robot.getDevice("rightDS1")
    rightSensor2 = robot.getDevice("rightDS2")
    rightSensor3 = robot.getDevice("rightDS3")

    frontSensor1.enable(timestep)
    frontSensor2.enable(timestep)
    frontSensor3.enable(timestep)
    
    backSensor1.enable(timestep)
    backSensor2.enable(timestep)
    backSensor3.enable(timestep)
    
    leftSensor1.enable(timestep)
    leftSensor2.enable(timestep)
    leftSensor3.enable(timestep)
    
    rightSensor1.enable(timestep)
    rightSensor2.enable(timestep)
    rightSensor3.enable(timestep)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller    
    count=0
    initial_count=0
    first_turn=True
    
    


while robot.step(timestep) != -1:
    fsValue1 = frontSensor1.getValue()
    fsValue2 = frontSensor2.getValue()
    fsValue3 = frontSensor3.getValue()
    
    bsValue1 = backSensor1.getValue()
    bsValue2 = backSensor2.getValue()
    bsValue3 = backSensor3.getValue()
    
    lsValue1 = leftSensor1.getValue()
    lsValue2 = leftSensor2.getValue()
    lsValue3 = leftSensor3.getValue()
    
    rsValue1 = rightSensor1.getValue()
    rsValue2 = rightSensor2.getValue()
    rsValue3 = rightSensor3.getValue()
    
    print("Sensor fsvalue 1 is: ", fsValue1)
    print("Sensor fsvalue 2 is: ", fsValue2)
    print("Sensor fsvalue 3 is: ", fsValue3)
    
    print("Sensor bsvalue 1 is: ", bsValue1)
    print("Sensor bsvalue 2 is: ", bsValue2)
    print("Sensor bsvalue 3 is: ", bsValue3)
    
    print("Sensor lsvalue 1 is: ", lsValue1)
    print("Sensor lsvalue 2 is: ", lsValue2)
    print("Sensor lsvalue 3 is: ", lsValue3)
    
    print("Sensor rsvalue 1 is: ", rsValue1)
    print("Sensor rsvalue 2 is: ", rsValue2)
    print("Sensor rsvalue 3 is: ", rsValue3)
    
    
    if( (fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) or (lsValue1 < 950) or (lsValue2 < 950) or (lsValue3 < 950) or (rsValue1 < 950) or (rsValue2 < 950) or (rsValue3 < 950)):
        right_front_motor.setVelocity(-2.0)
        left_front_motor.setVelocity(-2.0)
        right_back_motor.setVelocity(-2.0)
        left_back_motor.setVelocity(-2.0)
        # break
    
    if((bsValue1 < 950) or (bsValue2 < 950) or (bsValue3 < 950)):
        right_front_motor.setVelocity(2.0)
        left_front_motor.setVelocity(2.0)
        right_back_motor.setVelocity(2.0)
        left_back_motor.setVelocity(2.0)
        # break
        



# distanceSensor = DistanceSensor("youBot")

# wheel1 = robot.getDevice("wheel1")
# sensor = robot.getDistanceSensor("distance sensor")
# distanceSensor.enable("distance sensor")
# type = distanceSensor.getType(robot)
# positionWheel1 = position.getValue("wheel1sensor")
# print(type)

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
 # motor = robot.getDevice('youBot')
 # print(motor)
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# Enter here exit cleanup code.
