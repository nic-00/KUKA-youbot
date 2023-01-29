"""coordinate_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, PositionSensor
from controller import Supervisor

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
    
def is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ):
    if targetX-gps_value[gpsX]<0.2 and targetX-gps_value[gpsX] >-0.2:
        return True
    else:
        return False
        
def is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ):
    if targetZ-gps_value[gpsZ]<0.2 and targetZ-gps_value[gpsZ] >-0.2:
        return True
    else:
        return False
    
# create the Robot instance.
robot = Robot()
arm1 = robot.getDevice("arm1")
arm2 = robot.getDevice("arm2")
arm3 = robot.getDevice("arm3")
arm4 = robot.getDevice("arm4")
arm5 = robot.getDevice("arm5")

finger1 = robot.getDevice("finger1")
finger2 = robot.getDevice("finger2")

finger1.setPosition(0.024);
finger2.setPosition(0.024);
arm1.setPosition(-2.93)
arm4.setPosition(1.76)

positionArm1 = PositionSensor("arm1sensor")
positionArm2 = PositionSensor("arm2sensor")
positionArm3 = PositionSensor("arm3sensor")
positionArm4 = PositionSensor("arm4sensor")
positionArm5 = PositionSensor("arm5sensor")

positionFinger1 = PositionSensor("finger1sensor")
positionFinger2 = PositionSensor("finger2sensor")

def ArmMovement():
    RightSide()
    PickWateringCan()
    # PourWater()

def RightSide():
    if (positionArm1.getValue() <= -2.91):
        arm2.setPosition(1.57)
        
        # print(positionArm2.getValue())

def PourWater():
    if (positionArm2.getValue() >= 0.7):
        arm5.setPosition(-1.46)
    
def PickWateringCan():
    # arm1.setPosition(1.57)
    # print(positionArm1.getValue());
    if (positionArm2.getValue() >= 1.55):
        arm1.setPosition(-1.65)
        if (positionArm1.getValue() <= -1.63):
            arm4.setPosition(0)
            
        if (positionArm4.getValue() <= 0.001):
            FingerMovement()
        # print(positionFinger1.getValue())
        if (positionFinger1.getValue() <= 0.01):
        # WateringPlant()
            arm2.setPosition(0.8)
            print(positionArm2.getValue())
        if (positionArm2.getValue() <= 0.9):
            arm4.setPosition(1.30)
            print("arm4")
            print(positionArm4.getValue())
            if (positionArm4.getValue() >= 1.2):
                arm5.setPosition(-1.46)

def FingerMovement():
    # finger1.setPosition(0.01)
    # finger2.setPosition(0.01)
    finger1.setPosition(0.0)
    finger2.setPosition(0.0)
    
def run_robot(robot):
    timestep=32
    max_speed=6.28
    targetX=1.0 #front and back +ve front, -ve back
    targetZ=1.0 #left and right +ve left, -ve right
    gpsX=1 #which one in gps array shows front and back
    gpsZ=0 #which one in gps array shows left and right
    turning_coordinate=0.7 # number of coordinate needeed for the rebot to make a turn
    
 # get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

    # Motor
    # right_front_motor=robot.getMotor('left wheel motor')
    # left_front_moteor=robot.getMotor('right wheel motor')
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
    # You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

    count=0
    initial_count=0
    first_turn=True
    while robot.step(timestep) != -1:
        count=count+1
        # Read the sensors:
        gps_value = gps.getValues()
        
        # print(gps_value)
        msg = "GPS values: "
        for each_val in gps_value:
            msg += " {0:0.5f}".format(each_val)
        print(msg)
        
        compass_value=compass.getValues()
        cmsg = "Compass values: "
        for each_val in compass_value:
            cmsg += " {0:0.5f}".format(each_val)
        print(cmsg)
        
        # print(compass_value[0])
        # print(compass_value[1])
        
        if compass_value[0]>0.9999: #facing forward, move x-axis
            # if(targetX-gps_value[gpsX]<1.4 and targetX-gps_value[gpsX]>-1.4 and first_turn): #start to turn to go to y-axis
                # print("first turn right")
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # first_turn=False
            if (targetX-gps_value[gpsX]>=turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ)): #move forward
                print("forward 0.7")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]>=0.2): #move forward
                print("forward 0.2")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]<=-turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ)): #move backward
                print("backward 0.7")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]<=-0.2): #move backward
                print("backward 0.2")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ) and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                ArmMovement()
            else: #if(targetX-gps_value[gpsX]<0.7 and targetX-gps_value[gpsX]>-0.7):
                print("turn right")
                turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
        
        elif compass_value[0]<-0.9999: #facing backward, move x-axis
            if (targetX-gps_value[gpsX]>=turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ)): #move forward
                print("back 0.7")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]>=0.2): #move forward
                print("back 0.2")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]<=-turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ)): #move backward
                print("front 0.7")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[gpsX]<=-0.2): #move backward
                print("front 0.2")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ) and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                ArmMovement()
            else: #if(targetX-gps_value[gpsX]<0.7 and targetX-gps_value[gpsX]>-0.7):
                print("turn left")
                turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
        
        elif compass_value[1]>=0.9999: #facing left, move Z-axis

            # if(targetX-gps_value[gpsX]<0.2 and targetX-gps_value[gpsX] >-0.2): #correct x coordinate, proceed
            if(targetZ-gps_value[gpsZ]>=0.2 and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("forward Y 0.2") # move forward until 0.2 if x coordinate is correct
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif(targetZ-gps_value[gpsZ]>=turning_coordinate):
                print("forward Y 0.7") #if wrong, turn when 0.7 before the coordinate
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetZ-gps_value[gpsZ]<=-0.2 and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)): #move backward
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("backward Y 0.2")
            elif (targetZ-gps_value[gpsZ]<=-turning_coordinate): #move backward
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("backward Y 0.7")
            elif (is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ) and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                ArmMovement()
            else: #wrong x coordinate, turn to x-axis
                print("turn left")
                turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                               
        elif compass_value[1]<=-0.9999: #facing left, move Z-axis

            # if(targetX-gps_value[gpsX]<0.2 and targetX-gps_value[gpsX] >-0.2): #correct x coordinate, proceed
            if(targetZ-gps_value[gpsZ]>=0.2 and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("back Y 0.2") # move forward until 0.2 if x coordinate is correct
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif(targetZ-gps_value[gpsZ]>=turning_coordinate):
                print("back Y 0.7") #if wrong, turn when 0.7 before the coordinate
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetZ-gps_value[gpsZ]<=-0.2 and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)): #move backward
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("front Y 0.2")
            elif (targetZ-gps_value[gpsZ]<=-turning_coordinate): #move backward
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("front Y 0.7")
            elif (is_Zcoordinate_correct(targetZ, gps_value, gpsX, gpsZ) and is_Xcoordinate_correct(targetX, gps_value, gpsX, gpsZ)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                ArmMovement()
            else: #wrong x coordinate, turn to x-axis
                print("turn right")
                turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# if__name__ == "__main__":
# my_robot = Robot()
# my_compass=Compass()
run_robot(robot)
# Enter here exit cleanup code.
