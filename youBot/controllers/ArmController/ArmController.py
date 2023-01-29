"""ArmController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, PositionSensor, Keyboard, DistanceSensor, Supervisor

# create the Robot instance.
robot = Robot()
keyboard = Keyboard()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

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
    # if (positionArm5.getValue() <= -1.30):
    arm5.setPosition(-1.46)
    print("frgdge")
    
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
                    print("arm2")
                    if (positionArm2.getValue() <= 1.56):
                        arm4.setPosition(1.30)
                        print("arm4")
                        print(positionArm4.getValue())
                        print("arm5")
                        if (positionArm4.getValue() >= 1.2):
                            arm5.setPosition(-1.46)
                            print("arm5")

def FingerMovement():
    # finger1.setPosition(0.01)
    # finger2.setPosition(0.01)
    finger1.setPosition(0.0)
    finger2.setPosition(0.0)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
    key = keyboard.getKey()
    # Process sensor data here.
    ArmMovement()
    if(key==Keyboard.CONTROL+ord('X')):
        PourWater()
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
