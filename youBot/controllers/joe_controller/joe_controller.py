"""coordinate_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
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
    if targetX-gps_value[1]<0.2 and targetX-gps_value[1] >-0.2:
        return True
    else:
        return False
        
def is_Zcoordinate_correct(targetZ, gps_value):
    if targetZ-gps_value[0]<0.2 and targetZ-gps_value[0] >-0.2:
        return True
    else:
        return False
    
# create the Robot instance.
# robot = Robot()
def run_robot(robot):
    timestep=32
    max_speed=6.28
    targetX=-1.0 #Y coordiate
    targetZ=1.0 #x-coordinate
    turning_coordinate=0.7 # number of coordinate needeed for the rebot to make a turn
    
 # get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())

    # Motor
    # right_front_motor=robot.getMotor('left wheel motor')
    # left_front_motor=robot.getMotor('right wheel motor')
    
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
    
    # leftSensor1 = robot.getDevice("leftDS1")
    # leftSensor2 = robot.getDevice("leftDS2")
    # leftSensor3 = robot.getDevice("leftDS3")
    
    # rightSensor1 = robot.getDevice("rightDS1")
    # rightSensor2 = robot.getDevice("rightDS2")
    # rightSensor3 = robot.getDevice("rightDS3")

    frontSensor1.enable(timestep)
    frontSensor2.enable(timestep)
    frontSensor3.enable(timestep)
    
    backSensor1.enable(timestep)
    backSensor2.enable(timestep)
    backSensor3.enable(timestep)
    
    # leftSensor1.enable(timestep)
    # leftSensor2.enable(timestep)
    # leftSensor3.enable(timestep)
    
    # rightSensor1.enable(timestep)
    # rightSensor2.enable(timestep)
    # rightSensor3.enable(timestep)
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
    
    turning_Front_ObsFront= False  #turning when face front and obstacle in front
    turning_Front_ObsBack = False  #turning when face front and obstacle at the back
    
    turning_Back_ObsFront = False  #turning when face back and obstacle in front 
    turning_Back_ObsBack = False   #turning when face back and obstacle iat the back
    
    turning_Right_ObsFront= False  #turning when face front and obstacle in front
    turning_Right_ObsBack = False  #turning when face front and obstacle at the back
    
    turning_Left_ObsFront= False  #turning when face front and obstacle in front
    turning_Left_ObsBack = False  #turning when face front and obstacle at the back
    
    moveBackCount =0  #got obstacle; face front ;  move back
    moveFrontCount = 0   #got obstacle; face front ; move front
    
    moveBackCount_FaceBack =0  #got obstacle; face back ; move back
    moveFrontCount_FaceBack =0 #got obstacle; face back ; move front
    
    while robot.step(timestep) != -1:
        count=count+1
        # Read the sensors:
        gps_value = gps.getValues()
        
        #Read Distance sensor:
        fsValue1 = frontSensor1.getValue()
        fsValue2 = frontSensor2.getValue()
        fsValue3 = frontSensor3.getValue()
        bsValue1 = backSensor1.getValue()
        bsValue2 = backSensor2.getValue()
        bsValue3 = backSensor3.getValue()
        
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
        
        # print("Sensor fsvalue 1 is: ", fsValue1)
        # print("Sensor fsvalue 2 is: ", fsValue2)
        # print("Sensor fsvalue 3 is: ", fsValue3)
        
        # print("Sensor bsvalue 1 is: ", bsValue1)
        # print("Sensor bsvalue 2 is: ", bsValue2)
        # print("Sensor bsvalue 3 is: ", bsValue3)
        
        if(compass_value[1]>0.9999 and turning_Front_ObsFront):
            turning_Front_ObsFront = False
            moveBackCount = 0
            moveBackCount_FaceBack = 0
        
        if(compass_value[1]>0.9999 and turning_Front_ObsBack):
            turning_Front_ObsBack = False
            moveFrontCount = 0
            moveFrontCount_FaceBack = 0
        
        if(compass_value[1]>0.9999 and turning_Back_ObsFront):
            turning_Front_ObsFront = False
            moveBackCount = 0
            moveBackCount_FaceBack = 0
        
        if(compass_value[0]>0.9999 and turning_Back_ObsBack):
            turning_Front_ObsBack = False
            moveFrontCount = 0
            moveFrontCount_FaceBack = 0
            
        if(compass_value[1]>0.9999 and turning_Right_ObsFront):
            turning_Front_ObsFront = False
            moveBackCount = 0
            moveBackCount_FaceBack = 0
        
        if(compass_value[0]>0.9999 and turning_Right_ObsBack):
            turning_Front_ObsBack = False
            moveFrontCount = 0
            moveFrontCount_FaceBack = 0
            
        if(compass_value[0]>0.9999 and turning_Left_ObsFront):
            turning_Left_ObsFront = False
            moveBackCount = 0
            moveBackCount_FaceBack = 0
            print("turning_Left_ObsFront")
         
        if(compass_value[0]>0.9999 and turning_Left_ObsBack):
            turning_Left_ObsBack = False
            moveBackCount = 0
            moveBackCount_FaceBack = 0
            print("turning_Left_ObsBack")
        # print("turning : ", not turning)
        
        # if( (fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) and (compass_value[0]>0.999)):
            # move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            # return
          
        # if compass_value[0]>0.9999: #facing forward, move x-axis
        if (compass_value[0]>0.9999 ): 
            print("success")
            if(turning_Front_ObsFront):
                print("moveBackCount", moveBackCount)
                moveBackCount = moveBackCount + 1
                if(moveBackCount <= 20 ):
                    print("BACK")
                    move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount <= 30):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount > 30):
                    print("RIGHT")
                    turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                #got obstacle at the back
            elif(turning_Front_ObsBack):
                print("moveFrontCount", moveFrontCount)
                moveFrontCount = moveFrontCount + 1
                if(moveFrontCount <= 20 ):
                    print("FRONT")
                    move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount <= 30 ):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount > 30):
                    print("RIGHT")
                    turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)  
                print("halo")
             
            elif((fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) and not turning_Front_ObsFront and not turning_Front_ObsBack):  #face front, move forward, got obstacle in front
                print("donnnn")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Front_ObsFront = True
                moveBackCount = moveBackCount + 1
                print("turning ", turning_Front_ObsFront)
            elif((bsValue1 < 950) or (bsValue2 < 950) or (bsValue3 < 950) and not turning_Front_ObsFront and not turning_Front_ObsBack): #face front, move backward, got obstacle at the back
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Front_ObsBack = True
                moveFrontCount = moveFrontCount + 1
                print("halooo")
            elif (targetX-gps_value[1]>=turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value) and not turning_Front_ObsFront): #move forward
                print("forward 0.7")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]>=0.2 and not turning_Front_ObsFront): #move forward
                print("forward 0.2")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]<=-turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value) and not turning_Front_ObsFront): #move backward
                print("backward 0.7")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]<=-0.2 and not turning_Front_ObsFront): #move backward
                print("backward 0.2")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (is_Zcoordinate_correct(targetZ, gps_value) and is_Xcoordinate_correct(targetX, gps_value)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            else:   #if(targetX-gps_value[0]<0.7 and targetX-gps_value[0]>-0.7 and not turning):
                print("turn right")
                turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
        
        elif compass_value[0]<-0.9999: #facing backward, move x-axis  
            print("moving?")
            if(turning_Back_ObsFront): 
                print("moveBackCount", moveBackCount)
                moveBackCount = moveBackCount + 1       
                if(moveBackCount <= 10 ):
                    print("BACK")
                    move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount <= 20 ):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount > 30):
                    print("LEFT")
                    turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)  ##check see
                #got obstacle at the back
                
            elif(turning_Back_ObsBack):
                print("run dao here???")
                moveFrontCount_FaceBack = moveFrontCount_FaceBack + 1
                print("moveFrontCount_FaceBack ", moveFrontCount_FaceBack )
                moveFrontCount_FaceBack  = moveFrontCount_FaceBack + 1
                if(moveFrontCount_FaceBack <= 80 ):
                    print("FRONT")
                    move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount_FaceBack <= 90 ):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount_FaceBack > 100 and moveFrontCount_FaceBack < 120):
                    print("LEFT")
                    turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)  
                print("halo")
             
            elif((fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) and not turning_Back_ObsFront and not turning_Back_ObsBack):  #face front, move forward, got obstacle in front
                print("Hitted")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Back_ObsFront = True
                moveBackCount = moveBackCount + 1
                print("turning ", turning_Back_ObsFront, turning_Back_ObsFront)
            elif((bsValue1 < 950) or (bsValue2 < 950) or (bsValue3 < 950) and not turning_Back_ObsFront and not turning_Back_ObsBack): #face front, move backward, got obstacle at the back
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Back_ObsBack = True
                moveFrontCount_FaceBack = moveFrontCount_FaceBack + 1
                print("HITTT")
            elif (targetX-gps_value[1]>=turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value)): #move forward
                print("back 0.7")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]>=0.2): #move forward
                print("back 0.2")
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]<=-turning_coordinate and is_Zcoordinate_correct(targetZ, gps_value)): #move backward
                print("front 0.7")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetX-gps_value[1]<=-0.2): #move backward
                print("front 0.2")
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (is_Zcoordinate_correct(targetZ, gps_value) and is_Xcoordinate_correct(targetX, gps_value)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            else: #if(targetX-gps_value[0]<0.7 and targetX-gps_value[0]>-0.7):
                print("turn left")
                turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
        
        elif compass_value[1]<= -0.9999: #facing left, move Z-axis   ##yw's facing right
            if(turning_Right_ObsFront): 
                print("moveBackCount_FaceBack", moveBackCount_FaceBack)
                moveBackCount_FaceBack = moveBackCount_FaceBack + 1       
                if(moveBackCount_FaceBack <= 40 ):
                    print("BACK")
                    move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount_FaceBack <= 50):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount_FaceBack > 50):
                    print("RIGHT")
                    turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                #got obstacle at the back
                
            elif(turning_Right_ObsBack):
                print("hopefully can run dao here")
                moveFrontCount = moveFrontCount + 1
                print("moveFrontCount_FaceBack ", moveFrontCount )
                moveFrontCount  = moveFrontCount + 1
                if(moveFrontCount <= 80 ):
                    print("FRONT")
                    move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount <= 90 ):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount > 100 and moveFrontCount < 120):
                    print("RIGHT")
                    turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)  
                print("halo")
            
            elif((fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) and not turning_Right_ObsFront and not turning_Right_ObsBack):  #face front, move forward, got obstacle in front
                print("Hitted dao leee")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Right_ObsFront = True
                moveBackCount_FaceBack = moveBackCount_FaceBack + 1
                print("turning ", turning_Front_ObsFront, turning_Front_ObsBack)
            elif((bsValue1 < 950) or (bsValue2 < 950) or (bsValue3 < 950) and not turning_Right_ObsFront and not turning_Right_ObsBack): #face front, move backward, got obstacle at the back
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Right_ObsBack = True
                moveFrontCount = moveFrontCount + 1
                print("HITTT")
            # if(targetX-gps_value[0]<0.2 and targetX-gps_value[0] >-0.2): #correct x coordinate, proceed
            elif((targetZ-gps_value[0]>=0.2 and is_Xcoordinate_correct(targetX, gps_value))):
                print("forward Y 0.2") # move forward until 0.2 if x coordinate is correct
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif(targetZ-gps_value[0]>=turning_coordinate):
                print("forward Y 0.7") #if wrong, turn when 0.7 before the coordinate
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetZ-gps_value[0]<=-0.2 and is_Xcoordinate_correct(targetX, gps_value)): #move backward
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("backward Y 0.2")
            elif (targetZ-gps_value[0]<=-turning_coordinate): #move backward
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("backward Y 0.7")
            elif (is_Zcoordinate_correct(targetZ, gps_value) and is_Xcoordinate_correct(targetX, gps_value)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            else: #wrong x coordinate, turn to x-axis
                print("turn left")
                turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                               
        elif compass_value[1]>= 0.9999: #facing left, move Z-axis   ##yw's facing left
            print("TESTTTTTTTTTT")
            print("turning_Front_ObsFront: ",turning_Left_ObsFront)
            if(turning_Left_ObsFront): 
                print("moveBackCount_FaceBack", moveBackCount_FaceBack)
                moveBackCount_FaceBack = moveBackCount_FaceBack + 1       
                if(moveBackCount_FaceBack <= 20 ):
                    print("BACK")
                    move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount_FaceBack <= 30):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveBackCount_FaceBack > 30):
                    print("LEFT")
                    turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                #got obstacle at the back
            elif(turning_Left_ObsBack):
                print("hopefully can run dao here")
                moveFrontCount = moveFrontCount + 1
                print("moveFrontCount_FaceBack ", moveFrontCount )
                moveFrontCount  = moveFrontCount + 1
                if(moveFrontCount <= 80 ):
                    print("FRONT")
                    move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount <= 90 ):   
                    print("STOP") 
                    stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                elif(moveFrontCount > 100 and moveFrontCount < 120):
                    print("LEFT")
                    turn_left(right_front_motor,left_front_motor,right_back_motor,left_back_motor)  
                print("halo")
                
            elif((fsValue1 < 950) or (fsValue2 < 950) or (fsValue3 < 950) and not turning_Left_ObsFront and not turning_Left_ObsBack):  #face front, move forward, got obstacle in front
                print("No hit dao ma?")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Left_ObsFront = True
                moveBackCount_FaceBack = moveBackCount_FaceBack + 1
                print("turning ", turning_Front_ObsFront, turning_Front_ObsBack)
            elif((bsValue1 < 950) or (bsValue2 < 950) or (bsValue3 < 950) and not turning_Left_ObsFront and not turning_Left_ObsBack): #face front, move backward, got obstacle at the back
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                # turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                turning_Left_ObsBack = True
                moveFrontCount = moveFrontCount + 1
                print("HITTT")  
            # if(targetX-gps_value[0]<0.2 and targetX-gps_value[0] >-0.2): #correct x coordinate, proceed
            elif(targetZ-gps_value[0]>=0.2 and is_Xcoordinate_correct(targetX, gps_value)):
                print("back Y 0.2") # move forward until 0.2 if x coordinate is correct
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif(targetZ-gps_value[0]>=turning_coordinate):
                print("back Y 0.7") #if wrong, turn when 0.7 before the coordinate
                move_backward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            elif (targetZ-gps_value[0]<=-0.2 and is_Xcoordinate_correct(targetX, gps_value)): #move backward
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("front Y 0.2")
            elif (targetZ-gps_value[0]<=-turning_coordinate): #move backward
                move_forward(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
                print("front Y 0.7")
            elif (is_Zcoordinate_correct(targetZ, gps_value) and is_Xcoordinate_correct(targetX, gps_value)):
                print("stoppppppppppp")
                stop(right_front_motor,left_front_motor,right_back_motor,left_back_motor)
            else: #wrong x coordinate, turn to x-axis
                print("turn right")
                turn_right(right_front_motor,left_front_motor,right_back_motor,left_back_motor)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # pass

# if__name__ == "__main__":
my_robot = Robot()
# my_compass=Compass()
run_robot(my_robot)
# Enter here exit cleanup code.
