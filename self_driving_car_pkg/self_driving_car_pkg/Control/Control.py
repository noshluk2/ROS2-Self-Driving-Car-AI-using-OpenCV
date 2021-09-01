from numpy import interp

run_car=True
prev_Mode = "Detection"
if run_car:
    car_speed=80
else:
    car_speed=0

import cv2

angle_of_car=0

def beInLane(Max_Sane_dist,distance,curvature , Mode , Tracked_class):

    IncreaseTireSpeedInTurns = True
    global car_speed,prev_Mode

    if((Tracked_class!=0) and (prev_Mode == "Tracking") and (Mode == "Detection")):
        if  (Tracked_class =="speed_sign_30"):
            car_speed = 30
        elif(Tracked_class =="speed_sign_60"):
            car_speed = 60
        elif(Tracked_class =="speed_sign_90"):
            car_speed = 90
        elif(Tracked_class =="left_turn"):
            car_speed = 50
        elif(Tracked_class =="stop"):
            car_speed = 0
        
    prev_Mode = Mode # Set prevMode to current Mode
    
    Max_turn_angle_neg = -90
    Max_turn_angle = 90

    CarTurn_angle = 0

    if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
        # Max sane distance reached ---> Max penalize (Max turn Tires)
        if(distance > Max_Sane_dist):
            #Car offseted left --> Turn full wheels right
            CarTurn_angle = Max_turn_angle + curvature
        else:
            #Car Offseted right--> Turn full wheels left
            CarTurn_angle = Max_turn_angle_neg + curvature
    else:
        # Within allowed distance limits for car and lane
        # Interpolate distance to Angle Range
        Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
        CarTurn_angle = Turn_angle_interpolated + curvature

    # Handle Max Limit [if (greater then either limits) --> set to max limit]
    if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
        if(CarTurn_angle > Max_turn_angle):
            CarTurn_angle = Max_turn_angle
        else:
            CarTurn_angle = -Max_turn_angle

    #angle = CarTurn_angle
    angle = interp(CarTurn_angle,[-90,90],[-45,45])

    curr_speed = car_speed
    
    if (IncreaseTireSpeedInTurns and (Tracked_class !="left_turn")):
        if(angle>30):
            car_speed_turn = interp(angle,[30,45],[80,100])
            curr_speed = car_speed_turn
        elif(angle<-30):
            car_speed_turn = interp(angle,[-45,-30],[100,80])
            curr_speed = car_speed_turn

    
    return angle , curr_speed

def Steer(Distance,Curvature,frame , Mode , Tracked_class):

    current_speed=0
    
    global angle_of_car

    if((Distance != -1000) and (Curvature != -1000)):

        angle_of_car , current_speed = beInLane(int(frame.shape[1]/4), Distance,Curvature , Mode , Tracked_class )
        if (angle_of_car <-30):
            direction_string="[ Moving Left ]"
            color_direction=(0,0,255)
        elif (angle_of_car >30):
            direction_string="[ Moving Right ]"
            color_direction=(0,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)

        cv2.putText(frame,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)
    elif(Tracked_class=="left_turn"):
        current_speed = 50
   
    return angle_of_car,current_speed


def Drive_Car(Current_State):
    [distance, Curvature, frame_disp , Mode , Tracked_class] = Current_State
    a,b=Steer(distance,Curvature,frame_disp , Mode , Tracked_class)    
    return a,b