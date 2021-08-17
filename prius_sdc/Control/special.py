from numpy import interp

run_car=True
prev_Mode = "Detection"
if run_car:
    car_speed=80
else:
    car_speed=0


import cv2


def beInLane(Max_Sane_dist,distance,curvature , Mode , Tracked_class):

    IncreaseTireSpeedInTurns = True
    global car_speed,prev_Mode
    if((Tracked_class!=0) and (prev_Mode == "Tracking") and (Mode == "Detection")):
        if  (Tracked_class =="speed_sign_70"):
            car_speed = 70
        elif(Tracked_class =="speed_sign_80"):
            car_speed = 80
        elif(Tracked_class =="left_turn"):
            car_speed = 50
        elif(Tracked_class =="stop"):
            car_speed = 0
        
    prev_Mode = Mode # Set prevMode to current Mode
    
    Max_turn_angle = 90
    Max_turn_angle_neg = -90

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
        print("Turn_angle_interpolated = ", Turn_angle_interpolated)
        CarTurn_angle = Turn_angle_interpolated + curvature

    # Handle Max Limit [if (greater then either limits) --> set to max limit]
    if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
        if(CarTurn_angle > Max_turn_angle):
            CarTurn_angle = Max_turn_angle
        else:
            CarTurn_angle = -Max_turn_angle

    angle = interp(CarTurn_angle,[-Max_turn_angle,Max_turn_angle],[30,120])
    # 30 - 75  Left   2 Middle
    # 75 - 120 Middle 2 Right
    curr_speed = car_speed
    
    if (IncreaseTireSpeedInTurns and (Tracked_class !="left_turn")):
        if(angle>95):
            car_speed_turn = interp(angle,[95,120],[80,100])
            curr_speed = car_speed_turn
        elif(angle<55):
            car_speed_turn = interp(angle,[30,55],[100,80])
            curr_speed = car_speed_turn

    
    return angle , curr_speed

def Steer(Distance,Curvature,frame , Mode , Tracked_class):
    angle_of_car=0;current_speed=0;
    if((Distance != -1000) and (Curvature != -1000)):

        angle_of_car , current_speed = beInLane(int(frame.shape[1]/4), Distance,Curvature , Mode , Tracked_class )
        angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + " , " + str(int(current_speed)) + " ] "
        #cv2.putText(frame_disp,str(angle_of_car),(frame.shape[1]-400,50),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),2)
        if (angle_of_car <55):
            direction_string="[ Moving Left ]"
            color_direction=(0,0,255)
        elif (angle_of_car >90):
            direction_string="[ Moving Right ]"
            color_direction=(0,0,255)
        else:
            direction_string="[ Straight ]"
            color_direction=(0,255,0)


        cv2.putText(frame,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.5,(0,0,255),1)
        cv2.putText(frame,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)
    else:
        angle_speed_str = "[ Angle ,Speed ] = [ " + str("N/A") + " , " + str("N/A") + " ] "
        cv2.putText(frame,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)
   
    return angle_of_car,current_speed


def Drive_Car(Current_State):
    [distance, Curvature, frame_disp , Mode , Tracked_class] = Current_State
    a,b=Steer(distance,Curvature,frame_disp , Mode , Tracked_class)    
    return a,b