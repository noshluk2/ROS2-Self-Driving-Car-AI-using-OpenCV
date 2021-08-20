import cv2

car_speed=65

def beInLane(Max_Sane_dist,distance,curvature):
    
    # >>> Step 1: Compute Car Offset <<<
    Max_turn_angle = 90
    Max_turn_angle_neg = -90
    CarTurn_angle = 0

    # 1a. Check if (RoadLane Out of Bounds) then proceed
    if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ): # Max sane distance reached ---> Max penalize (Max turn Tires)
        if(distance > Max_Sane_dist): #Car offseted left --> Turn full wheels right
            CarTurn_angle = Max_turn_angle + curvature
        else:                         #Car Offseted right--> Turn full wheels left
            CarTurn_angle = Max_turn_angle_neg + curvature
    
    # 1b. Else (Normal Condition)
    else: # Within allowed distance limits for car and lane
        Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90]) # Interpolate distance to Angle Range
        CarTurn_angle = Turn_angle_interpolated + curvature

    # 2. Handle Max Limit [if (greater then either limits) --> set to max limit] <<<
    if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
        if(CarTurn_angle > Max_turn_angle):
            CarTurn_angle = Max_turn_angle
        else:
            CarTurn_angle = -Max_turn_angle

    # 3. Translate proposed Correction in Front tire Steerin  <<<
    angle = interp(CarTurn_angle,[-Max_turn_angle,Max_turn_angle],[30,120])    

    setServoAngle(int(angle))
    return angle , car_speed
