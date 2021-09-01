import cv2


def Display_CarDriveState(frame_disp,angle_of_car,current_speed,Tracked_class,Traffic_State,Detected_LeftTurn, Activat_LeftTurn):
    
    ###################################################  Displaying CONTROL STATE ####################################

    if (angle_of_car <-22):
        direction_string="[ Left ]"
        color_direction=(120,0,255)
    elif (angle_of_car >22):
        direction_string="[ Right ]"
        color_direction=(120,0,255)
    else:
        direction_string="[ Straight ]"
        color_direction=(0,255,0)

    if(current_speed>0):
        direction_string = "Moving --> "+ direction_string
    else:
        color_direction=(0,0,255)


    cv2.putText(frame_disp,str(direction_string),(20,40),cv2.FONT_HERSHEY_DUPLEX,0.4,color_direction,1)

    angle_speed_str = "[ Angle ,Speed ] = [ " + str(int(angle_of_car)) + "deg ," + str(int(current_speed)) + "mph ]"
    cv2.putText(frame_disp,str(angle_speed_str),(20,20),cv2.FONT_HERSHEY_DUPLEX,0.4,(0,0,255),1)

    cv2.putText(frame_disp,"Traffic Light State = [ "+Traffic_State+" ] ",(20,60),cv2.FONT_HERSHEY_COMPLEX,0.35,255)
    
    if (Tracked_class=="left_turn"):
        font_Scale = 0.30
        if (Detected_LeftTurn):
            Tracked_class = Tracked_class + " : Detected { True } "
        else:
            Tracked_class = Tracked_class + " : Activated { "+ str(Activat_LeftTurn) + " } "
    else:
        font_Scale = 0.35
    cv2.putText(frame_disp,"Sign Detected ==> "+str(Tracked_class),(20,85),cv2.FONT_HERSHEY_COMPLEX,font_Scale,(255,0,0),1)