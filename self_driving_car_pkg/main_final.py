import cv2
import numpy as np
from numpy import interp
import os
import config

# ****************************************************  DETECTION ****************************************************
# ****************************************************    LANES   ****************************************************
from Detection.Lanes.Lane_Detection import Detect_Lane
# ****************************************************    SIGNS   ****************************************************
from Detection.Signs.SignDetectionApi import detect_Signs

from .Detection.Signs.a_Localization.TLD import detect_TrafficLight

# ****************************************************   CONTROL  *****************************************************
from Control.special import Drive_Car
#from Control.Drive import Drive_Car


Live_Testing = False

def main():

    if Live_Testing:
        cap = cv2.VideoCapture(0)
    else:
        vid_Path=os.path.abspath("self_driving_car_pkg/data/vids/ishara_turning.avi").replace('\\', '/')
        cap = cv2.VideoCapture(vid_Path)

    waitTime = 0
    
    prev_Mode = "Detection"
    Left_turn_iterations = 0

    while(1):

        ret,frame = cap.read()
        if not ret:
            break

        img = frame[0:640,238:1042] 
        img = cv2.resize(img,(320,240))
        img_orig = img.copy()
        frame_draw = img.copy()
        Traffic_State = detect_TrafficLight(img,frame_draw)

        distance, Curvature = Detect_Lane(img)
        Mode , Tracked_class = detect_Signs(img_orig,img)
        if ( (prev_Mode =="Detection") and (Mode=="Tracking") and (Tracked_class=="left_turn") ):
            prev_Mode = "Tracking"
            config.Activat_LeftTurn = True
            Frozen_Curvature = distance
            Frozen_Curvature = Curvature
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tracking <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        elif ( (prev_Mode =="Tracking") and (Mode=="Detection") and (Tracked_class=="left_turn") ):
            print("Left Activated")
            print("config.Activat_LeftTurn ",config.Activat_LeftTurn)
            if ( ((Left_turn_iterations % 24 ) ==0) and (Left_turn_iterations>25) ):
                Frozen_Curvature = Frozen_Curvature -1 # Move left by 1 degree 
            if(Left_turn_iterations==100):
                print("Left DeActivated")
                prev_Mode = "Detection"
                config.Activat_LeftTurn = False
                Left_turn_iterations = 0
            Left_turn_iterations = Left_turn_iterations + 1
            if (config.Activat_LeftTurn == True):
                distance = Frozen_Curvature
                Curvature = Frozen_Curvature
        Current_State = [distance, Curvature , img , Mode , Tracked_class]
        a,b=Drive_Car(Current_State)
        a=interp(a,[30,120],[0.5,-0.5])
        b=interp(b,[50,90],[1,2])
        print("\n\nComputed A = ",a,"   B = ", b,"\n\n")
        a=a

        if(Traffic_State == "Stop"):
            b = 0.0 # Noob luqman
        else:
            b = b
        print("\n\nAfter Traffic Light A = ",a,"   B = ", b,"\n\n")


        cv2.putText(img,Traffic_State,(20,20),cv2.FONT_HERSHEY_COMPLEX,2,255)
        cv2.imshow("Frame",img)
        cv2.waitKey(1)
 


if __name__ == '__main__':
	main()