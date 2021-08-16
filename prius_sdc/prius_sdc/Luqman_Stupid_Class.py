import cv2
import numpy as np
import os
import config

# ****************************************************  DETECTION ****************************************************
# ****************************************************    LANES   ****************************************************
from Detection.Lanes.Lane_Detection import Detect_Lane
# ****************************************************    SIGNS   ****************************************************
from Detection.Signs.SignDetectionApi import detect_Signs


# ****************************************************   CONTROL  *****************************************************
from Control.special import Drive_Car
#from Control.Drive import Drive_Car


Live_Testing = False

def main():

    if Live_Testing:
        cap = cv2.VideoCapture(0)
    else:
        #cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/signs_and_road.avi"))#720p
        #cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/signs_training.avi"))#720p
        #cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/full_track_drive.avi"))#720p
        cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/ishara+turning.avi"))#720p
        #cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/NEW_full_track_tour.mp4"))#720p

    waitTime = 0
    
    prev_Mode = "Detection"
    Left_turn_iterations = 0

    while(1):

        ret,img = cap.read()
        if not ret:
            break

        img = img[0:640,402:1267] # Cropping for 720p Less Skew Righty


        # >>>>>>>>>>>>>>>>>>>>>>>> Optimization No 1 [RESIZING]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        img = cv2.resize(img,(320,240))
        img_orig = img.copy()# Keep it for

        # ****************************************************  DETECTION [LANES] ****************************************************
        # 1. Extracting Information that best defines relation between lanes and Car
        distance, Curvature = Detect_Lane(img)
        # ****************************************************  DETECTION  [LANES] ****************************************************

        # ****************************************************  DETECTION  [SIGNS] ****************************************************
        # 2. Detecting and extracting information from the signs
        Mode , Tracked_class = detect_Signs(img_orig,img)
        # ****************************************************  DETECTION  [SIGNS] ****************************************************

        # ****************************************************  DETECTION  [LeftHandTurn] ****************************************************
        if ( (prev_Mode =="Detection") and (Mode=="Tracking") and (Tracked_class=="left_turn") ):
            prev_Mode = "Tracking"
            config.Activat_LeftTurn = True
            Frozen_Dist = distance
            Frozen_Curvature = Curvature
            print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Tracking <<<<<<<<<<<<<<<<<<<<<<<<<<<")
        elif ( (prev_Mode =="Tracking") and (Mode=="Detection") and (Tracked_class=="left_turn") ):
            # Car just besides left turn,
            # Keep moving left 60 degrees for 100 iterations 
            print("Left Activated")
            print("config.Activat_LeftTurn ",config.Activat_LeftTurn)
            if ((Left_turn_iterations % 2 ) ==0):
                Frozen_Curvature = Frozen_Curvature -1 # Move left by 1 degree 
            if(Left_turn_iterations==100):
                print("Left DeActivated")
                # Dectivate Left turn by Setting prev_Mode = "Detection"
                prev_Mode = "Detection"
                config.Activat_LeftTurn = False
                Left_turn_iterations = 0
            Left_turn_iterations = Left_turn_iterations + 1
        # ****************************************************  DETECTION  [LeftHandTurn] ****************************************************

        # ****************************************************  CONTROL ****************************************************
        # 3. Drive Car On Basis of Current State Info provided by Detection module
        if (config.Activat_LeftTurn == True):
            distance = Frozen_Dist
            Curvature = Frozen_Curvature
        Current_State = [distance, Curvature , img , Mode , Tracked_class]
        Drive_Car(Current_State)
        # ****************************************************  CONTROL ****************************************************
        
        cv2.imshow("Frame",img)
        k = cv2.waitKey(waitTime)
        if k==27:
            break
 


if __name__ == '__main__':
	main()