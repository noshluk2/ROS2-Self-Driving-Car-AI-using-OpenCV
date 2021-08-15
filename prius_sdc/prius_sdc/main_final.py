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

# >>>>>>>>>>>>>>>>>> OPTIMIZING CODE # 3 [Threading] (PyImageSearch) <<<<<<<<<<<<<<<<<<<<<<
# Threading Controls
Use_Threading = False
Live_Testing = False
if Use_Threading:
    from imutils.video.pivideostream import PiVideoStream



def main():

    if Use_Threading:
        # start the file video stream thread and allow the buffer to fill
        PI_vs = PiVideoStream().start()

    else:
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
        if Use_Threading:
            img = PI_vs.read().copy()
        else:
            ret,img = cap.read()
            #ROI = cv2.selectROI(img)
            #print(ROI)
            #img = img[ROI[1]:ROI[1]+ROI[3],ROI[0]:ROI[0]+ROI[2]]
            #img = img[81:381,105:535] # Cropping for 640p
            #img = img[0:640,252:1227] # Cropping for 720p
            #img = img[0:640,302:1167] # Cropping for 720p Less Skew
            img = img[0:640,402:1267] # Cropping for 720p Less Skew Righty
            #cv2.imshow("img",img)
            #cv2.waitKey(0)
            if not ret:
                break
        # >>>>>>>>>>>>>>>>>>>>>>>> Optimization No 1 [RESIZING]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if not Use_Threading:
            img = cv2.resize(img,(320,240))
        #CropHeight = 130
        #minArea = 250
        #CropHeight = 260
        #minArea = 500
        
        img_orig = img.copy()# Keep it for

        # ****************************************************  DETECTION [LANES] ****************************************************

        # 1. Extracting Information that best defines relation between lanes and Car
        distance, Curvature = Detect_Lane(img)

        # ****************************************************  DETECTION  [LANES] ****************************************************

        # ****************************************************  DETECTION  [SIGNS] ****************************************************
        Disp_img = img.copy()

        # 2. Detecting and extracting information from the signs
        Mode , Tracked_class = detect_Signs(img_orig,img)
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

            
        # ****************************************************  DETECTION  [SIGNS] ****************************************************
        
        # 3. Drive Car On Basis of Current State Info provided by Detection module
        if (config.Activat_LeftTurn == True):
            distance = Frozen_Dist
            Curvature = Frozen_Curvature

        Current_State = [distance, Curvature , img , Mode , Tracked_class]
        Drive_Car(Current_State)
        
        config.loopCount = config.loopCount+1
        if(config.loopCount==50000):
            break

        cv2.imshow("Frame",img)
        k = cv2.waitKey(waitTime)
        if k==27:
            break


    if Use_Threading:
        PI_vs.stop() 


if __name__ == '__main__':
	main()