#git push from raspberry pi
#Control Variables for 3c_threaded_Mod4
import os
import cv2

detect = 1 # Set to 1 for Lane detection

Testing = True# Set to True --> if want to see what the car is seeing
Profiling = False # Set to True --> If you want to profile code
write = False # Set to True --> If you want to Write input / output videos
In_write = False
Out_write = False

debugging = True # Set to True --> If you want to debug code

debugging_Lane = True

debugging_L_ColorSeg = True
debugging_L_Est= True
debugging_L_Cleaning= True
debugging_L_LaneInfoExtraction= True

debugging_Signs = True
debugging_TrafficLights = True
debugging_TL_Config = True

Detect_lane_N_Draw = True
Training_CNN = False 

vid_path = os.path.abspath("data/vids/Ros2/lane.avi")
loopCount=0


Resized_width = 320#320#240#640#320 # Control Parameter
Resized_height = 240#240#180#480#240

in_q = cv2.VideoWriter( os.path.abspath("data/Output/in_new.avi") , cv2.VideoWriter_fourcc('M','J','P','G'), 30, (Resized_width,Resized_height))
out  = cv2.VideoWriter( os.path.abspath("data/Output/out_new.avi") , cv2.VideoWriter_fourcc('M','J','P','G'), 30, (Resized_width,Resized_height))

if debugging:
    waitTime = 1
else:
    waitTime = 1

#============================================ Paramters for Lane Detection =======================================
Ref_imgWidth = 1920
Ref_imgHeight = 1080

#Ref_imgWidth = 640
#Ref_imgHeight = 480

Frame_pixels = Ref_imgWidth * Ref_imgHeight

Resize_Framepixels = Resized_width * Resized_height

Lane_Extraction_minArea_per = 1000 / Frame_pixels
minArea_resized = int(Resize_Framepixels * Lane_Extraction_minArea_per)

BWContourOpen_speed_MaxDist_per = 500 / Ref_imgHeight
MaxDist_resized = int(Resized_height * BWContourOpen_speed_MaxDist_per)

CropHeight = 600 # Required in Camera mounted on top of car 640p
CropHeight_resized = int( (CropHeight / Ref_imgHeight ) * Resized_height )