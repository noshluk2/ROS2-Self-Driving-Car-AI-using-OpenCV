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
clr_segmentation_tuning = True # Set to True --> If you want to tune color segmentation parameters

Detect_lane_N_Draw = True

vid_path = os.path.abspath("Detection/Lanes/Inputs/signs_forward.mp4")
#vid_path = os.path.abspath("Detection/Lanes/Inputs/in_16_2.avi")
loopCount=0


Resized_width = 320#240#640#320 # Control Parameter
Resized_height = 240#180#480#240

in_q = cv2.VideoWriter( os.path.abspath("Detection/Lanes/Results/in_new.avi") , cv2.VideoWriter_fourcc('M','J','P','G'), 30, (Resized_width,Resized_height))
out  = cv2.VideoWriter( os.path.abspath('Detection/Lanes/Results/out_new.avi') , cv2.VideoWriter_fourcc('M','J','P','G'), 30, (Resized_width,Resized_height))

if debugging:
    waitTime = 0
else:
    waitTime = 1

#============================================ Paramters for Lane Detection =======================================
Ref_imgWidth = 1920
Ref_imgHeight = 1080
Frame_pixels = Ref_imgWidth * Ref_imgHeight

Resize_Framepixels = Resized_width * Resized_height

Lane_Extraction_minArea_per = 1500 / Frame_pixels
minArea_resized = int(Resize_Framepixels * Lane_Extraction_minArea_per)

BWContourOpen_speed_MaxDist_per = 600 / Ref_imgHeight
MaxDist_resized = int(Resized_height * BWContourOpen_speed_MaxDist_per)

CropHeight = 600
CropHeight_resized_crop = int( (CropHeight / Ref_imgHeight ) * Resized_height )