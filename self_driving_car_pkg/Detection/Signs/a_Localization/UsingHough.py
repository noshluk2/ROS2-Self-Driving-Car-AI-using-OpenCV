import os # for getting absolute filepath to mitigate cross platform inconsistensies
import cv2
import time
import numpy as np
import config

detected_circle = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite
write_data = False
draw_detected = True
display_images = True

def Circledetector(gray,cimg,frame_draw):

    # 2. Apply the HoughCircles to detect the circular regions in the Image        
    NumOfVotesForCircle = 25 #parameter 1 MinVotes needed to be classified as circle
    CannyHighthresh = 200 # High threshold value for applying canny
    mindDistanBtwnCircles = 10 # kept as sign will likely not be overlapping
    max_rad = 150 # smaller circles dont have enough votes so only maxRadius need to be controlled 
                    # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,mindDistanBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=5,maxRadius=max_rad)
    
    # 3. Loop over detected Circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        # 4. Check if Circles larger then minim size
        for i in circles[0,:]:
            center =(i[0],i[1])
            radius = i[2] + 5
            if (radius !=5):
                global detected_circle
                detected_circle = detected_circle + 1 

                startP = (center[0]-radius,center[1]-radius)
                endP = (center[0]+radius,center[1]+radius)
                detected_sign = cimg[startP[1]:endP[1],startP[0]:endP[0]]

                if(detected_sign.shape[1] and detected_sign.shape[0]):
                    if draw_detected:
                        # draw the outer circle
                        cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),1)
                        # draw the center of the circle
                        cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3)

        
        if display_images:
            cimg_str = 'detected circles'
            cv2.imshow(cimg_str,frame_draw)
            #cv2.waitKey(1)

def detect_Circles(frame,frame_draw):
    
    # 1. Cvt frame to grayscale
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Localizing Potetial Candidates and Classifying them in SignDetection    
    Circledetector(gray.copy(),frame.copy(),frame_draw)