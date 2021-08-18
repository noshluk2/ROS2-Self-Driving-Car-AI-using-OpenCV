import os # for getting absolute filepath to mitigate cross platform inconsistensies
import cv2
import time
import numpy as np
import config
import math

detected_circle = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite
write_data = False
draw_detected = True
display_images = True

def dist(a,b):
    test= (a[1]-b[1])**2  +  (a[0]-b[0])**2 
    print(test)
    return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )

def Circledetector(gray,cimg,frame_draw):
    frame_draw_special= frame_draw.copy()

    # 2. Apply the HoughCircles to detect the circular regions in the Image        
    NumOfVotesForCircle = 25 #parameter 1 MinVotes needed to be classified as circle
    CannyHighthresh = 200 # High threshold value for applying canny
    mindDistanBtwnCircles = 1 # kept as sign will likely not be overlapping
    max_rad = 150 # smaller circles dont have enough votes so only maxRadius need to be controlled 
                    # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,mindDistanBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=5,maxRadius=max_rad)
    
    # 3. Loop over detected Circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        # 4. Check if Circles larger then minim size
        i_count=0
        for i in circles[0,:]:
            center =(i[0],i[1])
            radius = i[2] + 5
            if (radius !=5):
                global detected_circle
                detected_circle = detected_circle + 1 
                j_count=0
                for j in circles[0,:]:
                    if j_count!=i_count:
                        center_cmp =(j[0],j[1])
                        radius_cmp = j[2] + 5
                        print("center = ",center, "center_cmp = ",center_cmp)
                        point_Dist = dist( ( int(center[0]),int(center[1]) ) , ( int(center_cmp[0]),int(center_cmp[1]) ) )
                        if ( (point_Dist>10) and (point_Dist<50) and ( abs(center[0]-center_cmp[0]) < 50 ) ):
                            #close enough
                            # draw the outer circle
                            cv2.circle(frame_draw_special,(i[0],i[1]),i[2],(0,255,0),1)
                            # draw the center of the circle
                            cv2.circle(frame_draw_special,(i[0],i[1]),2,(0,0,255),3)

                            # draw the outer circle
                            cv2.circle(frame_draw_special,(j[0],j[1]),j[2],(255,0,0),1)
                            # draw the center of the circle
                            cv2.circle(frame_draw_special,(j[0],j[1]),2,(0,0,255),3)
                            cv2.imshow('frame_draw_special',frame_draw_special)
                        j_count=j_count+1

                i_count=i_count+1


                startP = (center[0]-radius,center[1]-radius)
                endP = (center[0]+radius,center[1]+radius)
                detected_sign = cimg[startP[1]:endP[1],startP[0]:endP[0]]

                if(detected_sign.shape[1] and detected_sign.shape[0]):
                    if draw_detected:
                        # draw the outer circle
                        cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),1)
                        # draw the center of the circle
                        cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3)
                        #cv2.imshow('circle',detected_sign)


        
        if display_images:
            cimg_str = 'detected circles'
            cv2.imshow(cimg_str,frame_draw)
            #cv2.waitKey(1)


HLS=0
src=0
Hue_Low = 56#66
Lit_Low = 66
Sat_Low = 47

def OnHueLowChange(val):
    global Hue_Low
    Hue_Low = val
    MaskExtract()
def OnLitLowChange(val):
    global Lit_Low
    Lit_Low = val
    MaskExtract()
def OnSatLowChange(val):
    global Sat_Low
    Sat_Low = val
    MaskExtract()

def MaskExtract():
    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(255       ,255,255))
    mask_ = mask != 0
    dst = src * (mask_[:,:,None].astype(src.dtype))
    cv2.imshow("mask",dst)
    return dst


cv2.namedWindow("mask")

cv2.createTrackbar("Hue_L","mask",Hue_Low,255,OnHueLowChange)
cv2.createTrackbar("Lit_L","mask",Lit_Low,255,OnLitLowChange)
cv2.createTrackbar("Sat_L","mask",Sat_Low,255,OnSatLowChange)  

def clr_segment(HSL,lower_range,upper_range):
    
    # 2. Performing Color Segmentation on Given Range
    lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
    upper = np.array( [upper_range[0]    ,255     ,255])
    mask = cv2.inRange(HSL, lower, upper)
    
    # 3. Dilating Segmented ROI's
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask


def detect_TrafficLight(frame,frame_draw):
    
    global HLS,src
    src = frame.copy()
    
    # 1. Converting frame to HLS ColorSpace
    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc

    #mask = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(255       ,255,255))
    frame_ROI = MaskExtract()
    #cv2.imshow("mask",mask)

    # 1. Cvt frame to grayscale
    #gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(frame_ROI,cv2.COLOR_BGR2GRAY)

    # Localizing Potetial Candidates and Classifying them in SignDetection    
    Circledetector(gray.copy(),frame.copy(),frame_draw)