import os # for getting absolute filepath to mitigate cross platform inconsistensies
import cv2
import numpy as np
import math

detected_circle = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite
write_data = False
draw_detected = True
display_images = True
Traffic_State = "Unknown"
prevTraffic_State = 0

debug_mode = True
def dist(a,b):
    return int( math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) ) )
def Check_Color_Cmb(HLS,center,center_cmp):
    Correct_Color_Comb = False
    A_hue=HLS[center[1]-1,center[0]-1,0]
    B_hue=HLS[center_cmp[1]-1,center_cmp[0]-1,0]
    #C_hue=HLS[int((center[1]+center_cmp[1])/2),center[0]-1,0]
    C_hue=HLS[center[1]-1,int((center[0]+center_cmp[0])/2),0]

    if( (A_hue<8) or ( (A_hue>56)and (A_hue<66) ) ):
        # A is either red or green
        if(A_hue<8):
            #A is Red then B Should be green
            if ( (B_hue>56)and (B_hue<66) ):
                print("A is Red B is green")
                if ((C_hue>28)and(C_hue<32)):
                    return True
                else:
                    print("Mid is not yello")
                    return Correct_Color_Comb          
            else:
                print("A is Red B is NOT green")
                return Correct_Color_Comb
        else:
            # A is green then B should be red
            if(B_hue<8):
                #B is red then A should be green
                if ( (A_hue>56)and (A_hue<66) ):
                    print("B is Red A is green")
                    if ((C_hue>28)and(C_hue<32)):
                        return True
                    else:
                        print("Mid is not yello")
                        return Correct_Color_Comb        
                else:
                    print("B is Red A is green")
                    return Correct_Color_Comb
    else:
        print("A is Neither Red B NOR green")
        return Correct_Color_Comb


def AreCircles_Intersecting(center,center_cmp,r1,r2):
    x1,y1=center
    x2,y2=center_cmp
    distSq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);  
    radSumSq = (r1 + r2) * (r1 + r2);  
    if (distSq == radSumSq): 
        return 1 
    elif (distSq > radSumSq): 
        return -1 
    else: 
        return 0 


def Circledetector(gray,cimg,frame_draw,HLS):
    frame_draw_special= frame_draw.copy()
    global Traffic_State,prevTraffic_State
    # 2. Apply the HoughCircles to detect the circular regions in the Image        
    NumOfVotesForCircle = 16 #parameter 1 MinVotes needed to be classified as circle
    CannyHighthresh = 230 # High threshold value for applying canny
    mindDistanBtwnCircles = 5 # kept as sign will likely not be overlapping
    max_rad = 50 # smaller circles dont have enough votes so only maxRadius need to be controlled 
                    # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,mindDistanBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=5,maxRadius=max_rad)
    TL_Update = "Unknown"
    # 3. Loop over detected Circles
    if circles is not None:
        circles = np.uint16(np.around(circles))
        # 4. Check if Circles larger then minim size
        i_count=0
        for i in circles[0,:]:
            center =(int(i[0]),int(i[1]))
            radius = int(i[2] + 5)
            if (radius !=5):
                global detected_circle
                detected_circle = detected_circle + 1 
                j_count=0
                for j in circles[0,:]:
                    if j_count!=i_count:
                        center_cmp =(int(j[0]),int(j[1]))
                        radius_cmp = int(j[2] + 5)
                        point_Dist = dist( ( center[0],center[1] ) , ( center_cmp[0],center_cmp[1] ) )
                        print("Distance between [ center = ", center, "center_cmp = ",center_cmp, " ] is  = ",point_Dist)
                        if ( (point_Dist>10) and (point_Dist<60) and ( abs(center[0]-center_cmp[0]) < 50 ) and ( abs(center[1]-center_cmp[1]) < 5 ) and (abs(radius - radius_cmp)<5) and (AreCircles_Intersecting(center,center_cmp,radius,radius_cmp)<0) ):
                            Correct_Color_Comb = Check_Color_Cmb(HLS,center,center_cmp)
                            if (Correct_Color_Comb):
                                #close enough
                                # draw the outer circle
                                cv2.circle(frame_draw_special,(i[0],i[1]),i[2],(0,255,0),1)
                                # draw the center of the circle
                                cv2.circle(frame_draw_special,(i[0],i[1]),2,(0,0,255),3)

                                # draw the outer circle
                                cv2.circle(frame_draw_special,(j[0],j[1]),j[2],(255,0,0),1)
                                # draw the center of the circle
                                cv2.circle(frame_draw_special,(j[0],j[1]),2,(0,0,255),3)
                                if debug_mode:
                                    cv2.imshow('Traffic Light Confirmed!! [Checking State!!!]',frame_draw_special)
                                
                                #If Center is Brighter
                                if( (int(HLS[center[1],center[0],1]) - int(HLS[center_cmp[1],center_cmp[0],1])) > 10 ):
                                    # Left was Brightest [Red]
                                    if(center[0]<center_cmp[0]):
                                        TL_Update = "Left was Brightest [Red]"
                                        Traffic_State="Stop"
                                    # Right was Brightest [Green]
                                    elif(center[0]>center_cmp[0]):
                                        TL_Update = "Right was Brightest [Green]"
                                        Traffic_State="Go"

                                #ElseIf Center_cmp is Brighter
                                elif( ( int(HLS[center[1],center[0],1]) - int(HLS[center_cmp[1],center_cmp[0],1]) ) < -10):
                                    # Left was Darker [Green]
                                    if(center[0]<center_cmp[0]):
                                        TL_Update = "Left was Darker [Green]"
                                        Traffic_State="Go"
                                    # Right was Darker [Red]
                                    elif(center[0]>center_cmp[0]):
                                        TL_Update = "Right was Darker [Red]"
                                        Traffic_State="Stop"

                                else:
                                    if (prevTraffic_State != "Stop"):
                                        Traffic_State= "Unknown"#Because No Traffic light is detected and we werent looking for Go then Reset Traffic State        
                                
                                print("HLS[center[1],center[0],1] = ",HLS[center[1],center[0],1], "HLS[center_cmp[1],center_cmp[0],1] = ",HLS[center_cmp[1],center_cmp[0],1])

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

        cv2.putText(frame_draw,str(circles.shape[1]),(40,50),cv2.FONT_HERSHEY_SIMPLEX,1,255)
        if display_images:
            cv2.putText(frame_draw, Traffic_State, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
            cimg_str = '[Fetch_TL_State] (2) detected circular reg'
            cv2.imshow(cimg_str,frame_draw)

        if (Traffic_State !=prevTraffic_State):
            print("#################TRAFFIC STATE CHANGED####################")
            print ("Traffic_State = ",Traffic_State)
            print ("TL_Reason = ",TL_Update)
            if debug_mode:
                cv2.waitKey(1)

        prevTraffic_State = Traffic_State

    return Traffic_State

HLS=0
src=0

Hue_Low_G = 56#66
Hue_High_G =66#66
Lit_Low_G = 66
Sat_Low_G = 23

Hue_Low_R = 0#66
Hue_High_R = 8#66
Lit_Low_R = 33
Sat_Low_R = 23

def OnHueLowChange(val):
    global Hue_Low_G
    Hue_Low_G = val
    MaskExtract()
def OnHueHighChange(val):
    global Hue_High_G
    Hue_High_G = val
    MaskExtract()
def OnLitLowChange(val):
    global Lit_Low_G
    Lit_Low_G = val
    MaskExtract()
def OnSatLowChange(val):
    global Sat_Low_G
    Sat_Low_G = val
    MaskExtract()


def OnHueLowChange_R(val):
    global Hue_Low_R
    Hue_Low_R = val
    MaskExtract()
def OnHueHighChange_R(val):
    global Hue_High_R
    Hue_High_R = val
    MaskExtract()
def OnLitLowChange_R(val):
    global Lit_Low_R
    Lit_Low_R = val
    MaskExtract()
def OnSatLowChange_R(val):
    global Sat_Low_R
    Sat_Low_R = val
    MaskExtract()

def MaskExtract():
    mask_Green   = clr_segment(HLS,(Hue_Low_G  ,Lit_Low_G   ,Sat_Low_G  ),(Hue_High_G       ,255,255))
    mask_Red   = clr_segment(HLS,(Hue_Low_R  ,Lit_Low_R   ,Sat_Low_R  ),(Hue_High_R       ,255,255))

    MASK = cv2.bitwise_or(mask_Green,mask_Red)
    MASK_Binary = MASK != 0

    dst = src * (MASK_Binary[:,:,None].astype(src.dtype))
    if debug_mode:
        cv2.imshow("mask",dst)
        cv2.imshow("mask_R",dst)
    return dst

if debug_mode:
    cv2.namedWindow("mask")
    cv2.namedWindow("mask_R")
    cv2.createTrackbar("Hue_L","mask",Hue_Low_G,255,OnHueLowChange)
    cv2.createTrackbar("Hue_H","mask",Hue_High_G,255,OnHueHighChange)
    cv2.createTrackbar("Lit_L","mask",Lit_Low_G,255,OnLitLowChange)
    cv2.createTrackbar("Sat_L","mask",Sat_Low_G,255,OnSatLowChange)
    cv2.createTrackbar("Hue_L_red","mask_R",Hue_Low_R,255,OnHueLowChange_R)
    cv2.createTrackbar("Hue_H_red","mask_R",Hue_High_R,255,OnHueHighChange_R)
    cv2.createTrackbar("Lit_L_red","mask_R",Lit_Low_R,255,OnLitLowChange_R)
    cv2.createTrackbar("Sat_L_red","mask_R",Sat_Low_R,255,OnSatLowChange_R)    

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
    
    # 0. To be accessed in Script Functions without explicitly passing as an Argument
    global HLS,src
    src = frame.copy()
    
    # 1. Converting frame to HLS ColorSpace
    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc
    # 2. Extracting Mask of Only Red And Color Regions
    frame_ROI = MaskExtract()
    #if debug_mode:
        #Lightness = HLS[:,:,1]
        #cv2.imshow("Lightness",Lightness)
    # 1. Cvt frame_ROI to grayscale
    gray = cv2.cvtColor(frame_ROI,cv2.COLOR_BGR2GRAY)
    # Localizing Potetial Candidates and Classifying them in SignDetection    
    Traffic_State = Circledetector(gray.copy(),frame.copy(),frame_draw,HLS)

    return Traffic_State