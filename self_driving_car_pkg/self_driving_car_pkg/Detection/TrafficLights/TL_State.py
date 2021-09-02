import cv2
import numpy as np
import math
from ...config import config

class TL_States:

    def __init__(self):
        self.detected_circle = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite
        self.Traffic_State = "Unknown"
        self.prevTraffic_State = 0

        self.write_data = False
        self.draw_detected = True
        self.display_images = True

        self.HLS = 0
        self.src = 0

    # Below Are Class Variables that will be same for each instance of the class
    # As all bots will be in same environments so these variables will be same
    Hue_Low_G  = 56
    Hue_High_G = 66
    Lit_Low_G  = 41
    Sat_Low_G  = 23

    Hue_Low_R  = 0
    Hue_High_R = 8
    Lit_Low_R  = 33
    Sat_Low_R  = 23

    def dist(self,a,b):
        return int( math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) ) )

    def Check_Color_Cmb(self,center,center_cmp):
        Correct_Color_Comb = False
        A_hue=self.HLS[center[1]-1,center[0]-1,0]
        B_hue=self.HLS[center_cmp[1]-1,center_cmp[0]-1,0]
        C_hue=self.HLS[center[1]-1,int((center[0]+center_cmp[0])/2),0]

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

    def AreCircles_Intersecting(self,center,center_cmp,r1,r2):
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

    def Circledetector(self,gray,cimg,frame_draw):

        frame_draw_special= frame_draw.copy()
        self.Traffic_State,self.prevTraffic_State
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
                center =(int(i[0])-1,int(i[1])-1)
                radius = int(i[2] + 5)
                if (radius !=5):
                    self.detected_circle = self.detected_circle + 1 
                    j_count=0
                    for j in circles[0,:]:
                        if j_count!=i_count:
                            center_cmp =(int(j[0])-1,int(j[1])-1)
                            radius_cmp = int(j[2] + 5)
                            point_Dist = self.dist( ( center[0],center[1] ) , ( center_cmp[0],center_cmp[1] ) )
                            print("Distance between [ center = ", center, "center_cmp = ",center_cmp, " ] is  = ",point_Dist)
                            if ( (point_Dist>10) and (point_Dist<80) and ( abs(center[0]-center_cmp[0]) < 80 ) and ( abs(center[1]-center_cmp[1]) < 5 ) and (abs(radius - radius_cmp)<5) and (self.AreCircles_Intersecting(center,center_cmp,radius,radius_cmp)<0) ):
                                Correct_Color_Comb = self.Check_Color_Cmb(center,center_cmp)
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
                                    if (config.debugging and config.debugging_TrafficLights):
                                        cv2.imshow('Traffic Light Confirmed!! [Checking State!!!]',frame_draw_special)
                                    else:
                                        cv2.destroyWindow('Traffic Light Confirmed!! [Checking State!!!]')
                                    #If Center is Brighter
                                    if( (int(self.HLS[center[1],center[0],1]) - int(self.HLS[center_cmp[1],center_cmp[0],1])) > 10 ):
                                        # Left was Brightest [Red]
                                        if(center[0]<center_cmp[0]):
                                            TL_Update = "Left was Brightest [Red]"
                                            self.Traffic_State="Stop"
                                        # Right was Brightest [Green]
                                        elif(center[0]>center_cmp[0]):
                                            TL_Update = "Right was Brightest [Green]"
                                            self.Traffic_State="Go"

                                    #ElseIf Center_cmp is Brighter
                                    elif( ( int(self.HLS[center[1],center[0],1]) - int(self.HLS[center_cmp[1],center_cmp[0],1]) ) < -10):
                                        # Left was Darker [Green]
                                        if(center[0]<center_cmp[0]):
                                            TL_Update = "Left was Darker [Green]"
                                            self.Traffic_State="Go"
                                        # Right was Darker [Red]
                                        elif(center[0]>center_cmp[0]):
                                            TL_Update = "Right was Darker [Red]"
                                            self.Traffic_State="Stop"
                                    else:
                                        if (self.prevTraffic_State != "Stop"):
                                            self.Traffic_State= "Unknown"#Because No Traffic light is detected and we werent looking for Go then Reset Traffic State        
                                    
                                    print("HLS[center[1],center[0],1] = ",self.HLS[center[1],center[0],1], "HLS[center_cmp[1],center_cmp[0],1] = ",self.HLS[center_cmp[1],center_cmp[0],1])

                            j_count=j_count+1

                    i_count=i_count+1


                    startP = (center[0]-radius,center[1]-radius)
                    endP = (center[0]+radius,center[1]+radius)
                    detected_sign = cimg[startP[1]:endP[1],startP[0]:endP[0]]

                    if(detected_sign.shape[1] and detected_sign.shape[0]):
                        if self.draw_detected:
                            # draw the outer circle
                            cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),1)
                            # draw the center of the circle
                            cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3)
                            #cv2.imshow('circle',detected_sign)

            cv2.putText(frame_draw,str(circles.shape[1]),(40,50),cv2.FONT_HERSHEY_SIMPLEX,1,255)
            if self.display_images:
                cv2.putText(frame_draw, self.Traffic_State, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
                
                if (config.debugging and config.debugging_TrafficLights):
                    cimg_str = '[Fetch_TL_State] (2) detected circular reg'
                    cv2.imshow(cimg_str,frame_draw)
                else:
                    cv2.destroyWindow('[Fetch_TL_State] (2) detected circular reg')    

            if (self.Traffic_State !=self.prevTraffic_State):
                print("#################TRAFFIC STATE CHANGED####################")
                print ("self.Traffic_State = ",self.Traffic_State)
                print ("TL_Reason = ",TL_Update)
                if (config.debugging and config.debugging_TrafficLights):
                    cv2.waitKey(1)

            self.prevTraffic_State = self.Traffic_State

        return self.Traffic_State


    def clr_segment(self,lower_range,upper_range):
        
        # 2. Performing Color Segmentation on Given Range
        lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
        upper = np.array( [upper_range[0]    ,255     ,255])
        mask = cv2.inRange(self.HLS, lower, upper)
        
        # 3. Dilating Segmented ROI's
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        return mask

    def MaskExtract(self):
        mask_Green   = self.clr_segment( (self.Hue_Low_G  ,self.Lit_Low_G   ,self.Sat_Low_G ), (self.Hue_High_G       ,255,255) )
        mask_Red     = self.clr_segment( (self.Hue_Low_R  ,self.Lit_Low_R   ,self.Sat_Low_R ), (self.Hue_High_R       ,255,255) )

        MASK = cv2.bitwise_or(mask_Green,mask_Red)
        MASK_Binary = MASK != 0

        dst = self.src * (MASK_Binary[:,:,None].astype(self.src.dtype))

        if (config.debugging and config.debugging_TrafficLights):
            cv2.imshow("mask",dst)
            cv2.imshow("mask_R",dst)
        else:
            cv2.destroyWindow("mask")    
            cv2.destroyWindow("mask_R")    

        return dst

    def OnHueLowChange(self,val):
        self.Hue_Low_G = val
        self.MaskExtract()
    def OnHueHighChange(self,val):
        self.Hue_High_G = val
        self.MaskExtract()
    def OnLitLowChange(self,val):
        self.Lit_Low_G = val
        self.MaskExtract()
    def OnSatLowChange(self,val):
        self.Sat_Low_G = val
        self.MaskExtract()


    def OnHueLowChange_R(self,val):
        self.Hue_Low_R = val
        self.MaskExtract()
    def OnHueHighChange_R(self,val):
        self.Hue_High_R = val
        self.MaskExtract()
    def OnLitLowChange_R(self,val):
        self.Lit_Low_R = val
        self.MaskExtract()
    def OnSatLowChange_R(self,val):
        self.Sat_Low_R = val
        self.MaskExtract()


    def Get_TL_State(self,frame,frame_draw):
        
        if (config.debugging and config.debugging_TrafficLights):
            cv2.namedWindow("mask")
            cv2.namedWindow("mask_R")
            cv2.createTrackbar("Hue_L","mask",self.Hue_Low_G,255,self.OnHueLowChange)
            cv2.createTrackbar("Hue_H","mask",self.Hue_High_G,255,self.OnHueHighChange)
            cv2.createTrackbar("Lit_L","mask",self.Lit_Low_G,255,self.OnLitLowChange)
            cv2.createTrackbar("Sat_L","mask",self.Sat_Low_G,255,self.OnSatLowChange)
            cv2.createTrackbar("Hue_L_red","mask_R",self.Hue_Low_R,255,self.OnHueLowChange_R)
            cv2.createTrackbar("Hue_H_red","mask_R",self.Hue_High_R,255,self.OnHueHighChange_R)
            cv2.createTrackbar("Lit_L_red","mask_R",self.Lit_Low_R,255,self.OnLitLowChange_R)
            cv2.createTrackbar("Sat_L_red","mask_R",self.Sat_Low_R,255,self.OnSatLowChange_R)
        else:
            cv2.destroyWindow("mask")    
            cv2.destroyWindow("mask_R")

        # 0. To be accessed in Script Functions without explicitly passing as an Argument
        self.src = frame.copy()
        # 1. Converting frame to HLS ColorSpace
        self.HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc
        # 2. Extracting Mask of Only Red And Color Regions
        frame_ROI = self.MaskExtract()

        # 1. Cvt frame_ROI to grayscale
        gray = cv2.cvtColor(frame_ROI,cv2.COLOR_BGR2GRAY)
        # Localizing Potetial Candidates and Classifying them in SignDetection    
        self.Circledetector(gray.copy(),frame.copy(),frame_draw)

        return self.Traffic_State