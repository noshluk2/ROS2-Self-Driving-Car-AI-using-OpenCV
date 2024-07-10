import cv2
import numpy as np

from ...config import config
import os
import math

class Segment_On_Clr:

    def __init__(self, a_1 = 56,a_2 = 66,a_3 = 41,a_4 = 23, b_1 = 0,b_2 = 8,b_3 = 33,b_4 = 23):
        
        self.HLS = 0
        self.src = 0

        self.Hue_Low_G  = a_1
        self.Hue_High_G = a_2
        self.Lit_Low_G  = a_3
        self.Sat_Low_G  = a_4

        self.Hue_Low_R  = b_1
        self.Hue_High_R = b_2
        self.Lit_Low_R  = b_3
        self.Sat_Low_R  = b_4

    def clr_segment(self,lower_range,upper_range):
        
        # 2. Performing Color Segmentation on Given Range
        lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
        upper = np.array( [upper_range[0]    ,255     ,255])
        mask = cv2.inRange(self.HLS, lower, upper)
        # 3. Dilating Segmented ROI's
        kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
        return mask

    def MaskExtract(self):
        mask_Green   = self.clr_segment( (self.Hue_Low_G  ,self.Lit_Low_G   ,self.Sat_Low_G ), (self.Hue_High_G       ,255,255) )
        mask_Red     = self.clr_segment( (self.Hue_Low_R  ,self.Lit_Low_R   ,self.Sat_Low_R ), (self.Hue_High_R       ,255,255) )

        Mask = cv2.bitwise_or(mask_Green,mask_Red)
        MASK_Binary = Mask != 0

        dst = self.src * (MASK_Binary[:,:,None].astype(self.src.dtype))

        if (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
            cv2.imshow("[TL_Config] mask2",dst)
            cv2.imshow("[TL_Config] mask_R2",dst)


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

    def in_hls(self,frame,mask=None,Rmv_Clr_From_Frame = False):
        
        Seg_ClrReg_rmvd = None

        Frame_Mask = np.ones((frame.shape[0],frame.shape[1]),np.uint8)*255

        if Rmv_Clr_From_Frame:
            ROI_detected = frame
        else:
            ROI_detected = cv2.bitwise_and(frame,frame,mask = mask )
        
        #cv2.imshow("ROI_detected",ROI_detected)
        
        if (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
            cv2.createTrackbar("Hue_L","[TL_Config] mask2",self.Hue_Low_G,255,self.OnHueLowChange)
            cv2.createTrackbar("Hue_H","[TL_Config] mask2",self.Hue_High_G,255,self.OnHueHighChange)
            cv2.createTrackbar("Lit_L","[TL_Config] mask2",self.Lit_Low_G,255,self.OnLitLowChange)
            cv2.createTrackbar("Sat_L","[TL_Config] mask2",self.Sat_Low_G,255,self.OnSatLowChange)
            cv2.createTrackbar("Hue_L_red","[TL_Config] mask_R2",self.Hue_Low_R,255,self.OnHueLowChange_R)
            cv2.createTrackbar("Hue_H_red","[TL_Config] mask_R2",self.Hue_High_R,255,self.OnHueHighChange_R)
            cv2.createTrackbar("Lit_L_red","[TL_Config] mask_R2",self.Lit_Low_R,255,self.OnLitLowChange_R)
            cv2.createTrackbar("Sat_L_red","[TL_Config] mask_R2",self.Sat_Low_R,255,self.OnSatLowChange_R)



        # 0. To be accessed in Script Functions without explicitly passing as an Argument
        self.src = ROI_detected.copy()
        # 1. Converting frame to HLS ColorSpace
        self.HLS = cv2.cvtColor(ROI_detected,cv2.COLOR_BGR2HLS)#2 msc
        # 2. Extracting Mask of Only Red And Color Regions
        Seg_ClrReg = self.MaskExtract()

        if mask is not None:
            frame_ROI_Gray = cv2.cvtColor(Seg_ClrReg,cv2.COLOR_BGR2GRAY)
            frame_ROI_Bin = cv2.threshold(frame_ROI_Gray,0,255,cv2.THRESH_BINARY)[1]
        if Rmv_Clr_From_Frame:
            Seg_ClrReg_rmvd = cv2.bitwise_xor(Frame_Mask,frame_ROI_Bin)
            Seg_ClrReg_rmvd = cv2.bitwise_and(frame,frame,mask=Seg_ClrReg_rmvd)
        else:    
            Seg_ClrReg_rmvd= cv2.bitwise_xor(mask,frame_ROI_Bin)
            #cv2.imshow("Seg_ClrReg",Seg_ClrReg)
            #cv2.imshow("Seg_ClrReg_rmvd",Seg_ClrReg_rmvd)
            #cv2.waitKey(0)

        return Seg_ClrReg,Seg_ClrReg_rmvd

class TL_States:

    def __init__(self):
        # Instance Variables
        self.detected_circle = 0 
        self.Traffic_State = "Unknown"
        self.prevTraffic_State = 0

        self.write_data = False
        self.draw_detected = True
        self.display_images = True

        self.HLS = 0
        self.src = 0

    # Class Variables
    Hue_Low_G  = 56
    Hue_High_G = 66
    Lit_Low_G  = 41
    Sat_Low_G  = 23

    Hue_Low_R  = 0
    Hue_High_R = 8
    Lit_Low_R  = 33
    Sat_Low_R  = 23

    @staticmethod
    def dist(a,b):
        return int( math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) ) )

    @staticmethod
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
                            #print("Distance between [ center = ", center, "center_cmp = ",center_cmp, " ] is  = ",point_Dist)
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
                                    if (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
                                        cv2.imshow('Traffic Light Confirmed!! [Checking State!!!]',frame_draw_special)

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

            if (config.debugging and config.debugging_TrafficLights):            
                detected_circles_str= "#_of_detected_circles = "+ str(circles.shape[1])
                cv2.putText(frame_draw,detected_circles_str,(20,100),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255))
            
            if self.display_images:
                
                if (config.debugging and config.debugging_TrafficLights):
                    Traffic_State_STR= "Traffic State = "+ self.Traffic_State
                    cv2.putText(frame_draw, Traffic_State_STR, (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255))
                    cimg_str = '[Fetch_TL_State] (2) detected circular reg'
                    cv2.imshow(cimg_str,frame_draw)
  

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

        if (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
            cv2.imshow("[TL_Config] mask",dst)
            cv2.imshow("[TL_Config] mask_R",dst)


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
        
        if (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
            cv2.namedWindow("[TL_Config] mask")
            cv2.namedWindow("[TL_Config] mask_R")
            cv2.createTrackbar("Hue_L","[TL_Config] mask",self.Hue_Low_G,255,self.OnHueLowChange)
            cv2.createTrackbar("Hue_H","[TL_Config] mask",self.Hue_High_G,255,self.OnHueHighChange)
            cv2.createTrackbar("Lit_L","[TL_Config] mask",self.Lit_Low_G,255,self.OnLitLowChange)
            cv2.createTrackbar("Sat_L","[TL_Config] mask",self.Sat_Low_G,255,self.OnSatLowChange)
            cv2.createTrackbar("Hue_L_red","[TL_Config] mask_R",self.Hue_Low_R,255,self.OnHueLowChange_R)
            cv2.createTrackbar("Hue_H_red","[TL_Config] mask_R",self.Hue_High_R,255,self.OnHueHighChange_R)
            cv2.createTrackbar("Lit_L_red","[TL_Config] mask_R",self.Lit_Low_R,255,self.OnLitLowChange_R)
            cv2.createTrackbar("Sat_L_red","[TL_Config] mask_R",self.Sat_Low_R,255,self.OnSatLowChange_R)



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

TL_States_ = TL_States()

class Cascade_Detector:

    def __init__(self):
        # Instance Variables
        print("Initialized Object of Cascade_Detector class")

    # Class Variables
    TrafficLight_cascade_str = os.path.join(os.getcwd(), "self_driving_car_pkg/self_driving_car_pkg/data/TrafficLight_cascade.xml")
    TrafficLight_cascade = cv2.CascadeClassifier()
    #-- 1. Load the cascades
    if not TrafficLight_cascade.load(cv2.samples.findFile(TrafficLight_cascade_str)):
        print('--(!)Error loading face cascade')
        exit(0)
    

    def detect(self,img):
        """ Uses haar cascade (object detector) to detect traffic light and return its bbox and state

        Args:
            img (numpy nd array): Prius front-cam view

        Returns:
            (rect): Detected Traffic Light bounding box
            (String): State of Traffic Light (Red | Green | Unknown)
        """
        img_draw=img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        target = self.TrafficLight_cascade.detectMultiScale(gray)
        TrafficLightFound=False
        Traffic_State = "Unknown"

        TL_iteration = 0
        for (x,y,w,h) in target:
            cv2.rectangle(img_draw, (x,y), (x+w,y+h), (0,165,255), 2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            TL_Maybe_mask = np.zeros(gray.shape,np.uint8)
            TL_Maybe_mask[y:y+h,x:x+w] = 255
            img_ROI = cv2.bitwise_and(img,img,mask=TL_Maybe_mask)

            if (config.debugging and config.debugging_TrafficLights):
                cv2.imshow('[Fetch_TL_State] (1) img_ROI', img_ROI)

            # Reconfirm if detected Traffic Light was the desired one
            Traffic_State = TL_States_.Get_TL_State(img_ROI,img_draw)
            if(Traffic_State!="Unknown"):
                print("Traffic State Recived at",TL_iteration," pos = ",Traffic_State)
                # Confirm Traffic Light 
                cv2.rectangle(img_draw, (x,y), (x+w,y+h), (0,255,0), 2)
                # Start Tracking
                TrafficLightFound = True
                if (config.debugging and config.debugging_TrafficLights):
                    cv2.imshow('[Fetch_TL_State] (3) Traffic Light With State', img_draw)

                # cv2.waitKey(0)
                break
            TL_iteration +=1

        #cv2.imshow('detected_TrafficLight', img_draw)
        #cv2.waitKey(1)

        
        if TrafficLightFound:
            TrafficLight_Rect = target[TL_iteration]
        else:
            TrafficLight_Rect = np.array([0,0,0,0])

        return TrafficLight_Rect,Traffic_State

class TL_Tracker:

    def __init__(self):
        # Instance Variables
        print("Initialized Object of signTracking class")

    # Class Variables
    mode = "Detection"
    max_allowed_dist = 100
    feature_params = dict(maxCorners=100,qualityLevel=0.3,minDistance=7,blockSize=7)
    lk_params = dict(winSize=(15, 15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10, 0.03))  
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    known_centers = []
    known_centers_confidence = []
    old_gray = 0
    p0 = []
    Tracked_class = 0
    mask = 0
    Tracked_ROI=0
    CollisionIminent = False

    def Distance(self,a,b):
        #return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )
        return math.sqrt( ( (float(a[1])-float(b[1]))**2 ) + ( (float(a[0])-float(b[0]))**2 ) )

    def MatchCurrCenter_ToKnown(self,center):
        match_found = False
        match_idx = 0
        for i in range(len(self.known_centers)):
            if ( self.Distance(center,self.known_centers[i]) < self.max_allowed_dist ):
                match_found = True
                match_idx = i
                return match_found, match_idx
        # If no match found as of yet return default values
        return match_found, match_idx
    def santitze_pts(self,pts_src,pts_dst):
        # Idea was to Order on Descending Order of Strongest Points [Strength here is 
        # considered when two points have minimum distance between each other]
        pt_idx = 0
        dist_list = []
        for pt in pts_src:
            pt_b = pts_dst[pt_idx]
            dist_list.append(self.Distance(pt,pt_b))
            pt_idx+=1

        pts_src_list = pts_src.tolist()
        pts_dst_list = pts_dst.tolist()

        pts_src_list = [x for _, x in sorted(zip(dist_list, pts_src_list))]
        pts_dst_list = [x for _, x in sorted(zip(dist_list, pts_dst_list))]

        pts_src = np.asarray(pts_src_list, dtype=np.float32)
        pts_dst = np.asarray(pts_dst_list, dtype=np.float32)

        return pts_src,pts_dst

    
    def EstimateTrackedRect(self,im_src,pts_src,pts_dst,img_draw):
        Tracking = "Tracking"
        im_dst = np.zeros_like(im_src)

        if(len(pts_src)>=3):
            pts_src,pts_dst = self.santitze_pts(pts_src,pts_dst)
            pts_src = pts_src[0:3][:]
            pts_dst = pts_dst[0:3][:]

            M = cv2.getAffineTransform(pts_src, pts_dst)
            im_dst = cv2.warpAffine(im_src, M ,(im_dst.shape[1],im_dst.shape[0]),flags=cv2.INTER_CUBIC)
            
            img_dst_2 = np.zeros_like(im_dst)

            kernel = np.ones((2,2), dtype=np.uint8)
            closing = cv2.morphologyEx(im_dst, cv2.MORPH_CLOSE, kernel)
            
            cnts = cv2.findContours(closing, cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_NONE )[1]
            cnt = max(cnts, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(cnt)

            # [NEW]: Identifying (Prius < = Proximity = > Traffic Light) 
            #        [ based on its location on left or right extrema  of image. ]
            if ( (              (x+w)             < (0.5*im_src.shape[1]) ) or
                 ( abs( (x+w) - im_src.shape[1] ) < (0.3*im_src.shape[1]) )    ):
                self.CollisionIminent = True
                
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            cv2.drawContours(img_dst_2,[box],0,255,-1)

            # Drawing Tracked Traffic Light Rect On Original Image
            if (config.debugging and config.debugging_TrafficLights):
                cv2.drawContours(img_draw,[box],0,(255,0,0),2)

            #https://stackoverflow.com/questions/39371507/image-loses-quality-with-cv2-warpperspective
            # Smoothing by warping is caused by interpolation
            #im_dst = cv2.warpAffine(im_src, M ,(im_dst.shape[1],im_dst.shape[0]))

        else:
            print("Points less then 3, Error!!!")
            #cv2.waitKey(0)
            Tracking = "Detection"
            # Set Img_dst_2 to Already saved Tracked Roi One last Time
            img_dst_2 = self.Tracked_ROI
            self.CollisionIminent = False # Reset
        
        return im_dst,img_dst_2,Tracking

    def Track(self,frame,frame_draw):

        Temp_Tracked_ROI = TL_Track.Tracked_ROI
        # 4a. Convert Rgb to gray
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        if (config.debugging and config.debugging_TrafficLights):
            Text2display = "OpticalFlow ( " + self.mode + " )"
            cv2.putText(frame_draw,Text2display,(20,150),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1)

        # Localizing Potetial Candidates and Classifying them in SignDetection
        # 4b. Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, self.p0, None,**self.lk_params)
        
        # 4c. If no flow, look for new points
        if p1 is None:
            self.mode = "Detection"
            self.mask = np.zeros_like(frame_draw)
            self.Reset()

        # 4d. If points tracked, Display and Update SignTrack class    
        else:
            # Select good points
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
            self.Tracked_ROI,Temp_Tracked_ROI,self.mode = self.EstimateTrackedRect(self.Tracked_ROI,good_old,good_new,frame_draw)
            # Draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = (int(x) for x in new.ravel())
                c, d = (int(x) for x in old.ravel())
                self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                frame_draw = cv2.circle(frame_draw, (a, b), 5, self.color[i].tolist(), -1)
            frame_draw_ = frame_draw + self.mask# Display the image with the flow lines
            np.copyto(frame_draw,frame_draw_)#important to copy the data to same address as frame_draw   
            self.old_gray = gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)
        #cv2.imshow("frame_draw",frame_draw)
        return Temp_Tracked_ROI

    def Reset(self):
        
        self.known_centers = []
        self.known_centers_confidence = []
        self.old_gray = 0
        self.p0 = []


cascade_detector = Cascade_Detector()
TL_Track = TL_Tracker()
Segment_On_Clr_ = Segment_On_Clr()

def detect_TrafficLights(img,frame_draw):
    """ Detect Traffic light (If-Present) and retrieve its state

    Args:
        img (numpy nd array): Prius front-cam view
        frame_draw (numpy nd array): for displaying detected traffic light

    Returns:
        (String): State of the Traffic Light (Red | Green | Unknown) [Unknown: No Traffic Light found!]
        (bool): SDC <== Close enough? ==> Traffic Light
    """    
    Curr_TL_State = "Unknown"
    # 4. Checking if SignTrack Class mode is Tracking If yes Proceed
    if(TL_Track.mode == "Tracking"):

        #_,ClrRegRmvd = Segment_On_Clr_.in_hls(img, mask=TL_Track.Tracked_ROI, Rmv_Clr_From_Frame = True )
        #cv2.imshow("[Tracking] ClrRegRmvd",ClrRegRmvd)
        #cv2.waitKey(0)
        Temp_Tracked_ROI = TL_Track.Track(img,frame_draw)
        #Temp_Tracked_ROI = TL_Track.Track(ClrRegRmvd,frame_draw)
        
        if (config.debugging and config.debugging_TrafficLights):
            cv2.imshow("[Fetch_TL_State] (4) Tracked_ROI",TL_Track.Tracked_ROI)

        img_ROI_tracked = cv2.bitwise_and(img,img,mask=Temp_Tracked_ROI)
        
        if (config.debugging and config.debugging_TrafficLights):        
            cv2.imshow('[Fetch_TL_State] (5) img_ROI_tracked_BoundedRect', img_ROI_tracked)

        # Reconfirm if detected Traffic Light was the desired one
        Curr_TL_State = TL_States_.Get_TL_State(img_ROI_tracked,frame_draw)

        if(Curr_TL_State!="Unknown"):
            print("Traffic State Recived While Tracking ",Curr_TL_State)

            if (config.debugging and config.debugging_TrafficLights):
                Collision_State= "Collision_State = "+ str(TL_Track.CollisionIminent)
                cv2.putText(frame_draw,Collision_State,(20,135),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255))

            if (config.debugging and config.debugging_TrafficLights):
                cv2.imshow('[Fetch_TL_State] (3) Traffic Light With State', frame_draw)

        #Curr_TL_State = cascade_detector.Get_TrafficLightState(img_ROI_tracked)

    # 3. If SignTrack is in Detection Proceed to intialize tracker
    elif (TL_Track.mode == "Detection"):

        # 3a. Select the ROI which u want to track
        r, TLD_Class = cascade_detector.detect(img)
        
        if ((r!=np.array([0,0,0,0])).all()):
            # Traffic Light Detected ===> Initialize Tracker 
            # 3b. Convert Rgb to gray
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # 3c. creating ROI mask
            ROI_toTrack = np.zeros_like(gray)
            ROI_toTrack[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = 255
            
            #ROI_mask = np.zeros_like(gray)
            #cv2.rectangle(ROI_mask, (int(r[0]),int(r[1])), (int(r[0]+r[2]),int(r[1]+r[3])),255, 2)

            #_,Mask_ClrRmvd = Segment_On_Clr_.in_hls(img,mask=ROI_toTrack)

            TL_Track.Tracked_ROI = ROI_toTrack
            # 3d. Updating signtrack class with variables initialized
            TL_Track.mode = "Tracking" # Set mode to tracking
            TL_Track.Tracked_class = TLD_Class # keep tracking frame sign name
            #if Mask_ClrRmvd is None:
            TL_Track.p0 = cv2.goodFeaturesToTrack(gray, mask = ROI_toTrack, **TL_Track.feature_params)
            #else:
            #    TL_Track.p0 = cv2.goodFeaturesToTrack(gray, mask = Mask_ClrRmvd, **TL_Track.feature_params)
            TL_Track.old_gray = gray.copy()
            TL_Track.mask = np.zeros_like(frame_draw)
            TL_Track.CollisionIminent = False

    if not (config.debugging and config.debugging_TrafficLights):
        cv2.destroyWindow('[Fetch_TL_State] (1) img_ROI')
        cv2.destroyWindow('[Fetch_TL_State] (2) detected circular reg')  
        cv2.destroyWindow('[Fetch_TL_State] (3) Traffic Light With State')
        cv2.destroyWindow("[Fetch_TL_State] (4) Tracked_ROI")
        cv2.destroyWindow('[Fetch_TL_State] (5) img_ROI_tracked_BoundedRect')
    
    if not (config.debugging and config.debugging_TrafficLights and config.debugging_TL_Config):
        cv2.destroyWindow('Traffic Light Confirmed!! [Checking State!!!]')
        cv2.destroyWindow("[TL_Config] mask")    
        cv2.destroyWindow("[TL_Config] mask_R")    


    return Curr_TL_State,TL_Track.CollisionIminent


