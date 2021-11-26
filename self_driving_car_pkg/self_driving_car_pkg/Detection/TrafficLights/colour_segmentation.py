import cv2
import numpy as np

class colour_segment:

    def __init__(self):
        # Instance Variables
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

    cv2.namedWindow("[TL_Config] mask")
    cv2.namedWindow("[TL_Config] mask_R")
    cv2.createTrackbar("Hue_L","[TL_Config] mask",Hue_Low_G,255,OnHueLowChange)
    cv2.createTrackbar("Hue_H","[TL_Config] mask",Hue_High_G,255,OnHueHighChange)
    cv2.createTrackbar("Lit_L","[TL_Config] mask",Lit_Low_G,255,OnLitLowChange)
    cv2.createTrackbar("Sat_L","[TL_Config] mask",Sat_Low_G,255,OnSatLowChange)
    cv2.createTrackbar("Hue_L_red","[TL_Config] mask_R",Hue_Low_R,255,OnHueLowChange_R)
    cv2.createTrackbar("Hue_H_red","[TL_Config] mask_R",Hue_High_R,255,OnHueHighChange_R)
    cv2.createTrackbar("Lit_L_red","[TL_Config] mask_R",Lit_Low_R,255,OnLitLowChange_R)
    cv2.createTrackbar("Sat_L_red","[TL_Config] mask_R",Sat_Low_R,255,OnSatLowChange_R)

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

        cv2.imshow("[TL_Config] mask",dst)
        cv2.imshow("[TL_Config] mask_R",dst)


        return dst


    def isolate_yelo_red_regions(self,frame):

        # 0. To be accessed in Script Functions without explicitly passing as an Argument
        self.src = frame.copy()
        # 1. Converting frame to HLS ColorSpace
        self.HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc
        # 2. Extracting Mask of Only Red And Color Regions
        frame_ROI = self.MaskExtract()

        # 1. Cvt frame_ROI to grayscale
        gray_regions_oi = cv2.cvtColor(frame_ROI,cv2.COLOR_BGR2GRAY)

        return gray_regions_oi

