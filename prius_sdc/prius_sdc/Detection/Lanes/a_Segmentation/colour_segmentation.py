import cv2
import numpy as np
import time
import config

from Detection.Lanes.Morph_op import BwareaOpen,RetLargestContour_OuterLane,Ret_LowestEdgePoints

HLS = 0
src = 0

Hue_Low = 82
Lit_Low = 95
Sat_Low = 0

Hue_Low_Y = 30
Hue_High_Y = 49
Lit_Low_Y = 0
Sat_Low_Y = 51

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

def OnHueLowChange_Y(val):
    global Hue_Low_Y
    Hue_Low_Y = val
    MaskExtract()
def OnHueHighChange_Y(val):
    global Hue_High_Y
    Hue_High_Y = val
    MaskExtract()	
def OnLitLowChange_Y(val):
    global Lit_Low_Y
    Lit_Low_Y = val
    MaskExtract()
def OnSatLowChange_Y(val):
    global Sat_Low_Y
    Sat_Low_Y = val
    MaskExtract()

def MaskExtract():
    mask   = clr_segment(HLS, (Hue_Low  , Sat_Low   ,Lit_Low  ),(255       ,255,255))
    mask_Y = clr_segment(HLS, (Hue_Low_Y, Sat_Low_Y ,Lit_Low_Y),(Hue_High_Y,255,255))#Combine 6ms
    mask_ = mask != 0
    mask_Y_ = mask_Y != 0
    dst = src * (mask_[:,:,None].astype(src.dtype))
    dst_Y = src * (mask_Y_[:,:,None].astype(src.dtype))
    cv2.imshow("[Segment_Colour] mask", dst)
    cv2.imshow("[Segment_Colour] mask_Y", dst_Y)
    #cv2.imshow('mask',mask)
    #cv2.imshow('mask_Y',mask_Y)


if(config.clr_segmentation_tuning):

    cv2.namedWindow("[Segment_Colour] mask",cv2.WINDOW_NORMAL)
    cv2.namedWindow("[Segment_Colour] mask_Y",cv2.WINDOW_NORMAL)

    cv2.createTrackbar("Hue_L","[Segment_Colour] mask",Hue_Low,255,OnHueLowChange)
    cv2.createTrackbar("Lit_L","[Segment_Colour] mask",Lit_Low,255,OnLitLowChange)
    cv2.createTrackbar("Sat_L","[Segment_Colour] mask",Sat_Low,255,OnSatLowChange)

    cv2.createTrackbar("Hue_L","[Segment_Colour] mask_Y",Hue_Low_Y,255,OnHueLowChange_Y)
    cv2.createTrackbar("Hue_H","[Segment_Colour] mask_Y",Hue_High_Y,255,OnHueHighChange_Y)
    cv2.createTrackbar("Lit_L","[Segment_Colour] mask_Y",Lit_Low_Y,255,OnLitLowChange_Y)
    cv2.createTrackbar("Sat_L","[Segment_Colour] mask_Y",Sat_Low_Y,255,OnSatLowChange_Y)

def clr_segment(HSL,lower_range,upper_range):
    lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
    upper = np.array( [upper_range[0]    ,255     ,255])
    mask = cv2.inRange(HSL, lower, upper)
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask


def Segment_Colour(frame,minArea):

    global HLS,src
    src = frame

    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc

    mask   = clr_segment(HLS, (Hue_Low  , Lit_Low   ,Sat_Low  ),(255       ,255,255))
    mask_Y = clr_segment(HLS, (Hue_Low_Y, Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,255,255))#Combine 6ms

    #cv2.imshow('HSL',HSL)
    #cv2.imshow('Mid_ROI_mask',Mid_ROI_mask)

    if(config.clr_segmentation_tuning):
        cv2.imshow('[Segment_Colour] mask',mask)
        cv2.imshow('[Segment_Colour] mask_Y',mask_Y)

    #return Mid_edge_ROI,Mid_ROI_mask,Outer_edge_ROI,None,OuterLane_OneSide,Outer_Points_list