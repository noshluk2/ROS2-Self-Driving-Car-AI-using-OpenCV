import cv2
import numpy as np
import time
from  ....config import config

from ..Morph_op import BwareaOpen,RetLargestContour_OuterLane,Ret_LowestEdgePoints

HLS=0
src=0
Hue_Low = 0
Lit_Low = 225
Sat_Low = 0#61

Hue_Low_Y = 30#30
Hue_High_Y = 33#40
Lit_Low_Y = 120#63
Sat_Low_Y = 0#81

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
    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(255       ,255,255))
    mask_Y = clr_segment(HLS,(Hue_Low_Y,Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,255,255))#Combine 6ms
    mask_Y_ = mask_Y != 0
    dst_Y = src * (mask_Y_[:,:,None].astype(src.dtype))
    mask_ = mask != 0

    dst = src * (mask_[:,:,None].astype(src.dtype))
    if (config.debugging_Lane and config.debugging and config.debugging_L_ColorSeg):
        cv2.imshow('[Segment_Colour_final] mask',dst)
        cv2.imshow('[Segment_Colour_final] mask_Y',dst_Y)


#cv2.namedWindow("HSL",cv2.WINDOW_NORMAL)
#cv2.namedWindow("frame_Lane",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Lane_gray",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Lane_gray_opened",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Lane_gray_Smoothed",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Lane_edge",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Lane_edge_ROI",cv2.WINDOW_NORMAL)
#cv2.namedWindow("Mid_ROI_mask",cv2.WINDOW_NORMAL)





def clr_segment(HSL,lower_range,upper_range):
    
    # 2. Performing Color Segmentation on Given Range
    lower = np.array( [lower_range[0],lower_range[1] ,lower_range[2]] )
    upper = np.array( [upper_range[0]    ,255     ,255])
    mask = cv2.inRange(HSL, lower, upper)
    
    # 3. Dilating Segmented ROI's
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3,3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
    return mask


def LaneROI(frame,mask,minArea):
    
    # 4a. Keeping only Midlane ROI of frame
    frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)#Extracting only RGB from a specific region
    # 4b. Converting frame to grayscale
    Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY) # Converting to grayscale
    # 4c. Keep Only larger objects
    Lane_gray_opened = BwareaOpen(Lane_gray,minArea) # Getting mask of only objects larger then minArea
    
    Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)# Getting the gray of that mask
    Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1) # Smoothing out the edges for edge extraction later

    # 4d. Keeping only Edges of Segmented ROI    
    Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3) # Extracting the Edge of Canny

    #cv2.imshow('ROI_mask',mask)
    #cv2.imshow('frame_Lane',frame_Lane)
    #cv2.imshow('Lane_gray',Lane_gray)
    #cv2.imshow('Lane_gray_opened',Lane_gray_opened)
    #cv2.imshow('Lane_gray_Smoothed',Lane_gray_Smoothed)
    #cv2.imshow('Lane_edge',Lane_edge)

    return Lane_edge,Lane_gray_opened

def OuterLaneROI(frame,mask,minArea):

    Outer_Points_list=[]

    # 5a. Extracted OuterLanes Mask And Edge
    frame_Lane = cv2.bitwise_and(frame,frame,mask=mask)#Extracting only RGB from a specific region
    Lane_gray = cv2.cvtColor(frame_Lane,cv2.COLOR_BGR2GRAY)# Converting to grayscale
    Lane_gray_opened = BwareaOpen(Lane_gray,minArea) # Getting mask of only objects larger then minArea
    Lane_gray = cv2.bitwise_and(Lane_gray,Lane_gray_opened)# Getting the gray of that mask
    Lane_gray_Smoothed = cv2.GaussianBlur(Lane_gray,(11,11),1)# Smoothing out the edges for edge extraction later
    Lane_edge = cv2.Canny(Lane_gray_Smoothed,50,150, None, 3) # Extracting the Edge of Canny

    # 5b. Kept Larger OuterLane
    ROI_mask_Largest,Largest_found = RetLargestContour_OuterLane(Lane_gray_opened,minArea) # Extracting the largest Yellow object in frame

    if(Largest_found):
        # 5c. Kept Larger OuterLane [Edge]
        Outer_edge_Largest = cv2.bitwise_and(Lane_edge,ROI_mask_Largest)
        # 5d. Returned Lowest Edge Points
        Lane_TwoEdges, Outer_Points_list = Ret_LowestEdgePoints(ROI_mask_Largest)
        Lane_edge = Outer_edge_Largest
    else:
        Lane_TwoEdges = np.zeros(Lane_gray.shape,Lane_gray.dtype)

    #cv2.imshow('frame_Lane',frame_Lane)
    #cv2.imshow('Lane_gray',Lane_gray)
    #cv2.imshow('Lane_gray_opened',Lane_gray_opened)
    #cv2.imshow('Lane_gray_Smoothed',Lane_gray_Smoothed)
    #cv2.imshow('Lane_edge_ROI',Lane_edge_ROI)

    #cv2.imshow('ROI_mask_Largest',ROI_mask_Largest)
    #cv2.imshow('Lane_edge',Lane_edge)
    #cv2.imshow('Lane_TwoEdges',Lane_TwoEdges)
    return Lane_edge,Lane_TwoEdges,Outer_Points_list

def Segment_Colour(frame,minArea):
    """ Segment Lane-Lines (both outer and middle) from the road lane

    Args:
        frame (numpy nd array): Prius front-cam view
        minArea (int): minimum area of an object required to be considered as a valid object

    Returns:
        numpy 2d array: Edges of white mid-lane
        numpy 2d array: Mask  of white  mid-lane
        numpy 2d array: Edges of yellow outer-lane
        numpy 2d array: Edges of outer-lane (Seperated to get inner side later)
                  List: Two points taken one each from outer-Lane edge seperated
    """
    
    global HLS,src

    src = frame.copy()
    
    # 1. Converting frame to HLS ColorSpace
    HLS = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)#2 msc

    mask   = clr_segment(HLS,(Hue_Low  ,Lit_Low   ,Sat_Low  ),(255       ,255,255))
    mask_Y = clr_segment(HLS,(Hue_Low_Y,Lit_Low_Y ,Sat_Low_Y),(Hue_High_Y,255,255))#Combine 6ms
    
    Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list = OuterLaneROI(frame,mask_Y,minArea+500)#27msec

    Mid_edge_ROI,Mid_ROI_mask = LaneROI(frame,mask,minArea)#20 msec

    #cv2.imshow('Mid_ROI_mask',Mid_ROI_mask)

    if (config.debugging_Lane and config.debugging and config.debugging_L_ColorSeg):
        # Debugging lane segmentation on colour only once......
        if not config.clr_seg_dbg_created:            
            config.clr_seg_dbg_created = True        
            cv2.namedWindow("[Segment_Colour_final] mask")
            cv2.namedWindow("[Segment_Colour_final] mask_Y")

            cv2.createTrackbar("Hue_L","[Segment_Colour_final] mask",Hue_Low,255,OnHueLowChange)
            cv2.createTrackbar("Lit_L","[Segment_Colour_final] mask",Lit_Low,255,OnLitLowChange)
            cv2.createTrackbar("Sat_L","[Segment_Colour_final] mask",Sat_Low,255,OnSatLowChange)

            cv2.createTrackbar("Hue_L","[Segment_Colour_final] mask_Y",Hue_Low_Y,255,OnHueLowChange_Y)
            cv2.createTrackbar("Hue_H","[Segment_Colour_final] mask_Y",Hue_High_Y,255,OnHueHighChange_Y)
            cv2.createTrackbar("Lit_L","[Segment_Colour_final] mask_Y",Lit_Low_Y,255,OnLitLowChange_Y)
            cv2.createTrackbar("Sat_L","[Segment_Colour_final] mask_Y",Sat_Low_Y,255,OnSatLowChange_Y)

            cv2.imshow('[Segment_Colour_final] mask',mask)
            cv2.imshow('[Segment_Colour_final] mask_Y',mask_Y)
        cv2.imshow('Mid_edge_ROI',Mid_edge_ROI)
        cv2.imshow('Outer_edge_ROI',Outer_edge_ROI)
        cv2.imshow('OuterLane_Side_Seperated',OuterLane_SidesSeperated)
    else:
        if config.clr_seg_dbg_created:
            cv2.destroyWindow('[Segment_Colour_final] mask')
            cv2.destroyWindow('[Segment_Colour_final] mask_Y')
        cv2.destroyWindow('Mid_edge_ROI')
        cv2.destroyWindow('Outer_edge_ROI')
        cv2.destroyWindow('OuterLane_Side_Seperated')

    return Mid_edge_ROI,Mid_ROI_mask,Outer_edge_ROI,OuterLane_SidesSeperated,Outer_Points_list