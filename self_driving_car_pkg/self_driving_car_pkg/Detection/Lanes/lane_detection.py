import cv2

from .a_colour_segmentation import segment_lanes
from .b_midlane_estimation import estimate_midlane
from ...config import config
from .c_cleaning import GetYellowInnerEdge,ExtendShortLane
from .d_data_extraction import FetchInfoAndDisplay


def detect_lanes(img):
    # cropping the roi (e.g keeping only below the horizon)
    img_cropped = img[config.CropHeight_resized:,:]

    mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points = segment_lanes(img_cropped,config.minArea_resized)

    estimated_midlane = estimate_midlane(mid_lane_edge,config.MaxDist_resized)

    # [Lane Detection] STAGE_3 (Cleaning) <<<<<<--->>>>>> [STEP_1]:
    OuterLane_OneSide,Outer_cnts_oneSide,Mid_cnts,Offset_correction = GetYellowInnerEdge(outerlane_side_sep,estimated_midlane,outerlane_points)#3ms
    # [Lane Detection] STAGE_3 (Cleaning) <<<<<<--->>>>>> [STEP_2]:
    extended_midlane,extended_outerlane = ExtendShortLane(estimated_midlane,Mid_cnts,Outer_cnts_oneSide,OuterLane_OneSide.copy())

    # [Lane Detection] STAGE_4 (Data_Extraction) <<<<<<--->>>>>> [Our Approach]:
    Distance , Curvature = FetchInfoAndDisplay(mid_lane_edge,extended_midlane,extended_outerlane,img_cropped,Offset_correction)

    #cv2.imshow("mid_lane_mask",mid_lane_mask)
    cv2.imshow("mid_lane_edge",mid_lane_edge)
    #cv2.imshow("outer_lane_edge",outer_lane_edge)
    #cv2.imshow("outerlane_side_sep",outerlane_side_sep)
    #cv2.imshow("estimated_midlane",estimated_midlane)

    #cv2.imshow("OuterLane_OneSide",OuterLane_OneSide)
    cv2.imshow("extended_midlane",extended_midlane)
    #cv2.imshow("extended_outerlane",extended_outerlane)
    
    cv2.waitKey(1)

    return Distance,Curvature
    


