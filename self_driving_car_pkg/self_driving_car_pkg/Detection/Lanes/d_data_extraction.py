import cv2
import numpy as np

from .utilities import Cord_Sort,findlaneCurvature

def LanePoints(midlane,outerlane,offset):
    mid_cnts = cv2.findContours(midlane,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[1]
    outer_cnts = cv2.findContours(outerlane,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[1]

    if mid_cnts and outer_cnts:
        mid_cnts_row_sorted = Cord_Sort(mid_cnts,"rows")
        outer_cnts_row_sorted = Cord_Sort(outer_cnts,"rows")

        m_rows = mid_cnts_row_sorted.shape[0]
        o_rows = outer_cnts_row_sorted.shape[0]

        m_rows_btm_pt = mid_cnts_row_sorted[m_rows-1,:]
        o_rows_btm_pt = outer_cnts_row_sorted[o_rows-1,:]
        m_rows_top_pt = mid_cnts_row_sorted[0,:]
        o_rows_top_pt = outer_cnts_row_sorted[0,:]

        traj_btm_pt = ( int((m_rows_btm_pt[0] + o_rows_btm_pt[0])/2)+ offset ,int((m_rows_btm_pt[1] + o_rows_btm_pt[1])/2))
        traj_top_pt = ( int((m_rows_top_pt[0] + o_rows_top_pt[0])/2)+ offset ,int((m_rows_top_pt[1] + o_rows_top_pt[1])/2))

        return traj_btm_pt,traj_top_pt

    else:
        return (0,0),(0,0)

def EstimateNonMidMask(MidEdgeROi):
    Mid_Hull_Mask = np.zeros((MidEdgeROi.shape[0], MidEdgeROi.shape[1], 1), dtype=np.uint8)
    contours = cv2.findContours(MidEdgeROi,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[1]
    if contours:
        hull_list = []
        contours = np.concatenate(contours)
        hull = cv2.convexHull(contours)
        hull_list.append(hull)
        Mid_Hull_Mask = cv2.drawContours(Mid_Hull_Mask, hull_list, 0, 255,-1)
    Non_Mid_Mask=cv2.bitwise_not(Mid_Hull_Mask)
    return Non_Mid_Mask


def FetchInfoAndDisplay(Mid_lane_edge,Mid_lane,Outer_Lane,frame,Offset_correction):
    # 1. Using Both outer and middle information to create probable path
    Traj_lowP,Traj_upP = LanePoints(Mid_lane,Outer_Lane,Offset_correction)
    
    # 2. Compute Distance and Curvature from Trajectory Points 
    PerpDist_LaneCentralStart_CarNose= -1000
    if(Traj_lowP!=(0,0)):
        PerpDist_LaneCentralStart_CarNose = Traj_lowP[0] - int(Mid_lane.shape[1]/2)
    curvature = findlaneCurvature(Traj_lowP[0],Traj_lowP[1],Traj_upP[0],Traj_upP[1])

    # 3. Keep only those edge that are part of MIDLANE
    Mid_lane_edge = cv2.bitwise_and(Mid_lane_edge,Mid_lane)

    # 4. Combine Mid and OuterLane to get Lanes Combined and extract its contours
    Lanes_combined = cv2.bitwise_or(Outer_Lane,Mid_lane)
    cv2.imshow("Lanes_combined",Lanes_combined)
    ProjectedLane = np.zeros(Lanes_combined.shape,Lanes_combined.dtype)
    cnts = cv2.findContours(Lanes_combined,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[1]

    # 5. Fill ProjectedLane with fillConvexPoly
    if cnts:
        cnts = np.concatenate(cnts)
        cnts = np.array(cnts)
        cv2.fillConvexPoly(ProjectedLane, cnts, 255)

    # 6. Remove Midlane_Region from ProjectedLane by extracting the midless mask
    Mid_less_Mask = EstimateNonMidMask(Mid_lane_edge)
    ProjectedLane = cv2.bitwise_and(Mid_less_Mask,ProjectedLane)

    # 7. Draw projected lane
    Lane_drawn_frame = frame
    Lane_drawn_frame[ProjectedLane==255] = Lane_drawn_frame[ProjectedLane==255] + (0,100,0)
    Lane_drawn_frame[Outer_Lane==255] = Lane_drawn_frame[Outer_Lane==255] + (0,0,100)# Outer Lane Coloured Red
    Lane_drawn_frame[Mid_lane==255] = Lane_drawn_frame[Mid_lane==255] + (100,0,0)# Mid Lane Coloured Blue
    Out_image = Lane_drawn_frame

    # 8. Draw Cars direction and Lanes direction and distance between car and lane path
    cv2.line(Out_image,(int(Out_image.shape[1]/2),Out_image.shape[0]),(int(Out_image.shape[1]/2),Out_image.shape[0]-int (Out_image.shape[0]/5)),(0,0,255),2)
    cv2.line(Out_image,Traj_lowP,Traj_upP,(255,0,0),2)
    if(Traj_lowP!=(0,0)):
        cv2.line(Out_image,Traj_lowP,(int(Out_image.shape[1]/2),Traj_lowP[1]),(255,255,0),2)# distance of car center with lane path

    # 9. Draw extracted distance and curvature 
    curvature_str="Curvature = " + f"{curvature:.2f}"
    PerpDist_ImgCen_CarNose_str="Distance = " + str(PerpDist_LaneCentralStart_CarNose)
    textSize_ratio = 0.5
    cv2.putText(Out_image,curvature_str,(10,30),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)
    cv2.putText(Out_image,PerpDist_ImgCen_CarNose_str,(10,50),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)
    return PerpDist_LaneCentralStart_CarNose,curvature



    




