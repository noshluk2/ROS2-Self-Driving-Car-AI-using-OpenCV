import cv2
import math
import numpy as np


def Distance_(a,b):
    return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )

def ApproxDistBWCntrs(cnt,cnt_cmp):
    # compute the center of the contour
    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # compute the center of the contour
    M_cmp = cv2.moments(cnt_cmp)
    cX_cmp = int(M_cmp["m10"] / M_cmp["m00"])
    cY_cmp = int(M_cmp["m01"] / M_cmp["m00"])
    minDist=Distance_((cX,cY),(cX_cmp,cY_cmp))
    Centroid_a=(cX,cY)
    Centroid_b=(cX_cmp,cY_cmp)
    return minDist,Centroid_a,Centroid_b

def RetLargestContour(gray):
    LargestContour_Found = False
    thresh=np.zeros(gray.shape,dtype=gray.dtype)
    _,bin_img = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
    #Find the two Contours for which you want to find the min distance between them.
    cnts = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    Max_Cntr_area = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
            LargestContour_Found = True
    if (Max_Cntr_idx!=-1):
        thresh = cv2.drawContours(thresh, cnts, Max_Cntr_idx, (255,255,255), -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    return thresh, LargestContour_Found




def estimate_midlane(midlane_patches,Max_dist):

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
    midlane_patches = cv2.morphologyEx(midlane_patches,cv2.MORPH_DILATE,kernel)

    # 1. keep a Midlane_draw for displaying shortest connectivity later on
    midlane_connectivity_bgr = cv2.cvtColor(midlane_patches,cv2.COLOR_GRAY2BGR)

    # 2. Extract the Contours that define each object
    cnts = cv2.findContours(midlane_patches,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[1]

    # 3. Keep Only those contours that are not lines 
    min_area = 1
    legit_cnts = []
    for _,cnt in enumerate(cnts):
        cnt_area = cv2.contourArea(cnt)
        if (cnt_area>min_area):
            legit_cnts.append(cnt)
    cnts = legit_cnts

    # 4. Connect each contous with its closest 
    #                       & 
    #    disconnecting any that may be farther then x distance

    CntIdx_BstMatch = []# [BstMatchwithCnt0,BstMatchwithCnt1,....]
    for index, cnt in enumerate(cnts):
        prevmin_dist = 100000 ; Bstindex_cmp = 0 ; BstCentroid_a=0  ; BstCentroid_b=0      
        for index_cmp in range(len(cnts)-index):
            index_cmp = index_cmp + index
            cnt_cmp = cnts[index_cmp]

            if (index!= index_cmp):
                min_dist, cent_a , cent_b = ApproxDistBWCntrs(cnt,cnt_cmp)
                if (min_dist<prevmin_dist):
                    if (len(CntIdx_BstMatch)==0):
                        prevmin_dist = min_dist
                        Bstindex_cmp = index_cmp
                        BstCentroid_a = cent_a
                        BstCentroid_b = cent_b
                    else:
                        already_present= False
                        for i in range(len(CntIdx_BstMatch)):
                            if ( (index_cmp == i) and (index == CntIdx_BstMatch[i]) ):
                                already_present = True
                        if not already_present:
                            prevmin_dist = min_dist
                            Bstindex_cmp = index_cmp
                            BstCentroid_a = cent_a
                            BstCentroid_b = cent_b

        if ((prevmin_dist!= 100_000) and (prevmin_dist>Max_dist)):
            #print("prev_mindist > Max Allowed Dist !!!")
            break
        if (type(BstCentroid_a)!= int):
            CntIdx_BstMatch.append(Bstindex_cmp)
            cv2.line(midlane_connectivity_bgr,BstCentroid_a,BstCentroid_b,(0,255,0),2)

    midlane_connectivity = cv2.cvtColor(midlane_connectivity_bgr,cv2.COLOR_BGR2GRAY)                                   


    # 5. Get estimated midlane by returning the largest contour

    estimated_midlane, largest_found = RetLargestContour(midlane_connectivity)

    # 6. Return Estimated Midlane if found otherwise send original

    if largest_found:
        return estimated_midlane
    else:
        return midlane_patches