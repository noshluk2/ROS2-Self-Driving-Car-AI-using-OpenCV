import cv2
import numpy as np

from .utilities import Distance_,Cord_Sort

def IsPathCrossingMid(Midlane,Mid_cnts,Outer_cnts):

    is_Ref_to_path_Left = 0
    Ref_To_CarPath_Image = np.zeros_like(Midlane)
    
    Midlane_copy = Midlane.copy()

    if not Mid_cnts:
        print("[Warning!!!] NO Midlane detected")
    
    Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
    Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")
    Mid_Rows = Mid_cnts_Rowsorted.shape[0]
    Outer_Rows = Outer_cnts_Rowsorted.shape[0]

    Mid_bottom_Pt = Mid_cnts_Rowsorted[Mid_Rows-1,:]
    Outer_bottom_Pt = Outer_cnts_Rowsorted[Outer_Rows-1,:]

    CarTraj_bottom_Pt = ( int( (Mid_bottom_Pt[0] + Outer_bottom_Pt[0]  ) / 2 ) , int( (Mid_bottom_Pt[1]  + Outer_bottom_Pt[1] ) / 2 ) )
    

    cv2.line(Ref_To_CarPath_Image,CarTraj_bottom_Pt,(int(Ref_To_CarPath_Image.shape[1]/2),Ref_To_CarPath_Image.shape[0]),(255,255,0),2)# line from carstart to car path
    cv2.line(Midlane_copy,tuple(Mid_bottom_Pt),(Mid_bottom_Pt[0],Midlane_copy.shape[0]-1),(255,255,0),2)# connecting midlane to bottom
    

    is_Ref_to_path_Left = ( (int(Ref_To_CarPath_Image.shape[1]/2) - CarTraj_bottom_Pt[0]) > 0 )

    if( np.any( (cv2.bitwise_and(Ref_To_CarPath_Image,Midlane_copy) > 0) ) ):
        # Midlane and CarPath Intersets (MidCrossing)
        return True,is_Ref_to_path_Left
    else:
        return False,is_Ref_to_path_Left


def GetYellowInnerEdge(OuterLanes,MidLane,OuterLane_Points):
    Offset_correction = 0
    Outer_Lanes_ret = np.zeros_like(OuterLanes)

    # 1. Extracting Mid and OuterLane Contours
    Mid_cnts = cv2.findContours(MidLane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # 2. Checking if OuterLane was Present initially or not
    if not Outer_cnts:
        NoOuterLane_before=True
    else:
        NoOuterLane_before=False

    # 3. Setting the first contour of Midlane as Refrence
    Ref = (0,0) #If MidContours are present use the first ContourPoint as Ref To Find Nearest YellowLaneContour
    if(Mid_cnts):
        Ref = tuple(Mid_cnts[0][0][0])

    
    # 4. >>>>>>>>>>>>>> Condition 1 : if Both Midlane and Outlane is detected <<<<<<<<<<<<<
    if Mid_cnts:
        # 4.A                    ******[len(OuterLane_Points)==2)] *******
        # (a) Fetching side of outelane nearest to midlane
        if  (len(OuterLane_Points)==2):
            Point_a = OuterLane_Points[0]
            Point_b = OuterLane_Points[1]
            
            Closest_Index = 0
            if(Distance_(Point_a,Ref) <= Distance_(Point_b,Ref)):
                Closest_Index=0
            elif(len(Outer_cnts)>1):
                Closest_Index=1
            Outer_Lanes_ret = cv2.drawContours(Outer_Lanes_ret, Outer_cnts, Closest_Index, 255, 1)
            Outer_cnts_ret = [Outer_cnts[Closest_Index]]

        # (b) If Correct outlane was detected =====================================
            IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts_ret)
            if(IsPathCrossing):
                OuterLanes = np.zeros_like(OuterLanes)
            else:
                return Outer_Lanes_ret ,Outer_cnts_ret, Mid_cnts,0


        # 4.B                    ******[len(OuterLane_Points)!=2)] ********
        elif( np.any(OuterLanes>0) ):
            IsPathCrossing , IsCrossingLeft = IsPathCrossingMid(MidLane,Mid_cnts,Outer_cnts)
            if(IsPathCrossing):
                OuterLanes = np.zeros_like(OuterLanes)#Empty outerLane
            else:
                return OuterLanes ,Outer_cnts, Mid_cnts,0

        # 4. >>>>>>>>>>>>>> Condition 2 : if MidLane is present but no Outlane detected >>>>>>>>>>>>>> Or Outlane got zerod because of crossings Midlane
        # Action: Create Outlane on Side that represent the larger Lane as seen by camera
        
        if(not np.any(OuterLanes>0)):
            # Fetching the column of the lowest point of the midlane 
            Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
            Mid_Rows = Mid_cnts_Rowsorted.shape[0]
            Mid_lowP = Mid_cnts_Rowsorted[Mid_Rows-1,:]
            Mid_highP = Mid_cnts_Rowsorted[0,:]
            Mid_low_Col = Mid_lowP[0]

            # Addresing which side to draw the outerlane considering it was present before or not		
            DrawRight = False
            if NoOuterLane_before:
                if(Mid_low_Col < int(MidLane.shape[1]/2)):
                    DrawRight = True
            else:
                if IsCrossingLeft:
                    DrawRight = True

            # Setting outerlane upperand lower points column to the right if draw right and vice versa
            if DrawRight:
                low_Col=(int(MidLane.shape[1])-1)
                high_Col=(int(MidLane.shape[1])-1)
                Offset_correction = 20
            else:
                low_Col=0
                high_Col=0
                Offset_correction = -20
            
            Mid_lowP[1] = MidLane.shape[0]# setting mid_trajectory_lowestPoint_Row to MaxRows of Image
            LanePoint_lower =  (low_Col , int( Mid_lowP[1] ) )
            LanePoint_top   =  (high_Col, int( Mid_highP[1]) )
            OuterLanes = cv2.line(OuterLanes,LanePoint_lower,LanePoint_top,255,1)
            Outer_cnts = cv2.findContours(OuterLanes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
            return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction

    # 5. Condition 3 [No MidLane]
    else:
        return OuterLanes, Outer_cnts, Mid_cnts, Offset_correction

def ExtendShortLane(MidLane,Mid_cnts,Outer_cnts,OuterLane):

    # 1. Sorting the Mid and Outer Contours on basis of rows (Ascending) [Max row is the last row]
    if(Mid_cnts and Outer_cnts):
        Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
        Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")
        Image_bottom = MidLane.shape[0]
        total_no_of_cnts_midlane = Mid_cnts_Rowsorted.shape[0]
        total_no_of_cnts_outerlane = Outer_cnts_Rowsorted.shape[0]

        # 2. Connect Midlane to imagebottom by drawing a Vertical line if not alraedy connected
        BottomPoint_Mid = Mid_cnts_Rowsorted[total_no_of_cnts_midlane-1,:]
        if (BottomPoint_Mid[1] < Image_bottom):
            MidLane = cv2.line(MidLane,tuple(BottomPoint_Mid),(BottomPoint_Mid[0],Image_bottom),255,2)

        # 3. Connect Outerlane to imagebottom by performing 2 steps (if neccasary)
            # [Step 1]: Extend Outerlane in the direction of its slope

        ## A) Taking last 20 points to estimate slope
        BottomPoint_Outer = Outer_cnts_Rowsorted[total_no_of_cnts_outerlane-1,:]
        if (BottomPoint_Outer[1] < Image_bottom):
            if(total_no_of_cnts_outerlane>20):
                shift=20
            else:
                shift=2
            RefLast10Points = Outer_cnts_Rowsorted[total_no_of_cnts_outerlane-shift:total_no_of_cnts_outerlane-1:2,:]

            ## B) Estimating Slope
            if(len(RefLast10Points)>1):# Atleast 2 points needed to estimate a line
                Ref_x = RefLast10Points[:,0]#cols
                Ref_y = RefLast10Points[:,1]#rows
                Ref_parameters = np.polyfit(Ref_x, Ref_y, 1)
                Ref_slope = Ref_parameters[0]
                Ref_yiCntercept = Ref_parameters[1]

                ## C) Extending outerlane in the direction of its slope
                if(Ref_slope < 0):
                    Ref_LineTouchPoint_col = 0
                    Ref_LineTouchPoint_row = Ref_yiCntercept
                else:
                    Ref_LineTouchPoint_col = OuterLane.shape[1]-1 # Cols have lenth of ColLength But traversal is from 0 to ColLength-1
                    Ref_LineTouchPoint_row = Ref_slope * Ref_LineTouchPoint_col + Ref_yiCntercept
                Ref_TouchPoint = (Ref_LineTouchPoint_col,int(Ref_LineTouchPoint_row))#(col ,row)
                Ref_BottomPoint_tup = tuple(BottomPoint_Outer)
                OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_BottomPoint_tup,255,2)

                # 3 [Step 2]: If required, connect outerlane to bottom by drawing a vertical line
                if(Ref_LineTouchPoint_row < Image_bottom):
                    Ref_TouchPoint_Ref = (Ref_LineTouchPoint_col,Image_bottom)
                    OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_TouchPoint_Ref,255,3)

    return MidLane,OuterLane


        