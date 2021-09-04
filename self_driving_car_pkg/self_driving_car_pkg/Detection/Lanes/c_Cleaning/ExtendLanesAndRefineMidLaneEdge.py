import cv2
import numpy as np
from ....config import config
from ..utilities import Cord_Sort

def ExtendShortLane(MidLane,Mid_cnts,Outer_cnts,OuterLane):

	# 1. Sorting the Mid and Outer Contours on basis of rows (Ascending)
	if(Mid_cnts and Outer_cnts):		
		Mid_cnts_Rowsorted = Cord_Sort(Mid_cnts,"rows")
		Outer_cnts_Rowsorted = Cord_Sort(Outer_cnts,"rows")

		Image_bottom = MidLane.shape[0]
		
		Lane_Rows = Mid_cnts_Rowsorted.shape[0]
		Lane_Cols = Mid_cnts_Rowsorted.shape[1]
		BottomPoint_Mid = Mid_cnts_Rowsorted[Lane_Rows-1,:]	
		
		# 2. Connect Midlane to imagebottom by drawing a Vertical line
		if (BottomPoint_Mid[1] < Image_bottom):
			MidLane = cv2.line(MidLane,tuple(BottomPoint_Mid),(BottomPoint_Mid[0],Image_bottom),255)


		RefLane_Rows = Outer_cnts_Rowsorted.shape[0]
		RefLane_Cols = Outer_cnts_Rowsorted.shape[1]
		BottomPoint_Outer = Outer_cnts_Rowsorted[RefLane_Rows-1,:]

		# 3. Connect Outerlane to imagebottom by performing 2 steps if neccasary
		if (BottomPoint_Outer[1] < Image_bottom):
			if(RefLane_Rows>20):
				shift=20
			else:
				shift=2
			RefLast10Points = Outer_cnts_Rowsorted[RefLane_Rows-shift:RefLane_Rows-1:2,:]

			# 3a. Connect Outerlane to imagebottom by Estimating its sloping and extending in
			#     the direction of that slope	
			if(len(RefLast10Points)>1):# Atleast 2 points needed to estimate a line
				Ref_x = RefLast10Points[:,0]#cols
				Ref_y = RefLast10Points[:,1]#rows
				Ref_parameters = np.polyfit(Ref_x, Ref_y, 1)
				Ref_slope = Ref_parameters[0]
				Ref_yiCntercept = Ref_parameters[1]
				#Decreasing slope means Current lane is left lane and by going towards 0 x we touchdown
				if(Ref_slope < 0):
					Ref_LineTouchPoint_col = 0
					Ref_LineTouchPoint_row = Ref_yiCntercept
				else:
					Ref_LineTouchPoint_col = OuterLane.shape[1]-1 # Cols have lenth of ColLength But traversal is from 0 to ColLength-1
					Ref_LineTouchPoint_row = Ref_slope * Ref_LineTouchPoint_col + Ref_yiCntercept
				Ref_TouchPoint = (Ref_LineTouchPoint_col,int(Ref_LineTouchPoint_row))#(col ,row)
				Ref_BottomPoint_tup = tuple(BottomPoint_Outer)
				OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_BottomPoint_tup,255)
				
				# 3b. Incase extended outerlane is still less then image bottom extend by
				#     drawing a vertical line
				if(Ref_LineTouchPoint_row < Image_bottom):
					Ref_TouchPoint_Ref = (Ref_LineTouchPoint_col,Image_bottom)
					OuterLane = cv2.line(OuterLane,Ref_TouchPoint,Ref_TouchPoint_Ref,255)
	if (config.debugging and config.debugging_Lane and config.debugging_L_Cleaning):
		cv2.imshow("[ExtendShortLane] OuterLanes",OuterLane)
	else:
		cv2.destroyWindow("[ExtendShortLane] OuterLanes")
	return MidLane,OuterLane