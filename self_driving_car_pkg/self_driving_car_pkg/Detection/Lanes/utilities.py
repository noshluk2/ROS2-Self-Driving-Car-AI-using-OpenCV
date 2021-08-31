import numpy as np
import cv2
import math 

def Distance(a,b):
    a_y = a[0,0]
    a_x = a[0,1]
    b_y = b[0,0]
    b_x = b[0,1]
    distance = math.sqrt( ((a_x-b_x)**2)+((a_y-b_y)**2) )
    return distance

def Distance_(a,b):
    return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )

def findlaneCurvature(x1,y1,x2,y2):
    offset_Vert=90# angle found by tan-1 (slop) is wrt horizontal --> This will shift to wrt Vetical

    if((x2-x1)!=0):
        slope = (y2-y1)/(x2-x1)
        y_intercept = y2 - (slope*x2) #y= mx+c
        anlgeOfinclination = math.atan(slope) * (180 / np.pi)#Conversion to degrees
    else:
        slope=1000#infinity
        y_intercept=0#None [Line never crosses the y axis]

        anlgeOfinclination = 90#vertical line

        #print("Vertical Line [Undefined slope]")
    if(anlgeOfinclination!=90):
        if(anlgeOfinclination<0):#right side
            angle_wrt_vertical = offset_Vert + anlgeOfinclination
        else:#left side
            angle_wrt_vertical = anlgeOfinclination - offset_Vert
    else:
        angle_wrt_vertical= 0#aligned
    return angle_wrt_vertical
 
def findLineParameter(x1,y1,x2,y2):
    if((x2-x1)!=0):
        slope = (y2-y1)/(x2-x1)
        y_intercept = y2 - (slope*x2) #y= mx+c
    else:
        slope=1000
        y_intercept=0
        #print("Vertical Line [Undefined slope]")
    return (slope,y_intercept)

def Cord_Sort(cnts,order):

    if cnts:
        cnt=cnts[0]
        cnt=np.reshape(cnt,(cnt.shape[0],cnt.shape[2]))
        order_list=[]
        if(order=="rows"):
            order_list.append((0,1))
        else:
            order_list.append((1,0))
        ind = np.lexsort((cnt[:,order_list[0][0]],cnt[:,order_list[0][1]]))
        Sorted=cnt[ind]
        return Sorted
    else:
        return cnts

def average_2b_(Edge_ROI):
    #First Threshold data
    TrajectoryOnEdge = np.copy(Edge_ROI)
    row = Edge_ROI.shape[0] # Shape = [row, col, channels]
    col = Edge_ROI.shape[1]
    Lane_detected = np.zeros(Edge_ROI.shape,dtype = Edge_ROI.dtype)
    Edge_Binary = Edge_ROI > 0
    Edge_Binary_nz_pix = np.where(Edge_Binary)
    x_len = Edge_Binary_nz_pix[0].shape[0]

    if(Edge_Binary_nz_pix[0].shape[0]):
        y = Edge_Binary_nz_pix[0]
        x = Edge_Binary_nz_pix[1]
        Zpoly = np.polyfit(x, y, 2)
        Zpoly_Func = np.poly1d(Zpoly)
        # calculate new x's and y's
        x_new = np.linspace(0, col, col)
        y_new = Zpoly_Func(x_new)
        
        x_new = x_new.astype(np.int32)
        y_new = y_new.astype(np.int32)

        draw_points = (np.asarray([x_new, y_new]).T).astype(np.int32)   # needs to be int32 and transposed

        cv2.polylines(TrajectoryOnEdge, [draw_points], False, (255,255,255),2)  # args: image, points, closed, color
        cv2.polylines(Lane_detected, [draw_points], False, (255,255,255),2)  # args: image, points, closed, color

    #cv2.namedWindow("TrajectoryOnEdge",cv2.WINDOW_NORMAL)
    #cv2.imshow("TrajectoryOnEdge",TrajectoryOnEdge)
    return Lane_detected


