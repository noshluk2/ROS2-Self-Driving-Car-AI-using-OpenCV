import cv2
import numpy as np

from . import config
import os

# [NEW]: find largest contour (max pixel amt)
def ret_largest_reg(mask):
    cnts = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[1]
    max_cntr_pix = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        curr_cnt_pix = cnt.shape[0]
        if curr_cnt_pix > max_cntr_pix:
            max_cntr_pix = curr_cnt_pix
            Max_Cntr_idx = index
    
    largst_reg_mask = np.zeros_like(mask)
    largst_reg_mask = cv2.drawContours(largst_reg_mask, cnts, Max_Cntr_idx, 255,-1)
    if Max_Cntr_idx!=-1:
        return cnts[Max_Cntr_idx],largst_reg_mask
    else:
        cnts,mask

# [NEW]: function to display provided screen on a device
def disp_on_mydev(screen,device="tablet"):
    resource_dir = "self_driving_car_pkg/self_driving_car_pkg/GPS_Navigation/resource"
    device_path = os.path.join(resource_dir,device) + ".png"
    device_view = cv2.imread(device_path)
    device_hls = cv2.cvtColor(device_view, cv2.COLOR_BGR2HLS)

    # Case : If the screen is the middle is brighter then everything else
    mask = cv2.inRange(device_hls, np.array([0,150,0]), np.array([255,255,255]))
    largst_reg_cnt,largst_reg_mask = ret_largest_reg(mask)
    [x,y,w,h] = cv2.boundingRect(largst_reg_cnt)

    dsize = (screen.shape[1]+ (2*x), screen.shape[0]+(2*y))
    device_view = cv2.resize(device_view, dsize)

    device_view[y:screen.shape[0]+y,x:screen.shape[1]+x] = screen
    return device_view,x,y
# [NEW]: Find closest point in a list of point to a specific position
def closest_node(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=(nodes.ndim-1))
    return np.argmin(dist_2)

# [NEW]: Find centroid of a contour
def get_centroid(cnt):

    M = cv2.moments(cnt)
    if M['m00']==0:
        (cx,cy) = cv2.minEnclosingCircle(cnt)[0]        
        return (int(cx),int(cy))
    else:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx,cy)

# [NEW]: Update the destination to user selected location
def click_event(event, x, y, flags, params):

    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:

        # displaying the coordinates
        # on the Shell
        config.destination = (x,y)

# [NEW]: Transform point to new Frame of Refrence [described by provided rot and translation tranformations]
def find_point_in_FOR(bot_cntr,transform_arr,rot_mat,cols,rows):
        
        # b) Converting from point --> array to apply transforms
        bot_cntr_arr =  np.array([bot_cntr[0],bot_cntr[1]])
        # c) Shift origin from sat_view -> maze
        bot_cntr_translated = np.zeros_like(bot_cntr_arr)

        bot_cntr_translated[0] = bot_cntr_arr[0] - transform_arr[0]
        bot_cntr_translated[1] = bot_cntr_arr[1] - transform_arr[1]

        # d) Applying rotation tranformation to bot_centroid to get bot location relative to maze
        bot_on_maze = (rot_mat @ bot_cntr_translated.T).T

        center_ = np.array([int(cols/2),int(rows/2)])
        center_rotated = (rot_mat @ center_.T).T

        # e) Translating Origin If neccesary (To get complete Image)
        rot_cols = transform_arr[3]
        rot_rows = transform_arr[2]

        bot_on_maze[0] = bot_on_maze[0] + (rot_cols * (center_rotated[0]<0) )  
        bot_on_maze[1] = bot_on_maze[1] + (rot_rows * (center_rotated[1]<0) )
        # Update the placeholder for relative location of car
        Point_on_FOR = (int(bot_on_maze[0]),int(bot_on_maze[1]))
        
        return Point_on_FOR


def imfill(image):
  cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]# OpenCV 4.2
  for idx,_ in enumerate(cnts):
    cv2.drawContours(image, cnts, idx, 255,-1)

def ret_largest_obj(img):
    #Find the two Contours for which you want to find the min distance between them.
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
    Max_Cntr_area = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
    img_largestobject = np.zeros_like(img)
    # handling boundary condition [Incase no largest found]
    if (Max_Cntr_idx!=-1):
        img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
        #img_largestobject = cv2.drawContours(img_largestobject, cnts, Max_Cntr_idx, 255, 2) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
        return img_largestobject,cnts[Max_Cntr_idx]
    else:
        return img,cnts

def ret_smallest_obj(cnts, noise_thresh = 10):
  Min_Cntr_area = 1000
  Min_Cntr_idx= -1
  for index, cnt in enumerate(cnts):
      area = cv2.contourArea(cnt)
      if (area < Min_Cntr_area) and (area > 10):
          Min_Cntr_area = area
          Min_Cntr_idx = index
          SmallestContour_Found = True
  print("min_area" , Min_Cntr_area)
  return Min_Cntr_idx

class Debugging:

    def __init__(self): 
       self.time_elasped = 0
       self.Live_created = False


    def nothing(self,x):
        pass

    cv2.namedWindow('CONFIG')
    # create switch for ON/OFF functionality
    debugging_SW = 'Debug'
    cv2.createTrackbar(debugging_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingLoc_SW = 'Debug Loc'
    cv2.createTrackbar(debuggingLoc_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingMapping_SW = 'Debug Mapp.'
    cv2.createTrackbar(debuggingMapping_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingPathPlanning_SW = 'Debug Path P.'
    cv2.createTrackbar(debuggingPathPlanning_SW, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality
    debuggingMotionPlanning_SW = 'Debug Motion P.'
    cv2.createTrackbar(debuggingMotionPlanning_SW, 'CONFIG',False,True,nothing)

    # create switch for ON/OFF functionality
    debugging_Live = 'Debug_Live'
    cv2.createTrackbar(debugging_Live, 'CONFIG',False,True,nothing)
    # create switch for ON/OFF functionality

    def setDebugParameters(self):

        if (self.time_elasped >5):
            # get current positions of four trackbars
            debug = cv2.getTrackbarPos(self.debugging_SW,'CONFIG')
            debug_localization = cv2.getTrackbarPos(self.debuggingLoc_SW,'CONFIG')
            debug_mapping = cv2.getTrackbarPos(self.debuggingMapping_SW,'CONFIG')
            debug_pathplanning = cv2.getTrackbarPos(self.debuggingPathPlanning_SW,'CONFIG')
            debug_motionplanning = cv2.getTrackbarPos(self.debuggingMotionPlanning_SW,'CONFIG')
            debug_live = cv2.getTrackbarPos(self.debugging_Live,'CONFIG')

            if debug:
                config.debug = True
            else:
                config.debug = False

            if debug_localization:
                config.debug_localization = True
            else:
                config.debug_localization = False    
            if debug_mapping:
                config.debug_mapping = True
            else:
                config.debug_mapping = False           
            if debug_pathplanning:
                config.debug_pathplanning = True
            else:
                config.debug_pathplanning = False
            if debug_motionplanning:
                config.debug_motionplanning = True
            else:
                config.debug_motionplanning = False
            if debug_live:
                config.debug_live = True
            else:
                config.debug_live = False
        else: 

            self.time_elasped +=1


        
        if config.debug_live:
            debuggingLIVEConfig_SW = 'Debug (Live)'
            debuggingMAPLIVEConfig_SW = 'Debug_map (Live)'
            debuggingPathLIVEConfig_SW = 'Debug_path (Live)'
            if not self.Live_created:
                self.Live_created = True
                cv2.namedWindow('CONFIG_LIVE')
                cv2.createTrackbar(debuggingLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)
                cv2.createTrackbar(debuggingMAPLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)
                cv2.createTrackbar(debuggingPathLIVEConfig_SW, 'CONFIG_LIVE',0,100,self.nothing)

            debug_live_amount = cv2.getTrackbarPos(debuggingLIVEConfig_SW,'CONFIG_LIVE')
            debug_map_live_amount = cv2.getTrackbarPos(debuggingMAPLIVEConfig_SW,'CONFIG_LIVE')
            debug_path_live_amount = cv2.getTrackbarPos(debuggingPathLIVEConfig_SW,'CONFIG_LIVE')

            config.debug_live_amount = (debug_live_amount/100)
            config.debug_map_live_amount = (debug_map_live_amount/100)
            config.debug_path_live_amount = (debug_path_live_amount/100)

        else:
            self.Live_created = False
            try:
                cv2.destroyWindow('CONFIG_LIVE')
            except:
                pass  


