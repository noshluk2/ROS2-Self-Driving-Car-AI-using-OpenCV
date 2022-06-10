'''
> Purpose :
Module to perform localization of robot using Background Subtraction.

> Usage :
You can perform localization of the robot by
1) Importing the class (bot_localizer)
2) Creating its object
3) Accessing the object's function of localize bot. 
E.g ( self.bot_localizer.localize_bot(self.sat_view, frame_disp) )


> Inputs:
1) Extracted frame from video feed of (Satellite or DroneCam)
2) Frame To display the localized robot

> Outputs:
1) self.car_loc => Cordinates (X,Y) of the localized car
2) self.maze_og => Occupancy Grid generated from the cropped maze

Author :
Haider Abbasi

Date :
6/04/22
'''
import cv2
import numpy as np

from .utilities import ret_smallest_obj,ret_largest_obj
from . import config


class bot_localizer():

    def __init__(self):

        # State Variables
        self.is_bg_extracted =False

        # Output Variables [BG_model,Refrence_Maze,Rel_Loc_of_car]
        self.bg_model = []
        self.maze_og = []
        self.loc_car = 0

        # Transfomation(Crop + Rotated) Variables
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []

        self.orig_rot = 0
        self.rot_mat = 0
    
    def update_frameofrefrence_parameters(self,X,Y,W,H,rot_angle):
        self.orig_X = X; self.orig_Y = Y; self.orig_rows = H; self.orig_cols = W; self.orig_rot = rot_angle # 90 degree counterClockwise
        self.transform_arr = [X,Y,W,H]
        # Rotation Matrix
        self.rot_mat = np.array(
                                [
                                 [ np.cos(np.deg2rad(self.orig_rot)) , np.sin(np.deg2rad(self.orig_rot))],
                                 [-np.sin(np.deg2rad(self.orig_rot)) , np.cos(np.deg2rad(self.orig_rot))]
                                ]
                               )
        self.rot_mat_rev = np.array(
                                [
                                 [ np.cos(np.deg2rad(-self.orig_rot)) , np.sin(np.deg2rad(-self.orig_rot))],
                                 [-np.sin(np.deg2rad(-self.orig_rot)) , np.cos(np.deg2rad(-self.orig_rot))]
                                ]
                               )


    # Program to find most frequent element in a list
    @staticmethod
    def most_frequent(List):
        return max(set(List), key = List.count)
    
    def refine_road_mask(self,edges,mask):

        cnts = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[1]
        cnts_boundedAreas = [0]*len(cnts)
        # [50less, 10less,150less,200less]
        pix_count = 50
        for idx,cnt in enumerate(cnts):
            area_idx = ((cnt.shape[0])//pix_count)
            cnts_boundedAreas[idx]= area_idx

        max_occuring = self.most_frequent(cnts_boundedAreas)
        cnts_small_removed = []
        for idx,cnt in enumerate(cnts):
            if ( cnt.shape[0] > ( ( (max_occuring+1)*pix_count ) + 50 ) ):
                cnts_small_removed.append(cnt)
                
        edges_small_remvd = np.zeros_like(edges)
        cv2.drawContours(edges_small_remvd, cnts_small_removed, -1, 255,1)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        edges_small_remvd = cv2.morphologyEx(edges_small_remvd, cv2.MORPH_CLOSE, kernel)

        _ ,contours ,hierarchy = cv2.findContours(edges_small_remvd, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Identifying and removing wrongly detected holes as part of road network
        lrgst_hole_h1_idx = -1
        prev_max = 0
        for i,cnt in enumerate(contours):
            # look for hierarchy[i][3]==0, ie holes under first hierarchy
            if ( hierarchy[0][i][3] == 0 ):
                if contours[i].shape[0]>prev_max:
                    prev_max = contours[i].shape[0]
                    lrgst_hole_h1_idx = i 
        
        road_hole_mask = np.zeros_like(mask)
        if lrgst_hole_h1_idx!=-1:
            # We have largest hole in first hierarchy (Insides of the road)
            child_idx = hierarchy[0][lrgst_hole_h1_idx][2]
            
            # Look until there is a hole present
            while (child_idx != -1):
                # We have a child contour inside the road hole 
                if ( contours[child_idx].shape[0] > ((max_occuring+1)*pix_count) ):
                    # Large Enough
                    cv2.drawContours(road_hole_mask, contours, child_idx, 255,-1)
                    # Look for next contour to current
                    nxtchild_idx = hierarchy[0][child_idx][0]
                    # Set child_idx to nxtChild_idx
                    child_idx = nxtchild_idx

        road_noholes_mask = cv2.bitwise_not(road_hole_mask)
        mask = cv2.bitwise_and(mask, mask,mask=road_noholes_mask)

        return mask

    @staticmethod
    def connect_objs(bin_img):

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
        cnts = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
        
        cncted_objects_list = []
        for idx,_ in enumerate(cnts):
            temp = np.zeros_like(bin_img)
            cncted_objects_list.append(cv2.drawContours(temp, cnts, idx, 255,-1))
            cncted_objects_list[idx] = cv2.morphologyEx(cncted_objects_list[idx], cv2.MORPH_CLOSE, kernel)
        cncted_objects = sum(cncted_objects_list)

        return cncted_objects

    def extract_bg(self,frame):

        # a) Find Contours of all ROI's in frozen sat_view 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges_canny = cv2.Canny(gray, 50, 150,None,3)

        # [connect_objs] => Connect disconnected edges that are close enough
        edges = self.connect_objs(edges_canny)

        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
        rois_mask = np.zeros((frame.shape[0],frame.shape[1]),dtype= np.uint8)
        
        for idx,_ in enumerate(cnts):
            cv2.drawContours(rois_mask, cnts, idx, 255,-1)

        # b) Extract BG_model by 
        #               i)  removing the smallest object from the scene (Bot)
        #               ii) filling the empty region with Ground_replica
        min_cntr_idx = ret_smallest_obj(cnts)
        rois_noCar_mask = rois_mask.copy()
        #    If Smallest Object (FG) found         
        if min_cntr_idx !=-1:
            cv2.drawContours(rois_noCar_mask, cnts, min_cntr_idx, 0,-1)
            # Drawing dilated car_mask
            car_mask = np.zeros_like(rois_mask)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255,-1)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, 3)
            notCar_mask = cv2.bitwise_not(car_mask)
            frame_car_remvd = cv2.bitwise_and(frame, frame,mask = notCar_mask)
            # Generating ground replica 
            base_clr = frame_car_remvd[0][0]
            Ground_replica = np.ones_like(frame)*base_clr
            # Generating BG_model
            self.bg_model = cv2.bitwise_and(Ground_replica, Ground_replica,mask = car_mask)
            self.bg_model = cv2.bitwise_or(self.bg_model, frame_car_remvd)
        
        # Step 2: Extracting the maze (Frame of Refrence) Maze Entry on Top
        # a) Finding dimensions of hull enclosing largest contour
        road_network_mask, road_network_cnt = ret_largest_obj(rois_mask)
        
        # Fetching edges of only road network
        road_network_edges = cv2.bitwise_and(edges_canny, edges_canny,mask=road_network_mask)
        # Removing holes wrongly considered to be part of roads
        road_network_mask = self.refine_road_mask(road_network_edges,road_network_mask)
        
        [X,Y,W,H] = cv2.boundingRect(road_network_cnt)
        # b) Cropping maze_mask from the image
        maze_occupencygrid = road_network_mask[Y:Y+H,X:X+W]
        
        # Creating Boundary-at-edges to represent no-go areas
        maze_occupencygrid_rotated = cv2.rotate(maze_occupencygrid, cv2.ROTATE_90_COUNTERCLOCKWISE)
        rows,cols = maze_occupencygrid_rotated.shape
        self.maze_og  = cv2.rectangle(maze_occupencygrid_rotated, (0,0), (cols-1,rows-1), 0,10)

        # Storing Crop and Rot Parameters required to maintain frame of refrence in the orig image
        self.update_frameofrefrence_parameters(X,Y,W,H,90)

        if (config.debug and config.debug_localization):
            cv2.imshow("1a. rois_mask",rois_mask)
            cv2.imshow("1b. frame_car_remvd",frame_car_remvd)
            cv2.imshow("1c. Ground_replica",Ground_replica)
            cv2.imshow("1d. bg_model",self.bg_model)
            cv2.imshow("2. maze_og",self.maze_og)

    @staticmethod
    def get_centroid(cnt):
        M = cv2.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx,cy)

    def get_car_loc(self,car_cnt,car_mask):
        
        # a) Get the centroid of the car
        bot_cntr = self.get_centroid(car_cnt)
        # b) Converting from point --> array to apply transforms
        bot_cntr_arr =  np.array([bot_cntr[0],bot_cntr[1]])
        # c) Shift origin from sat_view -> maze
        bot_cntr_translated = np.zeros_like(bot_cntr_arr)

        bot_cntr_translated[0] = bot_cntr_arr[0] - self.orig_X
        bot_cntr_translated[1] = bot_cntr_arr[1] - self.orig_Y

        # d) Applying rotation tranformation to bot_centroid to get bot location relative to maze
        bot_on_maze = (self.rot_mat @ bot_cntr_translated.T).T

        center_ = np.array([int(car_mask.shape[1]/2),int(car_mask.shape[0]/2)])
        chachu = (self.rot_mat @ center_.T).T

        # e) Translating Origin If neccesary (To get complete Image)
        rot_cols = self.orig_rows
        rot_rows = self.orig_cols

        bot_on_maze[0] = bot_on_maze[0] + (rot_cols * (chachu[0]<0) )  
        bot_on_maze[1] = bot_on_maze[1] + (rot_rows * (chachu[1]<0) )
        # Update the placeholder for relative location of car
        self.loc_car = (int(bot_on_maze[0]),int(bot_on_maze[1]))


    def localize_bot(self,curr_frame,frame_disp):
        
        # Step 1: Background Model Extraction
        if not self.is_bg_extracted:
            self.extract_bg(curr_frame.copy())
            self.is_bg_extracted = True
            
        # Step 2: Foreground Detection
        change = cv2.absdiff(curr_frame, self.bg_model)
        change_gray = cv2.cvtColor(change, cv2.COLOR_BGR2GRAY)
        change_mask = cv2.threshold(change_gray, 15, 255, cv2.THRESH_BINARY)[1]
        car_mask, car_cnt = ret_largest_obj(change_mask)

        # Step 3: Fetching the (relative) location of car.
        self.get_car_loc(car_cnt,car_mask)

        # Drawing bounding circle around detected car
        center, radii = cv2.minEnclosingCircle(car_cnt)
        car_circular_mask = cv2.circle(car_mask.copy(), (int(center[0]), int(center[1])), int(radii+(radii*0.4)), 255, 3)
        car_circular_mask = cv2.bitwise_xor(car_circular_mask, car_mask)
        frame_disp[car_mask>0]  = frame_disp[car_mask>0] + (0,64,0)
        frame_disp[car_circular_mask>0]  = (0,0,255)


        if (config.debug and config.debug_localization):
            cv2.imshow("1d. bg_model",self.bg_model)
            cv2.imshow("2. maze_og",self.maze_og)
            
            cv2.imshow("change_mask(Noise Visible)", change_mask) 
            cv2.imshow("Detected_foreground(car)", car_mask)

        else:
            try:
                cv2.destroyWindow("1d. bg_model")
                cv2.destroyWindow("2. maze_og")
                
                cv2.destroyWindow("change_mask(Noise Visible)")
                cv2.destroyWindow("Detected_foreground(car)")

                cv2.destroyWindow("1a. rois_mask")
                cv2.destroyWindow("1b. frame_car_remvd")
                cv2.destroyWindow("1c. Ground_replica")

            except:
                pass
