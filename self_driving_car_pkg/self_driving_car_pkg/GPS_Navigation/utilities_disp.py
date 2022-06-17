import cv2
import numpy as np
from math import pi,cos,sin
from . import config


# Overlay detected regions over the bot_view
def overlay(image,overlay_img):

    gray = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
    mask_inv = cv2.bitwise_not(mask)


    roi = image
    img2 = overlay_img
    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
    
    image = img1_bg + img2_fg
    return image

# Overlay detected regions (User-specified-amount) over the frame_disp
def overlay_cropped(frame_disp,image_rot,crop_loc_row,crop_loc_col,overlay_cols):
    
    image_rot_cols = image_rot.shape[1]
    gray = cv2.cvtColor(image_rot[:,image_rot_cols-overlay_cols:image_rot_cols], cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)[1]
    mask_inv = cv2.bitwise_not(mask)

    frame_overlay_cols = crop_loc_col + image_rot_cols
    roi = frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols]            
    img2 = image_rot[:,image_rot_cols-overlay_cols:image_rot_cols]

    # Now black-out the area of logo in ROI
    img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
    # Take only region of logo from logo image.
    img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
    
    frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols] = img1_bg + img2_fg


def overlay_live(frame_disp,overlay,overlay_map,overlay_path,transform_arr,crp_amt):

    overlay_rot = cv2.rotate(overlay, cv2.ROTATE_90_CLOCKWISE)
    map_rot = cv2.rotate(overlay_map, cv2.ROTATE_90_CLOCKWISE)
    image_rot = cv2.rotate(overlay_path, cv2.ROTATE_90_CLOCKWISE)

    crop_loc_col = transform_arr[0]+crp_amt
    #crop_loc_endCol = transform_arr[0]+transform_arr[2]+crp_amt
    crop_loc_row = transform_arr[1]+crp_amt

    new_cols = int(overlay_rot.shape[1]*config.debug_live_amount)
    new_path_cols = int(overlay_rot.shape[1]*config.debug_path_live_amount)
    new_map_cols = int(overlay_rot.shape[1]*config.debug_map_live_amount)


    frame_disp[crop_loc_row:crop_loc_row + overlay_rot.shape[0],crop_loc_col:crop_loc_col + new_cols] = overlay_rot[:,0:new_cols]
    
    if config.debug_map_live_amount>0:
        overlay_cropped(frame_disp,map_rot,crop_loc_row,crop_loc_col,new_map_cols)
    if config.debug_path_live_amount>0:
        overlay_cropped(frame_disp,image_rot,crop_loc_row,crop_loc_col,new_path_cols)

# Draw speedometer and arrows indicating bot speed and direction at given moment
def draw_bot_speedo(image,bot_speed,bot_turning):
    height, width = image.shape[0:2]
    # Ellipse parameters
    radius = 50
    center = (int(width / 2), height - 25)
    axes = (radius, radius)
    angle = 0
    startAngle = 180
    endAngle = 360
    thickness = 10

    # http://docs.opencv.org/modules/core/doc/drawing_functions.html#ellipse
    cv2.ellipse(image, center, axes, angle, startAngle, endAngle, (0,0,0), thickness)

    Estimted_line = np.zeros_like(image)
    max_speed = 1.5
    angle = -(((bot_speed/max_speed)*180)+90)
    speed_mph = int((bot_speed/max_speed)*200)
    length = 300
    P1 = center
    
    P2 = ( 
            int(P1[0] + length * sin(angle * (pi / 180.0) ) ),
            int(P1[1] + length * cos(angle * (pi / 180.0) ) ) 
            )
    
    cv2.line(Estimted_line,center, P2, (255,255,255),3)
    meter_mask = np.zeros((image.shape[0],image.shape[1]),np.uint8)

    cv2.ellipse(meter_mask, center, axes, angle, 0, endAngle, 255, -1)

    Estimted_line = cv2.bitwise_and(Estimted_line, Estimted_line,mask = meter_mask)

    speed_clr = (0,0,0)
    if speed_mph<20:
        speed_clr = (0,255,255)
    elif speed_mph<40:
        speed_clr = (0,255,0)
    elif speed_mph<60:
        speed_clr = (0,140,255)
    elif speed_mph>=60:
        speed_clr = (0,0,255)
    cv2.putText(image, str(speed_mph), (center[0]+10,center[1]-10), cv2.FONT_HERSHEY_PLAIN, 2, speed_clr,3)


    image = overlay(image,Estimted_line)
    if bot_turning>0.2:
        image = cv2.arrowedLine(image, (40,int(image.shape[0]/2)), (10,int(image.shape[0]/2)),
                                        (0,140,255), 13,tipLength=0.8)
    elif bot_turning<-0.2:
        image = cv2.arrowedLine(image, (image.shape[1]-40,int(image.shape[0]/2)), (image.shape[1]-10,int(image.shape[0]/2)),
                                        (0,140,255), 13,tipLength=0.8)

    return image


def disp_SatNav(frame_disp,sbot_view,bot_curr_speed,bot_curr_turning,maze_interestPts,choosen_route,img_choosen_route,transform_arr,crp_amt):
    # View bot view on left to frame Display
    bot_view = cv2.resize(sbot_view,None,fx=0.95,fy=0.95)

    # Draw & Display [For better Understanding of current robot state]
    center_frame_disp = int(frame_disp.shape[0]/2)
    center_bot_view = int(bot_view.shape[0]/4)
    bot_offset = frame_disp.shape[0] - bot_view.shape[0] - 25
    center_img_shortest_path = int(img_choosen_route.shape[0]/2)
    isp_offset = center_frame_disp - center_img_shortest_path

    bot_view = draw_bot_speedo(bot_view,bot_curr_speed,bot_curr_turning)

    if config.debug_live:
        overlay_live(frame_disp,img_choosen_route,maze_interestPts,choosen_route,transform_arr,crp_amt)

    orig_col = 10 + int(bot_view.shape[1]/4)
    orig = (orig_col,bot_offset-10)
    cv2.putText(frame_disp, "Bot View", orig, cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0),3)
    frame_disp = cv2.rectangle(frame_disp, (20,bot_offset), (bot_view.shape[1]+20,(bot_view.shape[0]+bot_offset)), (0,0,255),12)
    frame_disp[bot_offset:(bot_view.shape[0]+bot_offset),20:bot_view.shape[1]+20] = bot_view
