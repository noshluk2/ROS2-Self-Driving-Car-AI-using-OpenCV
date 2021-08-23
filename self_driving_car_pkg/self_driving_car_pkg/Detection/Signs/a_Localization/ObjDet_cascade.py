import cv2
import numpy as np

from TLD import detect_TrafficLight
from self_driving_car_pkg.Detection.Signs.c_Tracking.OpticalFlow_adv import signTrack,Track

TrafficLight_cascade_str = "self_driving_car_pkg/data/TrafficLight_cascade.xml"
TrafficLight_cascade = cv2.CascadeClassifier()
#-- 1. Load the cascades
if not TrafficLight_cascade.load(cv2.samples.findFile(TrafficLight_cascade_str)):
    print('--(!)Error loading face cascade')
    exit(0)


def detect_TrafficLight_Cscde(img):

    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    target = TrafficLight_cascade.detectMultiScale(gray_image)
    for (x,y,w,h) in target:
        cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
        roi_gray = gray_image[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
    cv2.imshow('detected_TrafficLight', img)
    if target:
        TrafficLight_Rect = target[0]
    else:
        TrafficLight_Rect = (0,0,0,0)

    return TrafficLight_Rect


def detect_TrafficLight_Cscde2(img):
    img_draw=img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    target = TrafficLight_cascade.detectMultiScale(gray)
    TrafficLightFound=False

    for (x,y,w,h) in target:
        cv2.rectangle(img_draw, (x,y), (x+w,y+h), (0,165,255), 2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        TL_Maybe_mask = np.zeros(gray.shape,np.uint8)
        TL_Maybe_mask[y:y+h,x:x+w] = 255
        img_ROI = cv2.bitwise_and(img,img,mask=TL_Maybe_mask)
        cv2.imshow('img_ROI', img_ROI)
        # Reconfirm if detected Traffic Light was the desired one
        Traffic_State = detect_TrafficLight(img_ROI,img_draw)
        if(Traffic_State!="Unknown"):
            print("Traffic State Recived",Traffic_State)
            # Confirm Traffic Light 
            cv2.rectangle(img_draw, (x,y), (x+w,y+h), (0,255,0), 2)
            # Start Tracking
            TrafficLightFound = True
            cv2.imshow('detected_TrafficLight', img_draw)
            cv2.waitKey(0)

    cv2.imshow('detected_TrafficLight', img_draw)
    cv2.waitKey(1)

    
    if TrafficLightFound:
        TrafficLight_Rect = target[0]
    else:
        TrafficLight_Rect = (0,0,0,0)

    return TrafficLight_Rect

def Testing():
    cap = cv2.VideoCapture("self_driving_car_pkg/data/vids/ishara_sign_conflict.avi")


    while 1:
        
        # 1. Read first frame.
        ret,img = cap.read()

        if ret:
            # Every form of drawing will be done on this stupit image
            frame_draw = img.copy()

            # 4. Checking if SignTrack Class mode is Tracking If yes Proceed
            if(signTrack.mode == "Tracking"):
                # Start timer
                timer = cv2.getTickCount()
                Track(img,frame_draw)
                # Calculate Frames per second (FPS)
                fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
                # Display FPS on frame
                cv2.putText(frame_draw, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
                # 2. Display frame and wait for keypress
                cv2.imshow("Tracked_ROI",signTrack.Tracked_ROI)

            # 3. If SignTrack is in Detection Proceed to intialize tracker
            elif (signTrack.mode == "Detection"):

                # 3a. Select the ROI which u want to track
                #r = cv2.selectROI("SelectROI",img)
                r = detect_TrafficLight_Cscde2(img)
                # 3b. Convert Rgb to gray
                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                # 3c. creating ROI mask
                ROI_mask = np.zeros_like(gray)
                ROI_mask[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = 255
                signTrack.Tracked_ROI = ROI_mask
                # 3d. Updating signtrack class with variables initialized
                signTrack.mode = "Tracking" # Set mode to tracking
                signTrack.Tracked_class = "Circle" # keep tracking frame sign name
                signTrack.p0 = cv2.goodFeaturesToTrack(gray, mask=ROI_mask, **signTrack.feature_params)
                signTrack.old_gray = gray.copy()
                signTrack.mask = np.zeros_like(frame_draw)

            elif(k==27):# If 'Esc' pressed then Exit
                break

            # 2. Display frame and wait for keypress
            cv2.imshow("frame",frame_draw)
            k = cv2.waitKey(20)        

        else:
            break

Testing()

