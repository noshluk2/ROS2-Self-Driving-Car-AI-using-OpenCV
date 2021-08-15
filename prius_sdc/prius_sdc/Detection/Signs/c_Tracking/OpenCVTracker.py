import cv2
import sys
import os

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

def main():
    # Set up tracker.

    tracker_types = ['MEDIANFLOW', 'CSRT']
    tracker_type = tracker_types[0]

    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type)
    else:
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
            #tracker = cv2.TrackerBoosting_create()
        elif tracker_type == "CSRT":
            tracker = cv2.TrackerCSRT_create()

    # Read video
    video = cv2.VideoCapture("data/vids/signs_multi.mp4")
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()
        
    while(1):

        # 1. Read first frame.
        ok, frame = video.read()
        
        if not ok:
            print('Cannot read video file')
            break
        
        # 2. Display frame and wait for keypress
        cv2.imshow("frame",frame)
        k = cv2.waitKey(130)

        # 3. If 'c' is pressed Proceed to intialize tracker
        if ((k ==99)): 
            # 3a. Select the ROI which u want to track
            bbox = cv2.selectROI(frame, False)
            # 3b. Initialize tracker with first frame and bounding box
            ok = tracker.init(frame, bbox)

            if(ok):
                break

    while True:

        # 4. Read the next frame
        ok, frame = video.read()
        
        if not ok:
            break
        
        # Start timer
        timer = cv2.getTickCount()

        # 5. Update tracker
        ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

        # 6a. Draw bounding box if succesfully tracked
        if ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        
        # 6b. Otherwise report a tracking failure    
        else :
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

        # Display result
        cv2.imshow("Tracking", frame)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

if __name__ == '__main__' :
    main()
