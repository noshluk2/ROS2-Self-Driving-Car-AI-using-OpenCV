import cv2
import os
from Detection.Signs.a_Localization.TLD import detect_TrafficLight
from Detection.Signs.a_Localization.UsingHough import detect_Circles

def main():
    #cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/NEW_full_track_tour.mp4"))
    vid_Path=os.path.abspath("self_driving_car_pkg/data/vids/ishara_turning.avi").replace('\\', '/')
    cap = cv2.VideoCapture(vid_Path)
    waitTime = 0
    while(1):
        ret,img = cap.read()
        if ret:
            img = cv2.resize(img,(320,240))
        else:
            break

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   TESTING   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
        frame_draw = img.copy()
        detect_TrafficLight(img,frame_draw)
        #detect_Circles(img,frame_draw)


        cv2.imshow("Frame",img)
        k = cv2.waitKey(waitTime)
        if k==27:
            break

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   TESTING   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


if __name__ == '__main__':
	main()