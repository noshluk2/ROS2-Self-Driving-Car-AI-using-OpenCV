import cv2
import os
from Detection.Signs.a_Localization.TLD import detect_TrafficLight
from Detection.Signs.a_Localization.UsingHough import detect_Circles

def main():
    vid_Path=os.path.abspath("self_driving_car_pkg/data/vids/ishara_turning.avi").replace('\\', '/')
    #vid_Path=os.path.abspath("self_driving_car_pkg/data/vids/new_ishara.avi").replace('\\', '/')
    #vid_Path=os.path.abspath("self_driving_car_pkg/data/vids/ishara_sign_conflict.avi").replace('\\', '/')
    cap = cv2.VideoCapture(vid_Path)
    waitTime = 1
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