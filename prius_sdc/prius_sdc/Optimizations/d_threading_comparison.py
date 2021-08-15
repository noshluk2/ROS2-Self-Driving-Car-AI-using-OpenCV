from imutils.video.pivideostream import PiVideoStream
import cv2
import time

# PreRequistes
# 1: Pi cam
# 2: pip install imutils


def main():
    # Normal Approach
    cap = cv2.VideoCapture(0)# Reading directly from the camera without threading
    frame_count = 0
    norm_start = time.time()
    while(1):
        ret,img = cap.read()
        if not ret:
            break
        
        frame_count+=1
        if(frame_count==50):
            norm_end = time.time()
            print("[Profiling] Norm Loop took ",norm_end - norm_start," sec <-->  ",(1/(norm_end - norm_start)),"  FPS ")
            cap.release()
            break

    # Threaded approach
    vs = PiVideoStream((1920,1080),30).start()
    frame_count = 0
    threaded_start = time.time()
    while(1):
        img = vs.read()
        
        frame_count+=1
        if(frame_count==50):
            threaded_end = time.time()
            print("[Profiling] Threaded Loop took ",threaded_end - threaded_start," sec <-->  ",(1/(threaded_end - threaded_start)),"  FPS ")
            break    


if __name__ == '__main__':
    main()