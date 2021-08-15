import cv2
import numpy as np
import math
import os

detected_img = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite

write_data = False
draw_detected = True
display_images = False

class SignTracking:

    def __init__(self):
        print("Initialized Object of signTracking class")

    mode = "Detection"

    max_allowed_dist = 100
    feature_params = dict(maxCorners=100,qualityLevel=0.3,minDistance=7,blockSize=7)
    lk_params = dict(winSize=(15, 15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10, 0.03))  
    # Create some random colors
    color = np.random.randint(0, 255, (100, 3))
    known_centers = []
    known_centers_confidence = []
    old_gray = 0
    p0 = []
    Tracked_class = 0
    mask = 0

    def Distance(self,a,b):
        #return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )
        return math.sqrt( ( (float(a[1])-float(b[1]))**2 ) + ( (float(a[0])-float(b[0]))**2 ) )

    def MatchCurrCenter_ToKnown(self,center):
        match_found = False
        match_idx = 0
        for i in range(len(self.known_centers)):
            if ( self.Distance(center,self.known_centers[i]) < self.max_allowed_dist ):
                match_found = True
                match_idx = i
                return match_found, match_idx
        # If no match found as of yet return default values
        return match_found, match_idx

    def Reset(self):
        
        self.known_centers = []
        self.known_centers_confidence = []
        self.old_gray = 0
        self.p0 = []

signTrack = SignTracking()

def Track(frame,frame_draw):
    
    # 4a. Convert Rgb to gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    Text2display = "OpticalFlow ( " + signTrack.mode + " )"
    # Localizing Potetial Candidates and Classifying them in SignDetection
    cv2.putText(frame_draw,Text2display,(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
    
    # 4b. Calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(signTrack.old_gray, gray, signTrack.p0, None,**signTrack.lk_params)
    
    # 4c. If no flow, look for new points
    if p1 is None:
        signTrack.mode = "Detection"
        signTrack.mask = np.zeros_like(frame_draw)
        signTrack.Reset()

    # 4d. If points tracked, Display and Update SignTrack class    
    else:
        # Select good points
        good_new = p1[st == 1]
        good_old = signTrack.p0[st == 1]
        # Draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = (int(x) for x in new.ravel())
            c, d = (int(x) for x in old.ravel())
            signTrack.mask = cv2.line(signTrack.mask, (a, b), (c, d), signTrack.color[i].tolist(), 2)
            frame_draw = cv2.circle(frame_draw, (a, b), 5, signTrack.color[i].tolist(), -1)
        frame_draw_ = frame_draw + signTrack.mask# Display the image with the flow lines
        np.copyto(frame_draw,frame_draw_)#important to copy the data to same address as frame_draw   
        signTrack.old_gray = gray.copy()
        signTrack.p0 = good_new.reshape(-1, 1, 2)


def main():
    cap = cv2.VideoCapture(os.path.abspath("data/vids/signs_multi.mp4"))

    while(1):

        # 1. Read first frame.
        ret,img = cap.read()

        if not ret:
            break
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
        cv2.imshow("frame",frame_draw)
        k = cv2.waitKey(30)

        # 3. If 'c' is pressed Proceed to intialize tracker
        if ((k ==99)):
            # 3a. Select the ROI which u want to track
            r = cv2.selectROI("SelectROI",img)


            # 3b. Convert Rgb to gray
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # 3c. creating ROI mask
            circle_mask = np.zeros_like(gray)
            circle_mask[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = 255

            # 3d. Updating signtrack class with variables initialized
            signTrack.mode = "Tracking" # Set mode to tracking
            signTrack.Tracked_class = "Circle" # keep tracking frame sign name
            signTrack.old_gray = gray.copy()
            signTrack.p0 = cv2.goodFeaturesToTrack(signTrack.old_gray, mask=circle_mask, **signTrack.feature_params)
            signTrack.mask = np.zeros_like(frame_draw)

        elif(k==27):
            break




if __name__ == "__main__":
    main()