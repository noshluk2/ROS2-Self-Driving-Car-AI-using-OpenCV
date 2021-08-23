import cv2
import numpy as np
import math
import os



class SignTracking:

    def __init__(self):
        #Variables created inside __init__ (and all other method functions)
        # and prefaced with self.
        print("Initialized Object of signTracking class")

    #Variable set outside __init__ belong to the class. 
    # They're shared by all instances.
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
    Tracked_ROI=0
    CollisionIminent = False

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
    
    def EstimateTrackedRect(self,im_src,pts_src,pts_dst):
        Tracking = "Tracking"
        im_dst = np.zeros_like(im_src)
        if(len(pts_src)>=3):
            pts_src = pts_src[0:3][:]
            pts_dst = pts_dst[0:3][:]

            M = cv2.getAffineTransform(pts_src, pts_dst)
            im_dst = cv2.warpAffine(im_src, M ,(im_dst.shape[1],im_dst.shape[0]),flags=cv2.INTER_CUBIC)
            
            img_dst_2 = np.zeros_like(im_dst)

            kernel = np.ones((2,2), dtype=np.uint8)
            closing = cv2.morphologyEx(im_dst, cv2.MORPH_CLOSE, kernel)
            
            cnts = cv2.findContours(closing, cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_NONE )[1]
            cnt = max(cnts, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(cnt)
            if ( abs( (x+w) - im_src.shape[1] ) < (0.25*im_src.shape[1]) ):
                self.CollisionIminent = True
                
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            cv2.drawContours(img_dst_2,[box],0,255,-1)

            #https://stackoverflow.com/questions/39371507/image-loses-quality-with-cv2-warpperspective
            # Smoothing by warping is caused by interpolation
            #im_dst = cv2.warpAffine(im_src, M ,(im_dst.shape[1],im_dst.shape[0]))

            max_value=np.amax(im_dst)
            # applying Otsu thresholding
            # as an extra flag in binary 
            # thresholding     
            #ret, thresh1 = cv2.threshold(im_dst, 255, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)     
            ret, thresh1 = cv2.threshold(im_dst, max_value-25, 255, cv2.THRESH_BINARY)     
            #im_dst=thresh1
            # the window showing output image         
            # with the corresponding thresholding         
            # techniques applied to the input image    
            #cv2.imshow('Otsu Threshold', thresh1)         
            #cv2.imshow('img_dst_2', img_dst_2)         
        else:
            print("Points less then 3, Error!!!")
            #cv2.waitKey(0)
            Tracking = "Detection"
            # Set Img_dst_2 to Already saved Tracked Roi One last Time
            img_dst_2 = self.Tracked_ROI
            self.CollisionIminent = False # Reset

        return im_dst,img_dst_2,Tracking

    def Track(self,frame,frame_draw):

        Temp_Tracked_ROI = signTrack.Tracked_ROI
        # 4a. Convert Rgb to gray
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        Text2display = "OpticalFlow ( " + self.mode + " )"
        # Localizing Potetial Candidates and Classifying them in SignDetection
        cv2.putText(frame_draw,Text2display,(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        # 4b. Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, self.p0, None,**self.lk_params)
        
        # 4c. If no flow, look for new points
        if p1 is None:
            self.mode = "Detection"
            self.mask = np.zeros_like(frame_draw)
            self.Reset()

        # 4d. If points tracked, Display and Update SignTrack class    
        else:
            # Select good points
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]

            self.Tracked_ROI,Temp_Tracked_ROI,self.mode = self.EstimateTrackedRect(self.Tracked_ROI,good_old,good_new)
            # Draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = (int(x) for x in new.ravel())
                c, d = (int(x) for x in old.ravel())
                self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                frame_draw = cv2.circle(frame_draw, (a, b), 5, self.color[i].tolist(), -1)
            frame_draw_ = frame_draw + self.mask# Display the image with the flow lines
            np.copyto(frame_draw,frame_draw_)#important to copy the data to same address as frame_draw   
            self.old_gray = gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)
        
        return Temp_Tracked_ROI

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

        signTrack.Tracked_ROI,signTrack.mode = SignTracking.EstimateTrackedRect(signTrack.Tracked_ROI,good_old,good_new)
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
    cap = cv2.VideoCapture(os.path.abspath("data/vids/Ros2/ishara+turning.avi"))

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
            cv2.imshow("Tracked_ROI",signTrack.Tracked_ROI)
        # 2. Display frame and wait for keypress
        cv2.imshow("frame",frame_draw)
        k = cv2.waitKey(20)

        # 3. If 'c' is pressed Proceed to intialize tracker
        if ((k ==99)):
            # 3a. Select the ROI which u want to track
            r = cv2.selectROI("SelectROI",img)


            # 3b. Convert Rgb to gray
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # 3c. creating ROI mask
            circle_mask = np.zeros_like(gray)
            circle_mask[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = 255
            signTrack.Tracked_ROI = circle_mask
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