import math
import numpy as np
import cv2
from ..config import config

class Tracker:
    def __init__(self):
        print("Initialized Object of signTracking class")
        # State Variables
        self.mode = "Detection"
        self.Tracked_class = 0
        # Proximity Variables
        self.known_centers = []
        self.known_centers_confidence = []
        self.known_centers_classes_confidence = []
        # Init Variables
        self.old_gray = 0
        self.p0 = []
        # Draw Variables
        self.mask = 0
        self.color = np.random.randint(0, 255, (100, 3))

    # Variables shared across instances of class
    max_allowed_dist = 100 # allowed dist between two detected Roi's to be considered same object
    feature_params = dict(maxCorners=100,qualityLevel=0.3,minDistance=7,blockSize=7)
    lk_params = dict(winSize=(15, 15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,10, 0.03))  

    def Distance(self,a,b):
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
        
    def init_tracker(self,label,gray,frame_draw,startP,endP,mask_to_track = None):

        if mask_to_track is None:
            sign_mask = np.zeros_like(gray)
            sign_mask[startP[1]:endP[1],startP[0]:endP[0]] = 255
            self.p0 = cv2.goodFeaturesToTrack(gray,mask= sign_mask,**self.feature_params)
        else:
            self.p0 = cv2.goodFeaturesToTrack(gray,mask= mask_to_track,**self.feature_params)
        
        if not config.Training_CNN:
            self.mode = "Tracking"
        self.Tracked_class = label
        self.old_gray = gray
        self.mask = np.zeros_like(frame_draw)

    def track(self,gray,frame_draw):
        p1,st,_ = cv2.calcOpticalFlowPyrLK(self.old_gray,gray,self.p0,None,**self.lk_params)

        # 5a. If no flow, look for new points
        if ( (p1 is None) or ( len(p1[st == 1])<3 ) ):
        #if p1 is None:
            self.mode = "Detection"
            self.mask = np.zeros_like(frame_draw)
            self.Reset()
        # 5b. If flow , Extract good points ... Update SignTrack class
        else:
            # Select good points
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
            # Draw the tracks
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = (int(x) for x in new.ravel())
                c, d = (int(x) for x in old.ravel())
                self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                frame_draw = cv2.circle(frame_draw, (a, b), 5, self.color[i].tolist(), -1)
            frame_draw_ = frame_draw + self.mask # Display the image with the flow lines
            np.copyto(frame_draw,frame_draw_) #important to copy the data to same address as frame_draw
            self.old_gray = gray.copy()  # Update the previous frame and previous points
            self.p0 = good_new.reshape(-1, 1, 2)

    def Reset(self):
        
        self.known_centers = []
        self.known_centers_confidence = []
        self.known_centers_classes_confidence = []
        self.old_gray = 0
        self.p0 = []


