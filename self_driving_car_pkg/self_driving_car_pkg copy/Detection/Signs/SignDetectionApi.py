import tensorflow as tf # tensorflow imported to check installed tf version
from tensorflow.keras.models import load_model # import load_model function to load trained CNN model for Sign classification
import os # for getting absolute filepath to mitigate cross platform inconsistensies
import cv2
import time
import numpy as np
from ...config import config
import math

detected_img = 1000 #Set this to current dataset images size so that new images number starts from there and dont overwrite

#if config.Detect_lane_N_Draw:
#    write_data = False # not gathering data # No Training
#else:
#    write_data = True

if config.Training_CNN:
    write_data = True
else:
    write_data = False # not gathering data # No Training
draw_detected = True
model_loaded = False
model = 0
sign_classes = ["speed_sign_30","speed_sign_60","speed_sign_90","stop","left_turn","No_Sign"] # Trained CNN Classes


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
    # [NEW]: If no Sign Tracked ==> Then default is Unknown
    Tracked_class = "Unknown"
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

def image_forKeras(image):
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)# Image everywher is in rgb but Opencv does it in BGR convert Back
    image = cv2.resize(image,(30,30)) #Resize to model size requirement
    image = np.expand_dims(image, axis=0) # Dimension of model is [Batch_size, input_row,inp_col , inp_chan]
    return image

def SignDetection_Nd_Tracking(gray,cimg,frame_draw,model):

    # 3. IF Mode of SignTrack is Detection , Proceed
    if (signTrack.mode == "Detection"):
        # cv2.putText(frame_draw,"Sign Detected ==> "+str(signTrack.Tracked_class),(20,85),cv2.FONT_HERSHEY_COMPLEX,0.75,(255,255,0),2)
        NumOfVotesForCircle = 32 #parameter 1 MinVotes needed to be classified as circle
        CannyHighthresh = 250 # High threshold value for applying canny
        mindDistanBtwnCircles = 100 # kept as sign will likely not be overlapping
        max_rad = 140 # smaller circles dont have enough votes so only maxRadius need to be controlled 
                        # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit

        # 4. Detection (Localization)
        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,mindDistanBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=16,maxRadius=max_rad)

        # 4a. Detection (Localization) Checking if circular regions were localized
        if circles is not None:
            circles = np.uint16(np.around(circles))

            # 4b. Detection (Localization) Looping over each localized circle
            for i in circles[0,:]:

                center =(i[0],i[1])
                #match_found,match_idx = MatchCurrCenter_ToKnown(center,known_centers,30)
                match_found,match_idx = signTrack.MatchCurrCenter_ToKnown(center)
                #if not match_found:
                    #signTrack.known_centers.append(center)

                radius = i[2] + 5
                if (radius !=5):
                    global detected_img
                    detected_img = detected_img + 1 

                    startP = (center[0]-radius + 3 ,center[1]-radius + 3 )
                    endP   = (center[0]+radius - 3 ,center[1]+radius - 3 )
                    
                    # 4c. Detection (Localization) Extracting Roi from localized circle
                    detected_sign = cimg[startP[1]:endP[1],startP[0]:endP[0]]

                    # 4d. Detection (Classification) Classifying sign in the ROi
                    if(detected_sign.shape[1] and detected_sign.shape[0]):
                        sign = sign_classes[np.argmax(model(image_forKeras(detected_sign)))]

                        # 4e. Check if Classified Region is a Sign
                        if(sign != "No_Sign"):

                            # 4f. If match found , Increment ... known centers confidence 
                            if match_found:
                                signTrack.known_centers_confidence[match_idx] += 1

                                # 4g. Check if same sign detected 3 times , If yes initialize OF Tracker
                                if(signTrack.known_centers_confidence[match_idx] > 3):
                                    #cv2.imshow("Detected_SIGN",detected_sign)
                                    circle_mask = np.zeros_like(gray)
                                    circle_mask[startP[1]:endP[1],startP[0]:endP[0]] = 255
                                    if not config.Training_CNN:
                                        signTrack.mode = "Tracking" # Set mode to tracking
                                    signTrack.Tracked_class = sign # keep tracking frame sign name
                                    signTrack.old_gray = gray.copy()
                                    signTrack.p0 = cv2.goodFeaturesToTrack(signTrack.old_gray, mask=circle_mask, **signTrack.feature_params)
                                    signTrack.mask = np.zeros_like(frame_draw)

                            # 4h. Else if Sign detected First time ... Update signs location and its detected count
                            else:
                                signTrack.known_centers.append(center)
                                signTrack.known_centers_confidence.append(1)

                            # 4i. Display BBox and Class                      
                            cv2.putText(frame_draw,sign,(endP[0]-150,startP[1]-10),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),1)
                            if draw_detected:
                                cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),1) # draw the outer circle
                                cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3) # draw the center of the circle

                        if write_data:
                            if  (sign =="speed_sign_30"):
                                class_id ="/0/"
                            elif(sign =="speed_sign_60"):
                                class_id ="/1/"
                            elif(sign =="speed_sign_90"):
                                class_id ="/2/"
                            elif(sign =="stop"):
                                class_id ="/3/"
                            elif(sign =="left_turn"):
                                class_id ="/4/"
                            else:
                                class_id ="/5/"

                            img_dir = os.path.abspath("self_driving_car_pkg/self_driving_car_pkg/data/dataset_signs/datasets") + class_id
                            img_name = img_dir + str(detected_img)+".png"
                            if not os.path.exists(img_dir):
                                os.makedirs(img_dir)
                            cv2.imwrite(img_name , detected_sign)

            
            if (config.debugging and config.debugging_Signs):
                cv2.imshow("detected Signs",frame_draw)


    # 5. IF Mode of SignTrack is Tracking , Proceed
    else:
        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(signTrack.old_gray, gray, signTrack.p0, None,**signTrack.lk_params)
        
        # 5a. If no flow, look for new points
        if ( (p1 is None) or ( len(p1[st == 1])<3 ) ):
        #if p1 is None:
            signTrack.mode = "Detection"
            signTrack.mask = np.zeros_like(frame_draw)
            signTrack.Reset()

        # 5b. If flow , Extract good points ... Update SignTrack class
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
            frame_draw_ = frame_draw + signTrack.mask # Display the image with the flow lines
            np.copyto(frame_draw,frame_draw_) #important to copy the data to same address as frame_draw
            signTrack.old_gray = gray.copy()  # Update the previous frame and previous points
            signTrack.p0 = good_new.reshape(-1, 1, 2)
    
    if not (config.debugging and config.debugging_Signs):
        cv2.destroyWindow("detected Signs")
           

def detect_Signs(frame,frame_draw):
    """Extract required data from the traffic signs on the road

    Args:
        frame (numpy nd array): Prius front-cam view
        frame_draw (numpy nd array): for displaying detected signs

    Returns:
        string: Current mode of signtracker class
        string: detected speed sign (e.g speed sign 70)
    """    
        
    global model_loaded
    if not model_loaded:
        print(tf.__version__)#2.4.1
        print("************ LOADING MODEL **************")

        # 1. Load CNN model
        global model
        model = load_model(os.path.join(os.getcwd(),"self_driving_car_pkg/self_driving_car_pkg/data/saved_model_Ros2_5_Sign.h5"),compile=False)

        # summarize model.
        model.summary()
        model_loaded = True

    # 2. Convert Rgb to Gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Localizing Potetial Candidates and Classifying them in SignDetection
    start_signDetection = time.time()
    #cv2.putText(frame_draw,signTrack.mode,(20,10),cv2.FONT_HERSHEY_PLAIN,0.5,(255,255,255),1)
    
    SignDetection_Nd_Tracking(gray.copy(),frame.copy(),frame_draw,model)
    end_signDetection = time.time()

    print("[Profiling] [ ",signTrack.mode," ] SignDetection took ",end_signDetection - start_signDetection," sec <-->  ")
    print("[Profiling] [ ",signTrack.mode," ] SignDetection took ",end_signDetection - start_signDetection," sec <-->  ",(1/(end_signDetection - start_signDetection + 0.0001)),"  FPS ")
    return signTrack.mode , signTrack.Tracked_class