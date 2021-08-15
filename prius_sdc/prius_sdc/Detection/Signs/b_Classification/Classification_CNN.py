import tensorflow as tf # tensorflow imported to check installed tf version
from tensorflow.keras.models import load_model # import load_model function to load trained CNN model for Sign classification
import timeit
import os # for getting absolute filepath to mitigate cross platform inconsistensies
import cv2
import time
import numpy as np
import config
import math

detected_img = 0 #Set this to current dataset images size so that new images number starts from there and dont overwrite
if config.Detect_lane_N_Draw:
    write_data = False
else:
    write_data = True
draw_detected = True
display_images = False
model_loaded = False
model = 0
sign_classes = ["speed_sign_70","speed_sign_80","stop","No_Sign"] # Trained CNN Classes


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

def image_forKeras(image):
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)# Image everywher is in rgb but Opencv does it in BGR convert Back
    image = cv2.resize(image,(30,30)) #Resize to model size requirement
    image = np.expand_dims(image, axis=0) # Dimension of model is [Batch_size, input_row,inp_col , inp_chan]
    return image

def SignDetection(gray,cimg,frame_draw,model):
    
    NumOfVotesForCircle = 40 #parameter 1 MinVotes needed to be classified as circle
    CannyHighthresh = 200 # High threshold value for applying canny
    mindDistanBtwnCircles = 100 # kept as sign will likely not be overlapping
    max_rad = 150 # smaller circles dont have enough votes so only maxRadius need to be controlled 
                    # As signs are right besides road so they will eventually be in view so ignore circles larger than said limit

    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,mindDistanBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=10,maxRadius=max_rad)

    if circles is not None:
        circles = np.uint16(np.around(circles))
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

                startP = (center[0]-radius,center[1]-radius)
                endP = (center[0]+radius,center[1]+radius)
                
                detected_sign = cimg[startP[1]:endP[1],startP[0]:endP[0]]

                if(detected_sign.shape[1] and detected_sign.shape[0]):
                    sign = sign_classes[np.argmax(model(image_forKeras(detected_sign)))]

                    if(sign != "No_Sign"):                                                          
                        cv2.putText(frame_draw,sign,(endP[0]-30,startP[1]-10),cv2.FONT_HERSHEY_DUPLEX,0.65,(0,0,255),1)

                        if draw_detected:
                            # draw the outer circle
                            cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),1)
                            # draw the center of the circle
                            cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3)

                    if write_data:
                        if  (sign =="speed_sign_70"):
                            class_id ="0/"
                        elif(sign =="speed_sign_80"):
                            class_id ="1/"
                        elif(sign =="stop"):
                            class_id ="2/"
                        else:
                            class_id ="3/"
                        img_dir = os.path.abspath("Detection/Signs/datasets/") + class_id
                        #img_name = "Detection/Signs/datasets/"+ class_id + str(detected_img)+".png"
                        img_name = img_dir + str(detected_img)+".png"
                        if not os.path.exists(img_dir):
                            os.makedirs(img_dir)
                        cv2.imwrite(img_name , detected_sign)

        
        if display_images:
            cimg_str = 'detected circles'
            cv2.imshow(cimg_str,frame_draw)
            cv2.waitKey(1)


def detect_Signs(frame,frame_draw):
    
    global model_loaded
    if not model_loaded:
        print(tf.__version__)#2.4.1
        print("************ LOADING MODEL **************")
        global model
        # load model
        model = load_model(os.path.abspath('data/saved_model.h5'),compile=False)
        # summarize model.
        model.summary()
        model_loaded = True

    # Convert Rgb to colourImg
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # Localizing Potetial Candidates and Classifying them in SignDetection
    start_signDetection = time.time()
    #cv2.putText(frame_draw,signTrack.mode,(10,10),cv2.FONT_HERSHEY_PLAIN,0.5,(255,255,255),1)
    
    SignDetection(gray.copy(),frame.copy(),frame_draw,model)
    end_signDetection = time.time()

    print("[Profiling] [ ",signTrack.mode," ] SignDetection took ",end_signDetection - start_signDetection," sec <-->  ")
    print("[Profiling] [ ",signTrack.mode," ] SignDetection took ",end_signDetection - start_signDetection," sec <-->  ",(1/(end_signDetection - start_signDetection + 0.0001)),"  FPS ")
    return signTrack.mode , signTrack.Tracked_class