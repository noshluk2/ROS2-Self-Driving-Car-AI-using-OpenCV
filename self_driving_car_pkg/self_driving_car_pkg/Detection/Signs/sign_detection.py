import cv2
import numpy as np
import os

# Training_Variables
save_dataset = True
iter = 0
saved_no = 0
# Classification Imports, Variables N Function (5a)
from tensorflow.keras.models import load_model
import tensorflow as tf
model_loaded = False
model = 0
sign_classes = ["speed_sign_30","speed_sign_60","speed_sign_90","stop","left_turn","No_Sign"] # Trained CNN Classes
from ..of_tracking import Tracker
from tensorflow.keras.utils import to_categorical
sign_tracker = Tracker()


def image_forKeras(image):
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)# Image everywher is in rgb but Opencv does it in BGR convert Back
    image = cv2.resize(image,(30,30)) #Resize to model size requirement
    image = np.expand_dims(image, axis=0) # Dimension of model is [Batch_size, input_row,inp_col , inp_chan]
    return image

def sign_det_n_track(gray,frame,frame_draw):

    if (sign_tracker.mode=="Detection"):

        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=250,param2=30,minRadius=10,maxRadius=100)

        # 4a. Checking if any circular regions were localized
        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 4b. Looping over each localized circle and extract its center and radius
            for i in circles[0,:]:
                center =(i[0],i[1])
                radius = i[2]+5
                # 4c. Extracting Roi from localized circle
                try:
                    startP = (center[0]-radius ,center[1]-radius )
                    endP   = (center[0]+radius ,center[1]+radius )

                    localized_sign = frame[startP[1]:endP[1],startP[0]:endP[0]]
                    # 4d. Indicating localized potential sign on frame and also displaying seperatly
                    cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,165,255),1) # draw the outer circle
                    cv2.circle(frame_draw,(i[0],i[1]),2,(0,0,255),3) # draw the center of the circle
                    cv2.imshow("ROI",localized_sign)
                    sign = sign_classes[np.argmax(model(image_forKeras(localized_sign)))]
                    # 5d. Check if Classified Region is a Sign
                    if(sign != "No_Sign"):
                        # 4i. Display Class                      
                        cv2.putText(frame_draw,sign,(endP[0]-80,startP[1]-10),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),1)
                        # 4d. Display confirmed sign in green circle
                        cv2.circle(frame_draw,(i[0],i[1]),i[2],(0,255,0),2) # draw the outer circle

                        match_found,match_idx = sign_tracker.MatchCurrCenter_ToKnown(center)
                        # 4f. If match found , Increment ... known centers confidence 
                        if match_found:
                            sign_tracker.known_centers_confidence[match_idx] += 1
                            sign_tracker.known_centers_classes_confidence[match_idx][sign_classes.index(sign)] += 1
                            # 4g. Check if same sign detected 3 times , If yes initialize OF Tracker
                            if(sign_tracker.known_centers_confidence[match_idx] > 4):
                                max_value = max(sign_tracker.known_centers_classes_confidence[match_idx])
                                max_index = sign_tracker.known_centers_classes_confidence[match_idx].index(max_value)
                                print("This sign we are about to track is most likely ( " + str(max_value)," ) a "+sign_classes[max_index])
                                sign_tracker.init_tracker(sign_classes[max_index],gray,frame_draw,startP,endP)
                        # 4h. Else if Sign detected First time ... Update signs location and its detected count
                        else:
                            sign_tracker.known_centers.append(center)
                            sign_tracker.known_centers_confidence.append(1)
                            sign_tracker.known_centers_classes_confidence.append(list(to_categorical(sign_classes.index(sign),num_classes=6)))

                    # Saving dataset
                    if save_dataset:
                        if  (sign =="speed_sign_30"):
                            class_id ="/0"
                        elif(sign =="speed_sign_60"):
                            class_id ="/1"
                        elif(sign =="speed_sign_90"):
                            class_id ="/2"
                        elif(sign =="stop"):
                            class_id ="/3"
                        elif(sign =="left_turn"):
                            class_id ="/4"
                        else:
                            class_id ="/5"

                        global iter,saved_no
                        iter = iter + 1 
                        # save evert 5 th image
                        if ((iter%5) ==0):
                            saved_no = saved_no + 1
                            img_dir = os.path.abspath("self_driving_car_pkg/self_driving_car_pkg/data/live_dataset") + class_id
                            img_name = img_dir + "/" + str(saved_no)+".png"
                            if not os.path.exists(img_dir):
                                os.makedirs(img_dir)
                            cv2.imwrite(img_name , localized_sign)

                except Exception as e:
                    print(e)
                    pass
            cv2.imshow("Signs Localized",frame_draw)
    else:
        # Tracking Mode
        sign_tracker.track(gray,frame_draw)
        # 4i. Display Class
        #cv2.putText(frame_draw,sign_tracker.Tracked_class,(20,60),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),1)
        

def detect_signs(frame,frame_draw):
    # 5b. Detection (Classification) [Loading CNN Model]
    global model_loaded
    if not model_loaded:
        print(tf.__version__)#2.4.1
        print("************ LOADING MODEL **************")
        # 1. Load CNN model
        global model
        model = load_model(os.path.join(os.getcwd(),"self_driving_car_pkg/self_driving_car_pkg/data/saved_model_5_Sign.h5"),compile=False)
        # summarize model.
        model.summary()
        model_loaded = True

    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    sign_det_n_track(gray,frame,frame_draw)

    return sign_tracker.mode,sign_tracker.Tracked_class
