import cv2
import os
from matplotlib import pyplot as plt
import numpy as np

def plt_bar(Categories,Data_Amount):

    #x_pos = [i for i, _ in enumerate(Categories)]
    plt.style.use('ggplot')
    max_value_idx = Data_Amount.index(max(Data_Amount))
    for i in range(len(Data_Amount)):
        if i == max_value_idx:
            color ='green'
        else:
            color ='red'
        plt.bar(Categories[i],Data_Amount[i],0.3,color=color)
    plt.ylabel("# of data")
    plt.xlabel("Categories")
    plt.title("Dataset Spread")
    plt.show()    

def count_files_in_dirs_n_subdirs(path=None, display_bar=True):
    if path is None:
        path= os.getcwd()
        print("CWD = {} ".format(path))
    Categories = []
    Amount = []
    mn = 20
    folders = ([name for name in os.listdir(path)
                if os.path.isdir(os.path.join(path, name))]) # get all directories 
    for folder in folders:
        contents = os.listdir(os.path.join(path,folder)) # get list of contents
        if len(contents) > mn: # if greater than the limit, print folder and number of contents
            print(folder,len(contents))
            Categories.append(folder)
            Amount.append(len(contents))
            
    if display_bar:
        plt_bar(Categories,Amount)

def generate_negative_description_file(Negative_dir):
    # open the output file for writing. will overwrite all existing data in there
    Neg_txt_dir=os.path.join(os.path.dirname(Negative_dir), 'neg.txt').replace("\\","/") 
    print("Saving Negative Images dirs to => ", Neg_txt_dir)
    with open(Neg_txt_dir, 'w') as f:
        # loop over all the filenames
        for filename in os.listdir(Negative_dir):
            f.write( Negative_dir+'/' + filename + '\n')

def extract_frames_from_vid(vid_path, dest_path = None, strt_idx = None, skip_frames = 5):

    if dest_path is None:
        dest_path = os.path.join(os.path.dirname(vid_path),"Extracted_frames")
        if not os.path.isdir(dest_path):
            os.mkdir(dest_path)
            print("Creating ExtractedFrame dir!!!")
    if strt_idx is None:
        # Compute Strt_idx
        strt_idx = len([name for name in os.listdir(dest_path)])
        print("Computed Strt_idx = {} ".format(strt_idx))

    # Creating a videocapture object to acces each frame
    cap = cv2.VideoCapture(vid_path)
    iter_idx = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret:
            if(iter_idx % skip_frames == 0):
                img_name = str(strt_idx) + ".png"
                save_img_path = os.path.join(dest_path,img_name)
                print("Saving {} at {} ".format(img_name,save_img_path))
                cv2.imwrite(save_img_path, frame)
                strt_idx += 1
            iter_idx += 1
        else:
            break

def extract_frames_from_batch(vids_folder=None, dest_path_ = None, skip_frames_ = 10):
    if vids_folder is None:
        print("\nError! : No Vid directory specified \n\n##### [Function(Arguments)] = extract_frames_from_batch(vids_folder=None, dest_path_ = None, skip_frames_ = 10) #####\n")
        return
    vids_dir = (os.path.join(vids_folder,vid_file).replace("\\","/") for vid_file in os.listdir(vids_folder) if os.path.isfile( os.path.join(vids_folder,vid_file) ) )
    for vid_dir in vids_dir:
        extract_frames_from_vid(vid_dir, dest_path = dest_path_, skip_frames = skip_frames_)

def test_trained_cascade(test_vid_path=None,cascade_path=None):
    if (test_vid_path and cascade_path) is None:
        print("\nError! : No test vid directory or cascade path specified \n\n##### [Function(Arguments)] = test_trained_cascade(test_vid_path,cascade_path) #####\n")
        return
    # Class Variables
    TrafficLight_cascade_str = os.path.join(cascade_path)
    TrafficLight_cascade = cv2.CascadeClassifier()
    #-- 1. Load the cascades
    if not TrafficLight_cascade.load(cv2.samples.findFile(TrafficLight_cascade_str)):
        print('--(!)Error loading face cascade')
        exit(0)
    cap = cv2.VideoCapture(test_vid_path)
    while(cap.isOpened()):
        ret,img=cap.read()
        if ret:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            target = TrafficLight_cascade.detectMultiScale(gray)
            for (x,y,w,h) in target:
                cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0),4)

            cv2.imshow("Test video",img)
            cv2.waitKey(1)
        else:
            break
### 1) Extracted frames from vids using following function
###    importing from Training/utils.py

#            vids_folder = "Path/to/vids"
#            extract_frames_from_batch(vids_folder)
#            OUTPUT  = "%vids_folder%/Extracted_frames"

### 2) Megative description file can be generated as following

#            Neg_dir = "Path/to/vids/Training_data/Negative"
#            generate_negative_description_file(Neg_dir)
#            OUTPUT  = "Path/to/vids/Training_data/neg.txt"


### 6) Testing Trained Cascade
#            from utils import test_trained_cascade
#            test_trained_cascade("vids/xyz.avi","Cascades/trained_cascade.xml")