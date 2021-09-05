import cv2
import os

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

def extract_frames_from_batch(vids_folder, dest_path_ = None, skip_frames_ = 10):
    vids_dir = (os.path.join(vids_folder,vid_file).replace("\\","/") for vid_file in os.listdir(vids_folder) if os.path.isfile( os.path.join(vids_folder,vid_file) ) )
    for vid_dir in vids_dir:
        extract_frames_from_vid(vid_dir, dest_path = dest_path_, skip_frames = skip_frames_)

### 1) Extracted frames from vids using following function
###    importing from Training/utils.py

#            vids_folder = "Path/to/vids"
#            extract_frames_from_batch(vids_folder)
#            OUTPUT  = "%vids_folder%/Extracted_frames"

### 2) Megative description file can be generated as following

#            Neg_dir = "Path/to/vids/Training_data/Negative"
#            generate_negative_description_file(Neg_dir)
#            OUTPUT  = "Path/to/vids/Training_data/neg.txt"
