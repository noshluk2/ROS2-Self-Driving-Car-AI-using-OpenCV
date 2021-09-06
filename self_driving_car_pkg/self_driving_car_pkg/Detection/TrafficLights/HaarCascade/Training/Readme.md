# Haar Cascade Training 


0) Gather Training Videos from ROS2 Simulation

1) Extracted frames from vids using following function by importing from Training/utils.py

  ```python
from util.py import extract_frames_from_batch
   extract_frames_from_batch(vids_folder)
  ```
2) Manually sort frames from Training/vids/Extracted_frames into +ve and -ve and place inside Training/Positive and Training/Negative respectively.

3) Visualize the data 
```Python
from utils import count_files_in_dirs_n_subdirs
count_files_in_dirs_n_subdirs(Path/To/Training, display_bar=True)
```

<br />

> **Note:** **( Step 4 Onwards )** Can be done Automatically by simply running the script file .

> a) Windows_Auto_Train.bat
 
> b) Linux_Auto_Train.sh

<br />

4) Open Cmd Prompt and Navigate Inside Training directory
```
cd path\to\Training
```

5) Generate neg.txt indicating the Negative samples directories by following command
```python
from util.py import generate_negative_description_file
 generate_negative_description_file('Negative')
```
6) To generate **pos.txt** file we can go one of two ways
    
    a) Either use few (10-15) + ve images and augment the rest of the images using opencv_createsamples.exe
    ```
    "Path_to_OpenCV"/build/install/x64/vc16/bin/opencv_createsamples -img Pos.jpg -bg neg.txt -info pos.txt -num 128 -maxxangle 0.0 -maxyangle 0.0 -maxzangle 0.3 -bgcolor 255 -bgthresh 8  -w 72 -h 24
    ```
    b) Otherwise, If you have all the positive images sorted already inside **Training/Positive/** then generate their **.txt** file by following cmd.
    ```
    "Path_to_OpenCV"/build/install/x64/vc16/bin/opencv_annotation.exe -a=pos.txt -i=Positive
    ```

7) Generate positive samples from the annotations to get a vector file by executing in cmd
```
"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_createsamples.exe -info pos.txt -w 72 -h 24 -num 1000 -vec pos.vec
```
8) Configure Parameters in the following cmd and start Training!
```

"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_traincascade.exe -data Cascades/ -vec pos.vec -bg neg.txt -precalcValBufSize 6000 -precalcIdxBufSize 6000 -numPos 200 -numNeg 400 -numStages 8 -w 72 -h 24 -maxFalseAlarmRate 0.08
```
> **Note:** Positive Samples (numPos) cannot be more than the actual samples present inside Training/Positive/

9) Test Trained Cascade 

```Python
from utils import test_trained_cascade
test_trained_cascade("vids/xyz.avi","Cascades/trained_cascade.xml")
```

