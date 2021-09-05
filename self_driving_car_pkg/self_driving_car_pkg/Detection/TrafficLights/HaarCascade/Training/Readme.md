# Haar Cascade Training 


0) Gather Training Videos from ROS2 Simulation

1) Extracted frames from vids using following function by importing from Training/utils.py

  ```python
from util.py import extract_frames_from_batch
   extract_frames_from_batch(vids_folder)
  ```
2) Manually sort frames from Training/vids/Extracted_frames into +ve and -ve and place inside Training/Positive and Training/Negative respectively.

<br />

> **Note:** **( Step 3 Onwards )** Can be done Automatically by simply running the script file .

> a) Windows_Auto_Train.bat
 
> b) Linux_Auto_Train.sh

<br />

3) Open Cmd Prompt and Navigate Inside Training directory
```
cd path\to\Training
```

4) Generate neg.txt indicating the Negative samples directories by following command
```python
from util.py import generate_negative_description_file
 generate_negative_description_file('Negative')
```
5) Generate pos.txt (+ve desription file) by executing in cmd
```
"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_annotation.exe -a=pos.txt -i=Positive
```
6)  Generate positive samples from the annotations to get a vector file by executing in cmd
```
"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_createsamples.exe -info pos.txt -w 72 -h 24 -num 1000 -vec pos.vec
```
7) Configure Parameters in the following cmd and start Training!
```

"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_traincascade.exe -data Cascades/ -vec pos.vec -bg neg.txt -precalcValBufSize 6000 -precalcIdxBufSize 6000 -numPos 200 -numNeg 400 -numStages 8 -w 72 -h 24 -maxFalseAlarmRate 0.08
```
> **Note:** Positive Samples (numPos) cannot be more than the actual samples present inside Training/Positive/

