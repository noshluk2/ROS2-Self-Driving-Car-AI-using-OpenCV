# Haar Cascade Training 


**0)** *Gather Training Videos from ROS2 Simulation*
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/0.png" style="width:700px;height:100px;"><br/><br/>
</p>
<br/>

**1)** *Extracted frames from vids using following function by importing from Training/utils.py*
  ```python
from utils import extract_frames_from_batch
   extract_frames_from_batch(vids_folder)
  ```
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/1.png" style="width:600px;height:300px;"><br/><br/>
</p>
<br/>

**2)** *Manually sort frames from Training/vids/Extracted_frames into +ve and -ve and place inside Training/Positive and Training/Negative respectively.*

<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/2.png" style="width:600px;height:400px;"><br/><br/>
</p>
<br/>

**3)** *Visualize the data*
```Python
from utils import count_files_in_dirs_n_subdirs
count_files_in_dirs_n_subdirs(Path/To/Training, display_bar=True)
```
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/3.png" style="width:450px;height:300px;"><br/><br/>
</p>
<br/>

<br/>

> **Note:** **( Step 4 Onwards )** Can be done Automatically by simply running the script file .

> a) Windows_Auto_Train.bat
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/Windows_bat_run.png" style="width:750px;height:300px;"><br/><br/>
</p>

> b) Linux_Auto_Train.sh

<br/>

**4)** *Open Cmd Prompt and Navigate Inside Training directory*
```
cd path\to\Training
```
<br/>

**5)** *Generate neg.txt indicating the Negative samples directories by following command*
```python
from util.py import generate_negative_description_file
 generate_negative_description_file('Negative')
```
<br/>

<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/5.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/5b.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>

**6)** *To generate **pos.txt** file*

<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/8.png" style="width:600px;height:300px;"><br/><br/>
</p>
<br/>



* We can go one of two ways

    a) Either use few (10-15) + ve images and augment the rest of the images using opencv_createsamples.exe
    ```
    "Path_to_OpenCV"/build/install/x64/vc16/bin/opencv_createsamples -img Pos.jpg -bg neg.txt -info pos.txt -num 128 -maxxangle 0.0 -maxyangle 0.0 -maxzangle 0.3 -bgcolor 255 -bgthresh 8  -w 72 -h 24
    ```
    b) Otherwise, If you have all the positive images sorted already inside **Training/Positive/** then generate their **.txt** file by following cmd.
    ```
    "Path_to_OpenCV"/build/install/x64/vc16/bin/opencv_annotation.exe -a=pos.txt -i=Positive
    ```

<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/6.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>

<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/6b.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>

**7)** *Generate positive samples from the annotations to get a vector file by executing in cmd*
```
"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_createsamples.exe -info pos.txt -w 72 -h 24 -num 1000 -vec pos.vec
```
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/7.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>

**8)** *Configure Parameters in the following cmd and start Training!*
```

"Path_to_OpenCV"\build\install\x64\vc16\bin\opencv_traincascade.exe -data Cascades/ -vec pos.vec -bg neg.txt -precalcValBufSize 6000 -precalcIdxBufSize 6000 -numPos 200 -numNeg 400 -numStages 8 -w 72 -h 24 -maxFalseAlarmRate 0.08
```
> **Note:** Positive Samples (numPos) cannot be more than the actual samples present inside Training/Positive/


<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/8b.png" style="width:450px;height:150px;"><br/><br/>
</p>
<br/>

**9)** *Test Trained Cascade*

```Python
from utils import test_trained_cascade
test_trained_cascade("vids/xyz.avi","Cascades/trained_cascade.xml")
```
<br/>
<p align="center">
   <img src="https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV/blob/main/Images_videos/HaarCascade/Training_Steps/9.png" style="width:600px;height:300px;"><br/><br/>
</p>
<br/>

