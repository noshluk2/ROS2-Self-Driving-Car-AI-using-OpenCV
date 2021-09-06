#!/bin/sh

# Author : Haider Abbasi
# Script follows here:
# Title Traffic Light Training batch script!
if [ $# -eq 0 ]; then
    echo 
    echo "############# ERROR #############"
    echo "> Path/to/OpenCV/bin not provided"
    echo "############# ERROR #############"

    exit 1
fi

OPENCV_PATH=$1
echo "OPENCV_BIN_PATH =  $OPENCV_PATH"
echo 
echo "########################################################"
echo "#### [Step - 1] : Generate neg.txt from Training/Negative"
echo "########################################################"

echo 
python -c "import utils;utils.generate_negative_description_file('Negative')"
echo 
read -p "Press enter to continue...."
echo

echo 
echo "###################################################################"
echo "#### [Step - 2] : Annotating Positive images from Training/Positive"
echo "###################################################################"

echo 
$OPENCV_PATH/opencv_annotation.exe -a=pos.txt -i=Positive
echo 
read -p "Press enter to continue..."
echo

echo "#######################################################################"
echo "#### [Step - 3] : Generating Positive Samples from the annotations ####"
echo "####### INFO: Add -show to display generated positive samples ########"
echo "#######################################################################"

echo 
$OPENCV_PATH/opencv_createsamples.exe -info pos.txt -w 72 -h 24 -num 1000 -vec pos.vec
echo 
read -p "Press enter to continue..."

echo
echo "##################################################"
echo "#### [Step - 4] : Training Haar Cascade ####"
echo "##################################################"
mkdir Cascades
echo 
$OPENCV_PATH/opencv_traincascade.exe -data Cascades/ -vec pos.vec -bg neg.txt -precalcValBufSize 6000 -precalcIdxBufSize 6000 -numPos 200 -numNeg 400 -numStages 8 -w 72 -h 24 -maxFalseAlarmRate 0.08
echo 
