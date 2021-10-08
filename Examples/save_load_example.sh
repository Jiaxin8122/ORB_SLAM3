#!/bin/bash
pathDatasetEuroc='/home/jiaxin/ws/src/ORB_SLAM3/dataset/euroc' #Example, it is necesary to change it by the dataset path

#------------------------------------
# Stereo Save Example
echo "Launching MH02 with Stereo sensor for Map Saving"
./Stereo/stereo_save ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml /home/jiaxin/ws/src/ORB_SLAM3/dataset/euroc/MH02 ./Stereo/EuRoC_TimeStamps/MH02.txt MH02_save

#------------------------------------
# Stereo Load Example
echo "Launching MH02 with Stereo sensor for Map Loading"
./Stereo/stereo_load ../Vocabulary/ORBvoc.txt ./Stereo/EuRoC.yaml /home/jiaxin/ws/src/ORB_SLAM3/dataset/euroc/MH02 ./Stereo/EuRoC_TimeStamps/MH02.txt MH02_save

