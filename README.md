# VSLAM_project
For the 3rd lib, have a reference of the [orbslam2](https://github.com/raulmur/ORB_SLAM2)

## uninstall old opencv
1. cd /home/fyl/docker-3rdparty/3rdparty/opencv-3.0.0/build_x86_64
2. sudo make uninstall

## intall opencv3.4.1 and its' contrib
1. cmake -DOPENCV_EXTRA_MODULES_PATH=/home/fyl/Downloads/software/opencv/opencv/opencv_contrib-3.4.1/modules /home/fyl/Downloads/software/opencv/opencv/opencv-3.4.1
2. sudo make -j10
3. sudo make install

## run the code
```
./testVSLAM ../data/left_image/ ../data/right_image/ 10 ../data/calib.txt
```
```
./testVSLAM ../../../VisualSLAM/data/00/image_0/  ../../../VisualSLAM/data/00/image_1/ 1000 ../../../VisualSLAM/data/00/calib.txt
```






