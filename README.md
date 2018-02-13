# SD-SLAM
**Authors:** [Eduardo Perdices](https://gsyc.urjc.es/~eperdices/)

SD-SLAM is a real-time SLAM library for **Monocular** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. 

# 1. License

SD-SLAM is released under a [GPLv3 license](https://github.com/eperdices/SD-SLAM/blob/master/License-gpl.txt).

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04**, but it should be easy to compile in other platforms.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
We use [Eigen3](http://eigen.tuxfamily.org) to perform matrices operations. Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## g2o (Included in extra folder)
We use modified versions of the [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. This library (which is BSD) is included in the *extra* folder.

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/eperdices/SD-SLAM
```

We provide a script `build.sh` to build *SD-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd SD-SLAM
chmod +x build.sh
./build.sh
```

This will create **libSD_SLAM.so**  at *lib* folder and the executables **monocular** and **rgbd** in *Examples* folder.

# 4. Monocular Examples

Inside `PATH_TO_SEQUENCE_FOLDER` there must be a file named ''files.txt'' with each image filename. 

```
./Examples/Monocular/monocular Examples/Monocular/X.yaml PATH_TO_SEQUENCE_FOLDER
```

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Change `X.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively.

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Change `X.yaml` to EuRoC.yaml.

## Initialization with pattern

A pattern can be emploid to create the initial map using real scale. The pattern detected is the same chessboard used for camera calibration, located inside `Examples/Calibration` folder.

To use a pattern, set `UsePattern: 1` in yaml configuration file.

# 5. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

# 6. Monocular and IMU Fusion Example

Inside `PATH_TO_SEQUENCE_FOLDER` there must be a file named ''files.txt'' with each image filename. IMU data has to be stored in a csv file where each line has a timestamp and 6 measurements (3 from gyroscope and 3 from accelerometer).

  ```
  ./Examples/Fusion/monocular_imu Examples/Monocular/X.yaml PATH_TO_SEQUENCE_FOLDER IMU_DATA_FILE.csv
  ```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Change `X.yaml` to EuRoC.yaml.
 
# 7. Processing your own sequences

You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM dataset for monocular or RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the SD-SLAM library and how to pass images to the SLAM system. RGB-D input must be synchronized and depth registered.

## Camera Calibration

We provide a tool to calibrate your camera using the calibration model of OpenCV. You need to print the pattern found inside ` Examples/Calibration` folder and take several pictures of this pattern from different points of view. Then store these pictures in a folder and execute the calibration tool.

  ```
  ./Examples/Calibration/calibration PATH_TO_IMAGES_FOLDER
  ```

Inside `PATH_TO_IMAGES_FOLDER` there must be a file named ''files.txt'' with each image filename.

You can check if the intrinsic parameters calculated are accurate checking the rectified images stored in `PATH_TO_IMAGES_FOLDER`.


# 8. Android Compilation
Read: https://github.com/taka-no-me/android-cmake

