//
// Created by omar on 7/12/19.
//

//[HOW TO COMMENT doxygen-clion]: https://stackoverflow.com/questions/29449340/clion-auto-documenting-functions-classes

#pragma once

#include <cmath>
#include <iostream>  //SOLO TEST
#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core.hpp>
#include <mrpt/vision/CDifodo.h>
#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <time.h>


class CVDifodo : public mrpt::vision::CDifodo {

public:
  CVDifodo();

  ~CVDifodo();

  /**
   * Initialize all values needed by the DifOdo algorithm.
   * All values related to the depth strema (resolution, FPS, FOV of the camera...)
   * If downsampling is required (if our camera resolution is too high and we cannot change the stream
   * a downsampling can be made for the input images.
   * The values are loaded within the code. (TODO: Change to an xml).
   * This option is useful for testing purposes since on an IDE you cannot use roslaunch and load a launch file.
   * Execute the binary file with option --use_inner_config to use this method instead the loadConfiguration()
   */
  void loadInnerConfiguration();

  /**
   * Loads the same values as loadInnerConfiguration but from the ros parameter server. Usually these parameters will
   * be defined in a launch.file
   */
  void loadConfiguration();

  /**
   * Loads the frame in the algorithm, making it ready to be used.
   * @param depth_frame
   */
  void loadFrame(cv::Mat &depth_frame);

  /**
   * Executes one iteration of the DIFODO algorithm, taking a new depth image if available and processing the
   * odometry.
   */
  void execute_iteration();

  /**
   * Returns the camera pose displacement between the old pose(last known pose - 1) and the cam_pose(last known pose)
   * @return
   */
  inline mrpt::poses::CPose3D getDisplacementPose() { return cam_pose_displacement; }

  /**
   * The axis in SD-SLAM and in DIFODO are in the same position, they are aligned. Although they are rotated from
   * one to the other. The relation between SD-SLAM axis and DIFODO is the following:
   * SD-SLAM = DIFODO
   *       x        y
   *       y        z
   *       z        x
   * This can be translated as a rotation of 90º in Zdifodo and then a 90º rotation in the Xdifodo. With this
   * transformation both systems have the axis aligned.
   * This method applies that transformation to the difodo pose, so it can be used by SD-SLAM later.
   * @return
   */
  mrpt::poses::CPose3D getDisplacementPoseInSDSLAMCoords();

private:

  /**
   * Loads the depth frame to the inner mrpt matrix.
   */
  void loadFrame() override;

  /**
   * Resolution of the images that comes through the subscribed topic in their original resolution
   */
  int rows_orig;
  int cols_orig;

  /**
   * This value represents the scale for the depth image. Depth images doesnt comes in meters, usually they come as
   * uint16 integers with a scale of 1000/1. in this example a value of 1000 would be 1 meter. 1510 would be 1.512m.
   * The DIFODO algorithm works in meters so we have to know the scale of the dataset first.
   */
  int depth_pixel_scale;

  /**
   * The minimum value allowed for the depth images. Values lower that this will be filter out. Set to 0
   */
  double min_depth_value_filter;

  /**
   * The maximum value allowed for the depth images. Values higher that this will be filter out. Set to 0
   */
  double max_depth_value_filter;

  /**
   * The finest resolution of the course to fine pyramid. Can be less than the depth image used but in the experiments
   * Ive made, results are bad when using a different resolution than the depth image after downsampling.
   * Because of this, this value is FIXED to the resolution of the depth images downsampled.
   */
  int rows_ctf;
  int cols_ctf;

  /**
   * Your camera FOV (Field of View) in degrees. If no values are provided the default values for realsense D435 will
   * be used.
   * REALSENSE DEPTH FOV (PAGE 57) index 4.3-4.6 -> https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf
   * Get the FOV of the realsense, the real one (probably with the program that allows to check calibration we also
   * get the real value of the FOV-> https://github.com/IntelRealSense/librealsense/issues/2141
   */
  float fovh_degrees;
  float fovv_degrees;

  /**
   * The cv mat that contains the depth image.
   *
   */
  cv::Mat depth_opencv_mat;

  /**
   * The displacement between the new_pose and the old_pose in the old_pose reference coordinate system.
   */
  mrpt::poses::CPose3D cam_pose_displacement;

  /**
   * Converts from a opencv like matrix to a mprt CMatrixFloat, which is the default depth structure used
   * by the DIFODO algorithm
   * @param depthCVMat
   * @param depthMRPTMat
   */
  void cvMatToMRPTMat(const cv::Mat &depthCVMat, mrpt::math::CMatrixFloat &depthMRPTMat);
};
