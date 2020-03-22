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

#include <fstream>
#include <iostream>
#include <time.h>


class CROSDifodo : public mrpt::vision::CDifodo {

public:
    CROSDifodo();
    ~CROSDifodo();

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
     * Starts the Difodo algorithm in a new thread that consumes the depth images coming from ROS
     */
    void start();

    /**
     * Send a cancel flag to the thread and joins it until it finish.
     */
    void cancel();

    /**
     * Loads the depth frame from the queue so DIFODO can used it.
     */
    void loadFrame() override;

    /**
     * Executes one iteration of the DIFODO algorithm, taking a new depth image if available and processing the
     * odometry.
     */
    void execute_iteration();

private:
    /**
     * A ros nodehandle to create publishers and subscribers
     */
    ros::NodeHandle nh;

    /**
     * The subscriber to the depth images.
     */
    image_transport::Subscriber it_sub_depth;
    //    image_transport::ImageTransport it; //TODO: Try to make it work maybe?

    /**
     * The publisher for the odometry message as in http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
     */
    ros::Publisher odom_publisher;

    /**
     * This publish the transformation required between frames (coordinate systems)
     */
    tf::TransformBroadcaster odom_broadcaster;

    /**
     * The topic from ROS to suscribe and get depth images from
     */
    std::string input_depth_topic;

    /**
     * ROS output topic to publish pose estimation from DIFODO
     */
    std::string output_odom_topic;

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
     * The encoding of the ros message. Following REP 118 https://www.ros.org/reps/rep-0118.html this can be either
     * 16bit or 32bit. "16UC1" and "32FC1". Check sensor_msgs/image_encoding.h to see all available encodings.
     */
    std::string pixel_encoding;

    /**
     * The objective frame rate that the algorithm will work at. This will be the maximum work rate but, due to hardware
     * limitations (sometimes when hardware cannot handle it), the real execution rate could be less. A warning message
     * will be shown when this happens.
     * Having a higher value than the camera_fps will just make the algorithm wait, so it doesnt make sense to set a
     * higher value, also is not an error.
     * NOTE: the DIFODO algorithm has a this->execution_time that measures the execution of DIFODO.
     */
    int objective_fps;

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
     * The pointer to a cv::bridge image. It will contain the information for the new images that arrived from the
     * ros subscriber thread.
     *
     */
    cv_bridge::CvImagePtr cv_copy_ptr;

    /**
     * Variable just to know that we are on the first iteration or not
     */
    bool first_iteration;

    /**
     * This flag lets the consumer thread know if the depth_image has already being process or has not being processed.
     */
    bool is_new_frame_available;

    /**
     * A mutex to protect the depth frame.
     */
    std::mutex frame_mutex;

    /**
     * When set to true this flag is used to stop the threads running within the class
     */
    bool is_cancel;

    /**
     * The thread that executes the Difodo algorithm in a loop until is cancel
     */
    std::thread difodo_thread;

    /**
     * Measures the time between the end of an iteration of the difodo algorithm and the next time that we reach the end
     * of the next iteration. Is not the time of the execution of difodo but the time between iterations which also
     * depends on the frequency that we get images. It will match the frequency of images (in time) when the DIFODO
     * algorithm can work at a higher frequency that the images and it will be less when DIFODO is processing at lower
     * frequencies compare to the images arriving. The work frequency can be extracted from this time by
     * 1000/working_time.
     */
//    double working_time;

    /**
     * Time to store the time when a certain part of the code was reach.
     */
    double last_execution_time;

    /**
     * Time to store the time of the current ros depth image message.
     */
    ros::Time current_depth_image_time;

    /**
     * Time to store the time of the last ros depth image message.
     */
    ros::Time last_depth_image_time;

    /**
     * If true this will use the depth images timestamp for the odometry timestamp message that is being publish instead
     * of the current timestamp. Useful when we are using a dataset with a groundtruth that has the old timestamps
     * (like RGBD TUM dataset)
     */
    bool use_depth_images_timestamp;

    /**
     * Executes the difodo algorithm in a loop until is cancelled
     */
    void run_difodo();

    /**
     * This method is executed when the user wants to run at a frequency lower than the FPS of the depth images are
     * being publish. To do so it checks if there is time left to sleep until the required execution time is reach and
     * sleep for that amount of time. If is going slower that the established frequency then it wont wait.
     * Returns the time that has taken this iteration to be process, without the time to sleep included.
     */
    double control_working_rate();

    /**
     * This callback stores the ros depth image in a queue (FIFO). So images can be used later by the
     * "consumer thread" loadFrame() and odometryCalculation()
     * @param msg
     */
    void callbackImageDepthRaw(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * Conversion from a ros depth image msg to a cv_bridge pointer that contains a cv::Mat. It creates a new copy of the
     * msg data since it may be needed to change the value if downsampling is applied and for the consumer to use it.
     * @param msg : The original ros message with the image information.
     * @param cv_copy_ptr : The pointer where the data will be copied.
     */
    void rosMsgToCvBridgePtr(const sensor_msgs::Image::ConstPtr &msg, cv_bridge::CvImagePtr &cv_copy_ptr);

    /**
     * Converts from a opencv like matrix (cv_bridge) to a mprt CMatrixFloat, which is the default depth structure used
     * by the DIFODO algorithm
     * @param cv_ptr
     * @param depth_image
     */
    void cvBrigdeToMRPTMat(const cv_bridge::CvImagePtr &cv_ptr, mrpt::math::CMatrixFloat &depth_image);

    /**
     * Unused currently, but it may be useful someday.
     * @param msg
     */
    void rosMsgToMRPTMat(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * This will create the transform over tf. It is not enough to publish the odometry messages but we also has to
     * specify and publish (or broadcast) the transformations between frames. What is the meaning of this?
     * Usually we have got different frames (or coordinates references systems), in this case we got "odom" and
     * "base_link" which are common names used in ROS. "odom" is usually referred to the world or general frame and
     * "base_link" is the robot coordinate system. So when we express the transformation between the odom (which is
     * static, we could think of it as the map, which is another default frame used usually in ROS) we must use the pose
     * estimation to define base_link in reference to odom.
     * @param pos_x
     * @param pos_y
     * @param pos_z
     * @param roll
     * @param pitch
     * @param yaw
     * @param current_time
     * @return
     */
    static geometry_msgs::TransformStamped createTransformStampedMsg(double pos_x, double pos_y, double pos_z,
                                                                          double roll, double pitch, double yaw,
                                                                          ros::Time current_time);

    /**
     * Creates a ros odometry message from a pose estimation of DIFODO.
     * @param pos_x
     * @param pos_y
     * @param pos_z
     * @param roll
     * @param pitch
     * @param yaw
     * @return A ros Odometry message
     */
    nav_msgs::Odometry createOdometryMsg(double pos_x, double pos_y, double pos_z,
                                         double roll, double pitch, double yaw, ros::Time current_time);

    /**
     * Publish the Odometry information (messages and transformations between frames)
     * @param pos_x
     * @param pos_y
     * @param pos_z
     * @param roll
     * @param pitch
     * @param yaw
     */
    void publishOdometryMsgs(double pos_x, double pos_y, double pos_z,
                                         double roll, double pitch, double yaw);
};
