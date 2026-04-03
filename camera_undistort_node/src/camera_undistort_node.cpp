#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

class CameraUndistortNode
{
public:
    CameraUndistortNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : it_(nh), map_initialized_(false)
    {
        loadParameters(pnh);

        K_ = (cv::Mat_<double>(3, 3) <<
              fx_, 0.0, cx_,
              0.0, fy_, cy_,
              0.0, 0.0, 1.0);

        D_ = (cv::Mat_<double>(4, 1) <<
              dist_coeffs_[0],
              dist_coeffs_[1],
              dist_coeffs_[2],
              dist_coeffs_[3]);

        image_sub_ = it_.subscribe(input_topic_, 1, &CameraUndistortNode::imageCallback, this);
        image_pub_ = it_.advertise(output_topic_, 1);
        info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(output_info_topic_, 1);

        initRectifyMap();

        ROS_INFO("Fisheye undistort node started.");
        ROS_INFO_STREAM("Input topic: " << input_topic_);
        ROS_INFO_STREAM("Output topic: " << output_topic_);
        ROS_INFO_STREAM("Output camera info topic: " << output_info_topic_);
    }

private:
    void loadParameters(ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("input_topic", input_topic_, std::string("/usb_cam/image_raw"));
        pnh.param<std::string>("output_topic", output_topic_, std::string("/usb_cam/image_rect"));
        pnh.param<std::string>("output_camera_info_topic", output_info_topic_, std::string("/usb_cam/camera_info_rect"));

        pnh.param<int>("image_width", image_width_, 1920);
        pnh.param<int>("image_height", image_height_, 1200);

        pnh.param<double>("fx", fx_, 750.6022573008312);
        pnh.param<double>("fy", fy_, 752.160192071341);
        pnh.param<double>("cx", cx_, 959.2247459250367);
        pnh.param<double>("cy", cy_, 581.7438043416802);

        std::vector<double> dist_coeffs_default = {
            0.3513712123496613,
            0.17902195708944021,
            -0.28688916527785197,
            0.11284247068393805
        };
        pnh.param<std::vector<double>>("dist_coeffs", dist_coeffs_, dist_coeffs_default);

        pnh.param<double>("balance", balance_, 1.0);
        pnh.param<bool>("use_same_k_for_output", use_same_k_for_output_, false);

        if (image_width_ <= 0 || image_height_ <= 0)
        {
            ROS_FATAL("image_width and image_height must be positive.");
            ros::shutdown();
            return;
        }

        if (dist_coeffs_.size() != 4)
        {
            ROS_FATAL("dist_coeffs size must be 4 for OpenCV fisheye model.");
            ros::shutdown();
            return;
        }

        ROS_INFO_STREAM("Loaded camera intrinsics:");
        ROS_INFO_STREAM("  image_width: " << image_width_);
        ROS_INFO_STREAM("  image_height: " << image_height_);
        ROS_INFO_STREAM("  fx: " << fx_);
        ROS_INFO_STREAM("  fy: " << fy_);
        ROS_INFO_STREAM("  cx: " << cx_);
        ROS_INFO_STREAM("  cy: " << cy_);
        ROS_INFO_STREAM("  dist_coeffs: ["
                        << dist_coeffs_[0] << ", "
                        << dist_coeffs_[1] << ", "
                        << dist_coeffs_[2] << ", "
                        << dist_coeffs_[3] << "]");
        ROS_INFO_STREAM("  balance: " << balance_);
        ROS_INFO_STREAM("  use_same_k_for_output: " << (use_same_k_for_output_ ? "true" : "false"));
    }

    void initRectifyMap()
    {
        cv::Size image_size(image_width_, image_height_);

        cv::Mat new_K;
        if (use_same_k_for_output_)
        {
            new_K = K_.clone();
        }
        else
        {
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                K_,
                D_,
                image_size,
                cv::Mat::eye(3, 3, CV_64F),
                new_K,
                balance_,
                image_size,
                1.0
            );
        }

        new_K_ = new_K.clone();

        cv::fisheye::initUndistortRectifyMap(
            K_,
            D_,
            cv::Mat::eye(3, 3, CV_64F),
            new_K_,
            image_size,
            CV_32FC1,
            map1_,
            map2_
        );

        map_initialized_ = true;

        ROS_INFO_STREAM("Rectification map initialized.");
        ROS_INFO_STREAM("Original K=\n" << K_);
        ROS_INFO_STREAM("New K=\n" << new_K_);
    }

    sensor_msgs::CameraInfo buildCameraInfo(const std_msgs::Header& header)
    {
        sensor_msgs::CameraInfo info;
        info.header = header;
        info.width = image_width_;
        info.height = image_height_;

        // 输出的是已经去畸变后的图像，因此给下游一个理想针孔模型
        info.distortion_model = "plumb_bob";
        info.D = {0.0, 0.0, 0.0, 0.0, 0.0};

        info.K = {
            new_K_.at<double>(0, 0), 0.0,                     new_K_.at<double>(0, 2),
            0.0,                     new_K_.at<double>(1, 1), new_K_.at<double>(1, 2),
            0.0,                     0.0,                     1.0
        };

        info.R = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        info.P = {
            new_K_.at<double>(0, 0), 0.0,                     new_K_.at<double>(0, 2), 0.0,
            0.0,                     new_K_.at<double>(1, 1), new_K_.at<double>(1, 2), 0.0,
            0.0,                     0.0,                     1.0, 0.0
        };

        return info;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        if (!map_initialized_)
        {
            ROS_WARN_THROTTLE(1.0, "Rectify map not initialized yet.");
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
            return;
        }

        cv::Mat undistorted;
        cv::remap(cv_ptr->image, undistorted, map1_, map2_, cv::INTER_LINEAR);

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = msg->encoding;
        out_msg.image = undistorted;

        image_pub_.publish(out_msg.toImageMsg());

        sensor_msgs::CameraInfo info_msg = buildCameraInfo(msg->header);
        info_pub_.publish(info_msg);
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher info_pub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string output_info_topic_;

    int image_width_;
    int image_height_;

    double fx_, fy_, cx_, cy_;
    std::vector<double> dist_coeffs_;
    double balance_;
    bool use_same_k_for_output_;

    cv::Mat K_;
    cv::Mat D_;
    cv::Mat new_K_;
    cv::Mat map1_;
    cv::Mat map2_;
    bool map_initialized_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_undistort_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CameraUndistortNode node(nh, pnh);
    ros::spin();
    return 0;
}