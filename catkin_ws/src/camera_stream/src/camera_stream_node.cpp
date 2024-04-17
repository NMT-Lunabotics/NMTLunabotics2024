#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <theora_image_transport/Packet.h>  // Required for Theora decoding

class CameraStreamNode {
public:
    CameraStreamNode()
    : it_(nh_) {
        // Initialize subscribers and publishers
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
            &CameraStreamNode::imageCb, this, image_transport::TransportHints("theora"));
        image_pub_ = it_.advertise("/camera_hud", 1);

        text_sub_ = nh_.subscribe<std_msgs::String>("/text_topic", 1, &CameraStreamNode::textCb, this);

        overlay_text_ = "Default Text";
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // Convert Theora Image to CV Image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw text on the video stream
        cv::putText(cv_ptr->image, overlay_text_, cv::Point(30, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }

    void textCb(const std_msgs::String::ConstPtr& msg) {
        overlay_text_ = msg->data;
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber text_sub_;
    std::string overlay_text_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_stream_node");
    CameraStreamNode csNode;
    ros::spin();
    return 0;
}
