#include <can_convert/ArmStatus.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <theora_image_transport/Packet.h> // Required for Theora decoding

class HudNode {
public:
  HudNode() : it_(private_nh_) {
    // Initialize subscribers and publishers
    // Get the topic names from parameters in the private namespace
    std::string image_in, image_out;
    private_nh_.param<std::string>("image_in", image_in, "/usb_cam/image_raw");
    private_nh_.param<std::string>("image_out", image_out, "/camera_hud");

    image_sub_ = it_.subscribe(image_in, 1, &HudNode::imageCb, this,
                               image_transport::TransportHints("compressed"));
    image_pub_ = it_.advertise(image_out, 1);

    text_sub_ = private_nh_.subscribe<can_convert::ArmStatus>(
        "/arm_status", 1, &HudNode::textCb, this);
  }

  void textCb(const can_convert::ArmStatus::ConstPtr &msg) {
    arm_angle = msg->arm_angle * 3.14 / 180;
    bucket_angle = msg->bucket_angle * 3.14 / 180;
    std::stringstream ss;
    ss << "Bucket Angle: " << int(-bucket_angle)
       << " deg, Arm Angle: " << int(arm_angle) << " deg";
    overlay_text = ss.str();
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      // Convert Theora Image to CV Image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int start_x = 500;
    int start_y = 100;
    int arm_len = 100;
    int bucket_1_len = 30;
    int bucket_2_len = 20;
    int arm_end_x = start_x + arm_len * cos(arm_angle);
    int arm_end_y = start_y + arm_len * sin(-arm_angle);
    int bucket_1_x = arm_end_x + bucket_1_len * cos(bucket_angle - arm_angle);
    int bucket_1_y = arm_end_y + bucket_1_len * sin(bucket_angle - arm_angle);
    int bucket_2_x =
        arm_end_x + bucket_2_len * cos(bucket_angle - arm_angle - 1.2);
    int bucket_2_y =
        arm_end_y + bucket_2_len * sin(bucket_angle - arm_angle - 1.2);
    cv::Point armBase(start_x, start_y);
    cv::Point armEnd(arm_end_x, arm_end_y);
    cv::Point bucket1(bucket_1_x, bucket_1_y);
    cv::Point bucket2(bucket_2_x, bucket_2_y);
    cv::Point groundStart(start_x, start_y + 2);
    cv::Point groundEnd(start_x + arm_len + bucket_1_len, start_y + 2);

    cv::line(cv_ptr->image, armBase, armEnd, cv::Scalar(0, 0, 255), 2,
             cv::LINE_8);
    cv::line(cv_ptr->image, armEnd, bucket1, cv::Scalar(0, 127, 255), 2,
             cv::LINE_8);
    cv::line(cv_ptr->image, armEnd, bucket2, cv::Scalar(0, 127, 255), 2,
             cv::LINE_8);
    cv::line(cv_ptr->image, groundStart, groundEnd, cv::Scalar(0, 255, 0), 2,
             cv::LINE_8);
    image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  ros::NodeHandle private_nh_{"~"};
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber text_sub_;
  std::string overlay_text;
  float arm_angle = 0;
  float bucket_angle = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hud_node");
  HudNode csNode;
  ros::spin();
  return 0;
}
