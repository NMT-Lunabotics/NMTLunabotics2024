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
  HudNode() : it_(nh_) {
    // Initialize subscribers and publishers
    // Get the topic names from parameters
    std::string image_input_topic, image_output_topic;
    nh_.param<std::string>("image_input_topic", image_input_topic, "/usb_cam/image_raw");
    nh_.param<std::string>("image_output_topic", image_output_topic, "/camera_hud");

    image_sub_ = it_.subscribe(image_input_topic, 1, &HudNode::imageCb, this,
                               image_transport::TransportHints("compressed"));
    image_pub_ = it_.advertise(image_output_topic, 1);

    text_sub_ = nh_.subscribe<can_convert::ArmStatus>("/arm_status", 1,
                                                      &HudNode::textCb, this);
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

    // Draw text and shapes on the video stream based on the robot arm status
    // Drawing logic remains the same as before

    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void textCb(const can_convert::ArmStatus::ConstPtr &msg) {
    arm_angle = msg->arm_angle * 3.14 / 180; // Converting degrees to radians
    bucket_angle = msg->bucket_angle * 3.14 / 180;
    // Update overlay text
    std::stringstream ss;
    ss << "Bucket Angle: " << int(-bucket_angle)
       << " deg, Arm Angle: " << int(arm_angle) << " deg";
    overlay_text = ss.str();
  }

private:
  ros::NodeHandle nh_;
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
