#include <ros/ros.h>

#include <string>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/JointState.h>

class CalJob
{
public:
  CalJob(const std::string& outfile)
    : file_(outfile.c_str())
    , scene_id_(0)
  {
    if (!file_) throw std::runtime_error("Could not open outfile");
    write_headers(file_);
  }

  ~CalJob()
  {
    write_footer(file_);
  }

  void new_scene(const sensor_msgs::JointState& state)
  {
    write_scene(file_, scene_id_++, state.position);
  }

private:
  void write_headers(std::ostream& os) const
  {
    const static char* header = "---\nreference_frame: world_frame\nscenes:\n"; 
    os << header;
  }

  void write_scene(std::ostream& os, std::size_t id, const std::vector<double>& joints) const
  {
    os << "-\n";
    os << "     scene_id: " << id << '\n';
    os << "     trigger: ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER\n";
    os << "     trig_action_server: rosRobotSceneTrigger_joint_values\n";
    os << "     joint_values:\n";
    
    for (size_t i = 0; i < joints.size(); ++i)
    {
      os << "        - " << joints[i] << '\n';
    } 

    os << "     observations:\n";
    os << "     -\n";
    os << "          camera: Kinect2\n";
    os << "          target: CircleGrid5x5\n";
    os << "          roi_x_min: 0\n";
    os << "          roi_x_max: 1920\n";
    os << "          roi_y_min: 0\n";
    os << "          roi_y_max: 1080\n";
    os << "          cost_type: LinkCameraCircleTargetReprjErrorPK\n";

    return;
  }

  void write_footer(std::ostream& os)
  {
    os << "\noptimization_parameters: xx";
  }

  std::size_t scene_id_;
  std::ofstream file_;
};


static const std::string OPENCV_WINDOW = "Image window";

static void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Downsize the 1080p image
  cv::pyrDown(cv_ptr->image, cv_ptr->image);

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

//sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(params_.scan_topic,ros::Duration(WAIT_MSG_DURATION));

int main(int argc, char** argv)
{
  ros::init(argc, argv, "caljob_creator");

  ros::NodeHandle nh ("~");

  // Load options from param server
  std::string image_topic_name;
  nh.param<std::string>("image_topic", image_topic_name, "/kinect2/hd/image_color");

  std::string output_file_name;
  nh.param<std::string>("output_file", output_file_name, "caljob.yaml");

  std::string joints_topic_name;
  nh.param<std::string>("joints_topic", joints_topic_name, "abb/joint_states");

  // Handlers for images 
  image_transport::ImageTransport image_trans (nh);

  // OpenCV window scopegaurd
  struct CvScopeGaurd {
    CvScopeGaurd() { cv::namedWindow(OPENCV_WINDOW); }
    ~CvScopeGaurd() { cv::destroyWindow(OPENCV_WINDOW); }
  } scope_gaurd;

  // Create subscribers for images
  image_transport::Subscriber image_sub = image_trans.subscribe(image_topic_name, 1, imageCb);

  CalJob caljob (output_file_name);

  // Main loop
  while (ros::ok())
  {
    char c = cv::waitKey(100); // wait 100 ms
    if (c == 'q') break; // exit
    else if (c > 0)
    {
      ROS_INFO("Attempting to capture joint states");
      sensor_msgs::JointStateConstPtr joints 
          = ros::topic::waitForMessage<sensor_msgs::JointState>(joints_topic_name, ros::Duration(1.0));
      if (!joints)
      {
        ROS_ERROR("Could not capture joint states within capture timeframe");
      }
      else
      {
        caljob.new_scene(*joints);
      }
    }

    ros::spinOnce();
  }
  ROS_INFO("Terminating caljob process");
  return 0;
}