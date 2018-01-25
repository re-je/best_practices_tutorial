#include <ros/ros.h>

#include <string>
#include <sstream>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <academy_msgs/SelectJoint.h>

#include <academy_lib/colors.h>

class MarkerPub{
public:
  MarkerPub();
private:
  bool handleSelectJoint(academy_msgs::SelectJoint::Request &req, academy_msgs::SelectJoint::Response &res);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  bool setJoint(const std::string &joint);

  std::string selected_joint_;
  ros:: Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  ros::ServiceServer select_joint_srv_;
  ros::Subscriber joint_states_sub_;
};

bool MarkerPub::setJoint(const std::string &joint) {
  size_t pos = joint.find("joint");

  if (pos != std::string::npos) { // if found
    selected_joint_ = joint;
    marker_.header.frame_id = std::string(joint).replace(pos, 5, "link");
    ROS_INFO("Show marker for '%s'", joint.c_str());
    return true;
  }else{
    ROS_ERROR("joint name '%s' does not contain 'joint'", joint.c_str());
    return false;
  }
}

MarkerPub::MarkerPub(){

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  if(nh_priv.hasParam("selected_joint")){
    setJoint(nh_priv.param<std::string>("selected_joint", ""));
  }

  // fill marker message
  marker_.ns = "joint_state";
  marker_.type = marker_.TEXT_VIEW_FACING;

  marker_.pose.position.x = 0.0;
  marker_.pose.position.y = 0.0;
  marker_.pose.position.z = 0.0;
  marker_.pose.orientation.x = 0.0;
  marker_.pose.orientation.y = 0.0;
  marker_.pose.orientation.z = 0.0;
  marker_.pose.orientation.w = 1.0;

  marker_.scale.x = 0.1;
  marker_.scale.y = 0.1;
  marker_.scale.z = 0.1;

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("selected_joint_marker", 1);
  select_joint_srv_ = nh.advertiseService("select_joint", &MarkerPub::handleSelectJoint, this);
  joint_states_sub_ = nh.subscribe("joint_states", 10,  &MarkerPub::jointStateCallback, this);

}
bool MarkerPub::handleSelectJoint(academy_msgs::SelectJoint::Request &req, academy_msgs::SelectJoint::Response &res) {
  res.success = setJoint(req.joint_name);
  return true;
}

void MarkerPub::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

  if(selected_joint_.empty()) return;  // nothin to do

  // check for proper input
  if (msg->name.size() != msg->position.size()) {
    ROS_WARN("JointState message is not valid");
    return;
  }

  // find selected joint name
  for(size_t i =0; i < msg->name.size(); ++i) {
      if(msg->name[i] == selected_joint_) {
        std::stringstream strs;
        strs << selected_joint_ << ": " << msg->position[i]; // fill text
        marker_.text = strs.str();
        marker_.color = academy::getColor(msg->position[i]);
        marker_.header.stamp = msg->header.stamp;
        if(marker_.header.stamp.isZero()) {
          marker_.header.stamp = ros::Time::now();
        }

        marker_pub_.publish(marker_);
        return; // jump out to prevent display warning
      }
  }
  ROS_WARN("JointState message does not contain %s", selected_joint_.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "MakerPub");
  MarkerPub marker_publisher;
  ros::spin();
  return 0;
}
