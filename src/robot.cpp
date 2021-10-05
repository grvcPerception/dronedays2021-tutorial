#include "dronedays2021/dronedays2021.hpp"

Robot::Robot(ros::NodeHandle n, std::string type){
  // Get info
  n.getParam("name", robotName_);
  robotType_ = type;

  // Initialize position
  std::vector<double> aux;
  n.getParam("init_position", aux);
  currentPosition_ = Eigen::Map<Eigen::Vector3d>(aux.data(), 3);

  // Initialize UWB
  n.getParam("var_uwb", var_uwb_);
  uwbDistribution_ = std::normal_distribution<double>(0.0, sqrt(0.0025));

  // Publishers
  std::string position_topic; n.getParam("position_topic", position_topic);
  positionPublisher_ = n.advertise<geometry_msgs::PointStamped>(position_topic.c_str(), 1);

  std::string contribution_topic; n.getParam("contribution_topic", contribution_topic);
  contributionPublisher_ = n.advertise<std_msgs::Float64MultiArray>(contribution_topic.c_str(), 1);

  // Frequency
  n.getParam("publication_freq", publicationFreq_);

  // Subscribers
  std::string human_position_topic; n.getParam("human_position_topic", human_position_topic);
  humanPositionSubscriber_ = n.subscribe(human_position_topic, 1, &Robot::humanPositionCallback, this);

  // Log
  ROS_INFO_STREAM(\
    "Created robot " << robotName_ << \
    " of type " << robotType_ << \
    " in [" << currentPosition_[0] << ", " << currentPosition_[1] << ", " << currentPosition_[2] << "]." << \
    " Publishing position in topic " << position_topic);
}

void Robot::humanPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
  const Eigen::Vector3d human_position (msg->point.x, msg->point.y, msg->point.z);
  humanDistance_ = (currentPosition_-human_position).norm() + uwbDistribution_(generator_);
}

void Robot::publishPosition(void){
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.point.x = currentPosition_[0];
  msg.point.y = currentPosition_[1];
  msg.point.z = currentPosition_[2];
  positionPublisher_.publish(msg);
}

void Robot::publishContribution(Eigen::Vector4d x, Eigen::Matrix4d R){
  std_msgs::Float64MultiArray msg;

  for(size_t i = 0; i < 4; i++)
    msg.data.push_back(x(i));

  for(size_t i = 0; i < 4; i++)
    for(size_t j = 0; j < 4; j++)
      msg.data.push_back(R(i,j));

  contributionPublisher_.publish(msg);
}
