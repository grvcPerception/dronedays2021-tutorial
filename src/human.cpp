#include "dronedays2021/dronedays2021.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "human");
  ros::NodeHandle n("~");

  ros::Publisher pub;
  std::string topic; n.getParam("topic", topic);
  pub = n.advertise<geometry_msgs::PointStamped>(topic.c_str(), 1);

  double freq = 0;
  n.getParam("publication_freq", freq);
  ros::Rate rate(freq);

  std::vector<double> position;
  n.getParam("position", position);

  geometry_msgs::PointStamped msg;
  msg.header.frame_id = "world";
  msg.point.x = position[0];
  msg.point.y = position[1];
  msg.point.z = 0;

  while(ros::ok()){
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    rate.sleep();
  }

  return 0;
}
