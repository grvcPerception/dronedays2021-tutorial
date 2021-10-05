#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <random>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <termios.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>

#ifndef DRONEDAYS2021_HPP_
#define DRONEDAYS2021_HPP_

#define TEST_EXERCISE_A 1
#define TEST_EXERCISE_B 1

using namespace Eigen;

class Robot {
public:
  Robot(ros::NodeHandle, std::string);
  ~Robot(){};
protected:
  double currentOrientation_ = 0;
  double humanDistance_ = 0;
  double publicationFreq_;
  double var_uwb_;
  Eigen::Vector3d currentPosition_;
  ros::Publisher contributionPublisher_;
  ros::Publisher positionPublisher_;
  ros::Subscriber humanPositionSubscriber_;
  std::default_random_engine generator_;
  std::normal_distribution<double> uwbDistribution_;
  std::string robotName_;
  std::string robotType_;

  void humanPositionCallback(const geometry_msgs::PointStamped::ConstPtr&);
  void publishContribution(Eigen::Vector4d, Eigen::Matrix4d);
  void publishPosition(void);
};

class UAV: private Robot {
public:
  UAV(ros::NodeHandle);
  ~UAV(){}
  void run(void);
private:
  bool goingToB_ = true;
  double sensingFreq_;
  double ugvDistance_ = 0;
  Eigen::Vector3d positionA_;
  Eigen::Vector3d positionB_;
  int sensing_period_;
  ros::Publisher visualizationHumanPublisher_;
  ros::Publisher visualizationUAVPublisher_;
  ros::Publisher visualizationUGVPublisher_;
  ros::Subscriber ugvContributionSubscriber_;
  ros::Subscriber ugvPositionSubscriber_;
  std::normal_distribution<double> gpsDistribution_;
  std::normal_distribution<double> simulationDistribution_;
  std::normal_distribution<double> velDistribution_;

  void doOneStepSimulation(Eigen::Vector3d*, double*);
  void publishVisualization(void);
  void readGPS(void);
  void ugvContributionCallback(const std_msgs::Float64MultiArray::ConstPtr&);
  void ugvPositionCallback(const geometry_msgs::PointStamped::ConstPtr&);

  // Filter
  double var_gps_;
  double var_staticV_, var_staticH_, var_v_;
  Vector3d vel_;
  Vector3d gps_;
  double dt_;
  Eigen::Vector2d initUgvGuess_;
  Eigen::Vector2d initHumanGuess_;
  Matrix<double, 7, 1> x_;
  Matrix<double, 7, 1> y_;
  Matrix<double, 7, 7> Y_;
  const Matrix<double, 7, 7> F_ = Matrix<double, 7, 7>::Identity();
  inline const Matrix<double, 5, 7> getH() const;
  inline const Matrix<double, 5, 5> getR() const;
  inline const Matrix<double, 7, 7> getQ() const;
  void doPrediction();
  void doUpdate();
};

class UGV: private Robot {
public:
  UGV(ros::NodeHandle);
  ~UGV(){}
  void run(void);
private:
  ros::Subscriber uavContributionSubscriber_;
  ros::Subscriber keyboardSubscriber_;
  int keyWS_ = 0;
  int keyAD_ = 0;

  void doOneStepSimulation(double*, double*);
  void uavContributionCallback(const std_msgs::Float64MultiArray::ConstPtr&);
  void keyboardCallback(const std_msgs::Int8::ConstPtr&);

  // Filter
  double sensingFreq_;
  int sensing_period_;
  ros::Publisher visualizationUGVPublisher_;
  ros::Publisher visualizationHumanPublisher_;
  std::normal_distribution<double> velDistribution_;
  std::normal_distribution<double> angvelDistribution_;

  void publishVisualization(void);

  double var_v_, var_w_;
  double var_staticH_;
  double vel_, ang_vel_;
  Eigen::Vector2d initHumanGuess_;
  double dt_;
  Matrix<double, 5, 1> x_;
  Matrix<double, 5, 1> y_;
  Matrix<double, 5, 5> Y_;
  inline const Matrix<double, 5, 5> getF() const;
  inline const Matrix<double, 1, 5> getH() const;
  inline const double getR() const;
  inline const Matrix<double, 5, 5> getQ() const;
  void doPrediction();
  void doUpdate();
};

#endif //DRONEDAYS2021_HPP_
