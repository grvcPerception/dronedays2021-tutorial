#include "dronedays2021/dronedays2021.hpp"

UAV::UAV(ros::NodeHandle n) : Robot(n, "UAV"){
  // Simulation parameters
  n.getParam("sensing_freq", sensingFreq_);
  dt_ = sensingFreq_ / publicationFreq_;
  sensing_period_ = publicationFreq_ / sensingFreq_;
  std::vector<double> aux;
  n.getParam("init_position",  aux);
  positionA_ = Eigen::Map<Eigen::Vector3d>(aux.data(),3);
  n.getParam("final_position", aux);
  positionB_ = Eigen::Map<Eigen::Vector3d>(aux.data(),3);
  n.getParam("init_ugv_guess", aux);
  initUgvGuess_ = Eigen::Map<Eigen::Vector2d>(aux.data(),2);
  n.getParam("init_human_guess", aux);
  initHumanGuess_ = Eigen::Map<Eigen::Vector2d>(aux.data(),2);

  double mean; n.getParam("velocity", mean);
  simulationDistribution_ = std::normal_distribution<double>(mean, sqrt(0.1));

  n.getParam("var_gps", var_gps_);
  n.getParam("var_staticV", var_staticV_);
  n.getParam("var_staticH", var_staticH_);
  n.getParam("var_v", var_v_);
  gpsDistribution_ = std::normal_distribution<double>(0.0, sqrt(0.001));
  velDistribution_ = std::normal_distribution<double>(0.0, sqrt(0.0025));

  //Publishers
  std::string visualization_uav_topic; n.getParam("visualization_uav_topic", visualization_uav_topic);
  visualizationUAVPublisher_ = n.advertise<visualization_msgs::Marker>(visualization_uav_topic.c_str(), 1);
  std::string visualization_ugv_topic; n.getParam("visualization_ugv_topic", visualization_ugv_topic);
  visualizationUGVPublisher_ = n.advertise<visualization_msgs::Marker>(visualization_ugv_topic.c_str(), 1);
  std::string visualization_human_topic; n.getParam("visualization_human_topic", visualization_human_topic);
  visualizationHumanPublisher_ = n.advertise<visualization_msgs::Marker>(visualization_human_topic.c_str(), 1);

  // Subscribers
  std::string ugv_position_topic; n.getParam("ugv_position_topic", ugv_position_topic);
  ugvPositionSubscriber_ = n.subscribe(ugv_position_topic, 1, &UAV::ugvPositionCallback, this);

  std::string ugv_contribution_topic; n.getParam("ugv_contribution_topic", ugv_contribution_topic);
  ugvContributionSubscriber_ = n.subscribe(ugv_contribution_topic, 1, &UAV::ugvContributionCallback, this);
}

void UAV::ugvPositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
  const Eigen::Vector3d ugv_position (msg->point.x, msg->point.y, msg->point.z);
  ugvDistance_ = (currentPosition_-ugv_position).norm() + uwbDistribution_(generator_);
}

#if TEST_EXERCISE_B==0
void UAV::ugvContributionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if(humanDistance_ == 0)
    return;
  for (size_t i = 0; i < 4; i++)
    y_(i) += msg->data[i];

  for(size_t i = 0; i < 4; i++)
    for(size_t j = 0; j < 4; j++)
      Y_(i,j) += msg->data[4*i+j+4];
}
#endif

void UAV::doOneStepSimulation(Eigen::Vector3d *dir, double *last_dir_norm){
  if(goingToB_)
    *dir = positionB_ - currentPosition_;
  else
    *dir = positionA_ - currentPosition_;

  const double vel = simulationDistribution_(generator_);
  const Vector3d velV = (*dir) * ( (vel*vel)/(dir->norm()) );
  vel_ = velV + Vector3d(velDistribution_(generator_), velDistribution_(generator_), velDistribution_(generator_));
  currentPosition_ += velV * dt_;

  if(dir->norm() > *last_dir_norm){
    goingToB_ = !goingToB_;
    *last_dir_norm = std::numeric_limits<double>::infinity();
  }
  else *last_dir_norm = dir->norm();
}

void UAV::readGPS(){
  const Eigen::Vector3d noise(gpsDistribution_(generator_), gpsDistribution_(generator_), gpsDistribution_(generator_));
  gps_ = currentPosition_ + noise;
}

void UAV::run(){
  Eigen::Vector3d dir;
  double last_dir_norm = std::numeric_limits<double>::infinity();

  // Initialize filter
  x_.segment(0, 2) = initUgvGuess_;
  x_.segment(2, 2) = initHumanGuess_;
  x_.segment(4, 3) = positionA_;
  Matrix<double, 7, 7> Pinit = (Matrix<double, 7, 1>()
          << 1.0, 1.0, 30.0, 30.0, 0.5, 0.5, 0.5)
      .finished()
      .asDiagonal();
  Y_ = Pinit.inverse();
  y_ = Y_ * x_;

  // Simulation
  int k = 0;
  ros::Rate rate(publicationFreq_);
  while(ros::ok()){
    doOneStepSimulation(&dir, &last_dir_norm);

    doPrediction();
    if (++k  == sensing_period_){
      readGPS(); // UWB is read outside of this function
      doUpdate();
      // Recompute information form for the desired exchange
      const Matrix<double, 7, 7> P = Y_.inverse();
      const Matrix<double, 7, 1> x = P * y_;
      const Matrix<double, 4, 4> sentY = P.topLeftCorner<4,4>().inverse();
      const Matrix<double, 4, 1> senty = sentY * x.head<4>();
      publishContribution(senty, sentY);
      k = 0;
    }
    publishPosition();
    publishVisualization();
    ros::spinOnce();
    rate.sleep();
  }
}

inline const Matrix<double, 5, 7> UAV::getH() const{
  const double vx = x_(0);
  const double vy = x_(1);
  const double hx = x_(2);
  const double hy = x_(3);
  const double dx = x_(4);
  const double dy = x_(5);
  const double dz = x_(6);
  return (Matrix<double, 5, 7>() <<
          2.0 * (vx - dx), 2.0 * (vy - dy), 0, 0, 2.0 * (dx - vx), 2.0 * (dy - vy), 2.0 * dz,
          0, 0, 2.0 * (hx - dx), 2.0 * (hy - dy), 2.0 * (dx - hx), 2.0 * (dy - hy), 2.0 * dz,
          Matrix<double, 3, 4>::Zero(), Matrix<double, 3, 3>::Identity())
      .finished();
};

inline const Matrix<double, 7, 7> UAV::getQ() const{
  return (Matrix<double, 7, 1>()
          << var_staticV_, var_staticV_, var_staticH_, var_staticH_,
             var_v_, var_v_, var_v_)
      .finished()
      .asDiagonal();
};

inline const Matrix<double, 5, 5> UAV::getR() const{
  return (Matrix<double, 5, 1>()
          << 4.0 * ugvDistance_ * ugvDistance_ * var_uwb_,
             4.0 * humanDistance_ * humanDistance_ * var_uwb_,
             var_gps_, var_gps_, var_gps_)
      .finished()
      .asDiagonal();
};

void UAV::doPrediction() {
  const Matrix<double, 7, 7> P_t1 = Y_.inverse();

  // Recover current state
  x_ = P_t1 * y_;
  const Matrix<double, 7, 7> Q = getQ();

  // Predict next information matrix
  Y_ = (F_ * P_t1 * F_.transpose() + Q).inverse();

  // Predict next state and information vector
  x_.block(4, 0, 3, 1) += vel_ * dt_;
  y_ = Y_ * x_;
}

void UAV::doUpdate() {
  if(ugvDistance_ == 0 || humanDistance_ == 0)
    return;
  const Matrix<double, 5, 5> Ri = getR().inverse();
  const Matrix<double, 5, 7> H = getH();

  // Update information matrix
  Y_ = Y_ + H.transpose() * Ri * H;

  // Update information vector
  const Matrix<double, 5, 1> z = (Matrix<double, 5, 1>()
      <<  ugvDistance_ * ugvDistance_,
          humanDistance_ * humanDistance_,
          gps_).finished();
  const Matrix<double, 5, 1> h = (Matrix<double, 5, 1>()
      << (x_.segment(4,2) - x_.segment(0,2)).squaredNorm() + x_(6) * x_(6),
         (x_.segment(4,2) - x_.segment(2,2)).squaredNorm() + x_(6) * x_(6),
         x_(4),
         x_(5),
         x_(6)).finished();
  y_ = y_ + H.transpose() * Ri * (z - h + H * x_);
}

void UAV::publishVisualization(){
  const Matrix<double, 7, 7> P = Y_.inverse();
  const Matrix<double, 7, 1> x = P * y_;

  visualization_msgs::Marker msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  msg.type = visualization_msgs::Marker::SPHERE;
  msg.color.r = 1.0;
  msg.color.g = 0;
  msg.color.b = 0;
  msg.color.a = 1.0;
  // msg.action = 0;
  // msg.lifetime = ros::Duration(1);

  SelfAdjointEigenSolver<Matrix3d> eigensolverUAV(P.block(4,4,3,3));
  const Vector3d eiValsUAV = eigensolverUAV.eigenvalues();
  const Matrix3d eiVecsUAV = eigensolverUAV.eigenvectors();
  Quaterniond quatUAV(eiVecsUAV);
  quatUAV.normalize();

  msg.pose.position.x = x(4);
  msg.pose.position.y = x(5);
  msg.pose.position.z = x(6);
  msg.pose.orientation.x = quatUAV.x();
  msg.pose.orientation.y = quatUAV.y();
  msg.pose.orientation.z = quatUAV.z();
  msg.pose.orientation.w = quatUAV.w();
  msg.scale.x = eiValsUAV(0)*3.0;
  msg.scale.y = eiValsUAV(1)*3.0;
  msg.scale.z = eiValsUAV(2)*3.0;
  visualizationUAVPublisher_.publish(msg);

  // const Matrix<double, 4, 4> P2 = Y_.block(0,0,4,4).inverse();
  // const Matrix<double, 4, 1> x2 = (P2 * y_.segment(0,4));


  SelfAdjointEigenSolver<Matrix2d> eigensolverUGV(P.block(0,0,2,2));
  const Vector2d eiValsUGV = eigensolverUGV.eigenvalues();
  const Rotation2Dd eiVecsUGV(eigensolverUGV.eigenvectors());
  double angUGV = eiVecsUGV.angle();

  msg.pose.position.x = x(0);
  msg.pose.position.y = x(1);
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = sin(angUGV/2.0);
  msg.pose.orientation.w = cos(angUGV/2.0);
  msg.scale.x = eiValsUGV(0)*3.0;
  msg.scale.y = eiValsUGV(1)*3.0;
  msg.scale.z = 0.01;

  msg.color.r = 0.0;
  msg.color.g = 1.0;
  msg.color.b = 0;

  visualizationUGVPublisher_.publish(msg);

  SelfAdjointEigenSolver<Matrix2d> eigensolverHuman(P.block(2,2,2,2));
  const Vector2d eiValsHuman = eigensolverHuman.eigenvalues();
  const Rotation2Dd eiVecsHuman(eigensolverHuman.eigenvectors());
  double angHuman = eiVecsHuman.angle();

  msg.pose.position.x = x(2);
  msg.pose.position.y = x(3);
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = sin(angHuman/2.0);
  msg.pose.orientation.w = cos(angHuman/2.0);
  msg.scale.x = eiValsHuman(0)*3.0;
  msg.scale.y = eiValsHuman(1)*3.0;
  msg.scale.z = 0.01;

  msg.color.r = 0.0;
  msg.color.g = 0.0;
  msg.color.b = 1.0;

  visualizationHumanPublisher_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uav");
  ros::NodeHandle n("~");

  UAV uav(n);
  uav.run();

  return 0;
}
