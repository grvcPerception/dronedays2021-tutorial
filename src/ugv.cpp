#include "dronedays2021/dronedays2021.hpp"

double cosd(double x){return cos(x*M_PI/180.0);}
double sind(double x){return sin(x*M_PI/180.0);}

UGV::UGV(ros::NodeHandle n) : Robot(n, "UGV"){
  // Simulation parameters
  n.getParam("sensing_freq", sensingFreq_);
  dt_ = sensingFreq_ / publicationFreq_;
  sensing_period_ = publicationFreq_ / sensingFreq_;
  std::vector<double> aux; n.getParam("init_position",  aux);
  currentPosition_ = Eigen::Map<Eigen::Vector3d>(aux.data(),3);
  n.getParam("init_human_guess", aux);
  initHumanGuess_ = Eigen::Map<Eigen::Vector2d>(aux.data(),2);

  n.getParam("var_staticH", var_staticH_);
  n.getParam("var_v", var_v_);
  n.getParam("var_w", var_w_);
  velDistribution_ = std::normal_distribution<double>(0.0, sqrt(0.00025));
  angvelDistribution_ = std::normal_distribution<double>(0.0, sqrt(0.00003));

  //Publishers
  std::string visualization_ugv_topic; n.getParam("visualization_ugv_topic", visualization_ugv_topic);
  visualizationUGVPublisher_ = n.advertise<visualization_msgs::Marker>(visualization_ugv_topic.c_str(), 1);
  std::string visualization_human_topic; n.getParam("visualization_human_topic", visualization_human_topic);
  visualizationHumanPublisher_ = n.advertise<visualization_msgs::Marker>(visualization_human_topic.c_str(), 1);

  // Subscribers
  std::string uav_contribution_topic; n.getParam("uav_contribution_topic", uav_contribution_topic);
  uavContributionSubscriber_ = n.subscribe(uav_contribution_topic, 1, &UGV::uavContributionCallback, this);

  keyboardSubscriber_ = n.subscribe("/keyboard", 1, &UGV::keyboardCallback, this);
}

void UGV::keyboardCallback(const std_msgs::Int8::ConstPtr& msg){
  switch (msg->data) {
    case   'w' : keyWS_ =  1; break;
    case   's' : keyWS_ = -1; break;
    case   'a' : keyAD_ =  1; break;
    case   'd' : keyAD_ = -1; break;
    case (-'w'): keyWS_ =  0; break;
    case (-'s'): keyWS_ =  0; break;
    case (-'a'): keyAD_ =  0; break;
    case (-'d'): keyAD_ =  0; break;
  }
}

void UGV::uavContributionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if(humanDistance_ == 0)
    return;
  for (size_t i = 0; i < 4; i++)
    y_(i) += msg->data[i];

  for(size_t i = 0; i < 4; i++)
    for(size_t j = 0; j < 4; j++)
      Y_(i,j) += msg->data[4*i+j+4];
}

void UGV::doOneStepSimulation(double *vel, double *ang){
  const double MAX_VEL = 0.01;
  const double MAX_ANG = 50;

  if(keyWS_ > 0) *vel += 0.001;
  else if(keyWS_ < 0) *vel -= 0.001;
  else (*vel) *= 0.99; //friction

  if(keyAD_ > 0) *ang += 1;
  else if(keyAD_ < 0) *ang -= 1;
  else *ang = 0;

  if(*vel < -MAX_VEL) *vel = -MAX_VEL;
  if(*vel > +MAX_VEL) *vel = +MAX_VEL;
  if(*ang < -MAX_ANG) *ang = -MAX_ANG;
  if(*ang > +MAX_ANG) *ang = +MAX_ANG;

  currentPosition_[0] += (*vel) * cosd(*ang) * cosd(currentOrientation_);
  currentPosition_[1] += (*vel)*cosd(*ang)*sind(currentOrientation_);
  vel_ = (*vel) * cosd(*ang) / dt_ + velDistribution_(generator_);
  ang_vel_ = (30 * (*vel) * sind(*ang) / dt_)*M_PI/180.0 + angvelDistribution_(generator_);
  currentOrientation_ += 30 * (*vel) * sind(*ang);

  if(abs(*vel) < 1e-4) *vel = 0;
}

void UGV::run(){
  double vel = 0;
  double ang = 0;

  // Initialize filter
  x_.segment(0, 2) = currentPosition_.segment(0,2);
  x_.segment(2, 2) = initHumanGuess_;
  x_(4) = 0.0;
  Matrix<double, 5, 5> Pinit = (Matrix<double, 5, 1>()
          << 1.0, 1.0, 30.0, 30.0, 0.1)
      .finished()
      .asDiagonal();
  Y_ = Pinit.inverse();
  y_ = Y_ * x_;

  // Simulation
  int k = 0;
  ros::Rate rate(publicationFreq_);
  while(ros::ok()){
    doOneStepSimulation(&vel, &ang);
    doPrediction();
    if (++k  == sensing_period_){
      doUpdate();
      // Recompute information form for the desired exchange
      const Matrix<double, 5, 5> P = Y_.inverse();
      const Matrix<double, 5, 1> x = P * y_;
      const Matrix<double, 4, 4> sentY = P.topLeftCorner<4,4>().inverse();
      const Matrix<double, 4, 1> senty = sentY * x.head<4>();
      publishContribution(senty, sentY);
      k = 0;
    }
    publishPosition();
    ros::spinOnce();
    publishVisualization();
    rate.sleep();
  }
}

inline const Matrix<double, 5, 5> UGV::getF() const{
  const double va = x_(4);
  Matrix<double, 5, 5> F = Matrix<double, 5, 5>::Identity();
  F(0, 4) = -sin(va) * vel_ * dt_;
  F(1, 4) = cos(va) * vel_ * dt_;
  return F;
};

inline const Matrix<double, 5, 5> UGV::getQ() const{
  const double vx = x_(0);
  const double vy = x_(1);
  const double hx = x_(2);
  const double hy = x_(3);
  const double va = x_(4);
  return (Matrix<double, 5, 1>()
          << cos(va)*cos(va)*dt_*dt_*var_v_, sin(va)*sin(va)*dt_*dt_*var_v_, var_staticH_, var_staticH_,
             dt_*dt_*var_w_)
      .finished()
      .asDiagonal();
};

void UGV::doPrediction() {
  Matrix<double, 5, 5> P_t1 = Y_.inverse();

  // Recover current state
  x_ = P_t1 * y_;
  const Matrix<double, 5, 5> Q = getQ();
  const Matrix<double, 5, 5> F = getF();

  // Limit covariance for stability
  if(P_t1(0,0) < 0.4) P_t1(0,0) = 0.4;
  if(P_t1(1,1) < 0.4) P_t1(1,1) = 0.4;
  if(P_t1(2,2) < 0.4) P_t1(2,2) = 0.4;
  if(P_t1(3,3) < 0.4) P_t1(3,3) = 0.4;
  if(P_t1(4,4) < 0.01) P_t1(4,4) = 0.01;

  // Predict next information matrix
  Y_ = (F * P_t1 * F.transpose() + Q).inverse();

  // Predict next state and information vector
  const double va = x_(4);
  x_(0) += cos(va) * vel_ * dt_;
  x_(1) += sin(va) * vel_ * dt_;
  x_(4) += ang_vel_ * dt_;

  y_ = Y_ * x_;
}

#if TEST_EXERCISE_A==0
inline const Matrix<double, 1, 5> UGV::getH() const{
  const double vx = x_(0);
  const double vy = x_(1);
  const double hx = x_(2);
  const double hy = x_(3);
  return (Matrix<double, 1, 5>() << 2.0 * (vx - hx), 2.0 * (vy - hy), 2.0 * (hx - vx), 2.0 * (hy - vy), 0.0)
      .finished();
};

inline const double UGV::getR() const{
  return 4.0 * humanDistance_ * humanDistance_ * var_uwb_;
};

void UGV::doUpdate() {
  if(humanDistance_ == 0)
    return;
  const double Ri = 1.0 / getR();
  const Matrix<double, 1, 5> H = getH();

  // Update information matrix
  Y_ = Y_ + H.transpose() * Ri * H;

  // Update information vector
  const double z = humanDistance_ * humanDistance_;
  const double h = (x_.segment(0, 2) - x_.segment(2, 2)).squaredNorm();
  y_ = y_ + H.transpose() * Ri * (z - h + H * x_);
}
#endif

void UGV::publishVisualization(){
  // const Matrix<double, 4, 4> P = Y_.block(0,0,4,4).inverse();
  // const Matrix<double, 4, 1> x = (P * y_.segment(0,4));
  const Matrix<double, 5, 5> P = Y_.inverse();
  const Matrix<double, 5, 1> x = (P * y_);

  visualization_msgs::Marker msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  msg.type = visualization_msgs::Marker::SPHERE;
  msg.color.r = 239.0/255.0;
  msg.color.g = 127.0/255.0;
  msg.color.b = 026.0/255.0;
  msg.color.a = 1.0;

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

  msg.color.r = 1.0;
  msg.color.g = 0.0;
  msg.color.b = 1.0;

  visualizationHumanPublisher_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ugv");
  ros::NodeHandle n("~");

  UGV ugv(n);
  ugv.run();

  return 0;
}
