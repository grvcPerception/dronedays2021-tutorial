#include "dronedays2021/dronedays2021.hpp"

#if TEST_EXERCISE_A==1
/* EXERCISE A: COMPLETE THE UPDATING STAGE FOR THE UGV */
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
  const double z = humanDistance_ * humanDistance_;
  const double h = (x_.segment(0, 2) - x_.segment(2, 2)).squaredNorm();

  // Update information matrix Y_

  // Update information vector y_
}
#endif

/* EXERCISE B: GET THE CONTRIBUTION FROM THE UGV */
// Tip: msg.data == [y0, y1, y2, y3, Y00, Y01, ..., Y33]
#if TEST_EXERCISE_B==1
void UAV::ugvContributionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  if(humanDistance_ == 0)
    return;

  // Sum information vector

  // Sum information matrix

}
#endif
