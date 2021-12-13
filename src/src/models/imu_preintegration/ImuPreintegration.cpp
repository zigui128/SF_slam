#include "lidar_localization/models/imu_preintegration/ImuPreintegration.hpp"
#include "iostream"

namespace lidar_localization {

void PreintegratedImuMeasurements::integrateMeasurement(const IMUData &imudata){
  if(!initial){
    std::cout << "Function integrateMeasurement need initial!!!" << std::endl;
    return;
  }

  Vector3f linear_acceleration((float)imudata.linear_acceleration.x,
                               (float)imudata.linear_acceleration.y,
                               (float)imudata.linear_acceleration.z);
  Vector3f angular_velocity((float)imudata.angular_velocity.x,
                            (float)imudata.angular_velocity.y,
                            (float)imudata.angular_velocity.z);



  int j = ImuIntegratorPose.size() -1;

  double dt = imudata.time -  ImuIntegratorPose[j].time;
  Vector3f un_acc_0 = Rs[j] * (acc_0 - bas) - g;
  Vector3f un_gyr = 0.5 * (gyr_0 + angular_velocity) - bgs;
  Matrix3f Rs_ = Rs[j] * Utility::deltaQ(un_gyr * dt).toRotationMatrix();
  Vector3f un_acc_1 = Rs[j] * (linear_acceleration - bas) - g;
  Vector3f un_acc = 0.5 * (un_acc_0 + un_acc_1);
  Vector3f Ps_ = Ps[j] + dt * Vs[j] + 0.5 * dt * dt * un_acc;
  Vector3f Vs_  = Vs[j] + dt * un_acc;


  Rs.push_back(Rs_);
  Ps.push_back(Ps_);
  Vs.push_back(Vs_);


  PoseData p;
  p.pose.block<3,3>(0,0) = Rs_;
  p.pose.block<3,1>(0,3) = Ps_.transpose();

  p.time = imudata.time;
  ImuIntegratorPose.push_back(p);

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;

}

void PreintegratedImuMeasurements::init_integration(const IMUData &imudata, Matrix3f Rs_, Vector3f Ps_, Vector3f Vs_){
  Vector3f linear_acceleration((float)imudata.linear_acceleration.x,
                               (float)imudata.linear_acceleration.y,
                               (float)imudata.linear_acceleration.z);
  Vector3f angular_velocity((float)imudata.angular_velocity.x,
                            (float)imudata.angular_velocity.y,
                            (float)imudata.angular_velocity.z);


  Rs.push_back(Rs_);
  Ps.push_back(Ps_);
  Vs.push_back(Vs_);

  PoseData p;
  p.pose.block<3,3>(0,0) = Rs_;
  p.pose.block<3,1>(0,3) = Ps_.transpose();

  p.time = imudata.time;
  ImuIntegratorPose.push_back(p);

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;

  initial = true;
}

}
