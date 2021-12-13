#include <gtest/gtest.h>
#include "lidar_localization/models/imu_preintegration/imu_simulation.hpp"
#include "lidar_localization/models/imu_preintegration/ImuPreintegration.hpp"
#include <iostream>

using namespace lidar_localization;

/*TEST(Preintegration, zerobias){
  //generater IMU simulation data, including imu pose gyro acc
  IMU imuGen;

  double t_start = 0.0;
  double t_end = 20.0;
  double imu_frequency = 30;
  // create imu data
  // imu pose gyro acc
  std::vector< MotionData > imudata;

  for (float t = t_start; t < t_end;) {
    MotionData data = imuGen.MotionModel(t);
    imudata.push_back(data);
    t += 1.0/imu_frequency;
  }


  double dt_start = imudata.at(0).timestamp;
  Eigen::Vector3f Pwb = imudata.at(0).twb;              // position :    from  imu measurements
  Eigen::Quaternionf Qwb(imudata.at(0).Rwb);            // Quaternionf:  from imu measurements
  Eigen::Vector3f Vw = imudata[0].imu_velocity;          // velocity  :   from imu measurements
  Eigen::Vector3f gw(0,0,-9.81);    // ENU frame

  Eigen::Vector3f imu_gyro_last = imudata.at(0).imu_gyro;
  Eigen::Vector3f imu_acc_last = imudata.at(0).imu_acc;

  for (int i = 1; i < imudata.size(); ++i) {

    MotionData imupose = imudata[i];
    double dt_end = imupose.timestamp;
    double dt = dt_end - dt_start;
    dt_start = dt_end;

    Eigen::Vector3f imu_gyro_current = imupose.imu_gyro;
    Eigen::Vector3f imu_acc_current = imupose.imu_acc;
    Eigen::Vector3f imu_gyro = (imu_gyro_last + imu_gyro_current)/2.0;
    Eigen::Vector3f imu_acc = (imu_acc_last + imu_acc_current)/2.0;
    imu_gyro_last = imu_gyro_current;
    imu_acc_last = imu_acc_current;

    //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
    Eigen::Quaternionf dq;
  //  Eigen::Vector3f dtheta_half =  imupose.imu_gyro * dt /2.0;
    Eigen::Vector3f dtheta_half =  imu_gyro * dt /2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    dq.normalize();

     /// 中值积分
   // Eigen::Vector3f acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
      Eigen::Vector3f acc_w = Qwb * (imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
    Qwb = Qwb * dq;
    Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
    Vw = Vw + acc_w * dt;



    //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
    std::cout <<imupose.timestamp<<" "
              <<Pwb(0)<<" "
              <<Pwb(1)<<" "
              <<Pwb(2)<<" "
              <<imupose.twb(0) << " "
              <<imupose.twb(1) << " "
              <<imupose.twb(2) << " "
              <<std::endl;

  }

  std::cout<<"test　end"<<std::endl;
} */


TEST(Preintegration, zerobias){

  double t_start = 0.0;
  double t_end = 20.0;
  double imu_frequency = 30;

  //generater IMU simulation data, including imu pose gyro acc
  IMU imuGen(imu_frequency);

  // create imu data
  // imu pose gyro acc
  std::vector< MotionData > imudata;

  for (float t = t_start; t < t_end;) {
    MotionData data = imuGen.MotionModel(t);
    imudata.push_back(data);
    t += 1.0/imu_frequency;
  }

  PreintegratedImuMeasurements imu_pre;

  MotionData imupose = imudata[0];

  IMUData imu_data0;
  imu_data0.time = imupose.timestamp;
  imu_data0.linear_acceleration.x = imupose.imu_acc(0);
  imu_data0.linear_acceleration.y = imupose.imu_acc(1);
  imu_data0.linear_acceleration.z = imupose.imu_acc(2);
  imu_data0.angular_velocity.x = imupose.imu_gyro(0);
  imu_data0.angular_velocity.y = imupose.imu_gyro(1);
  imu_data0.angular_velocity.z = imupose.imu_gyro(2);

  imu_pre.init_integration(imu_data0, imudata.at(0).Rwb,
                           imudata.at(0).twb, imudata[0].imu_velocity);


  for (int i = 1; i < imudata.size(); ++i) {
      MotionData imupose = imudata[i];

      IMUData imu_data;
      imu_data.time = imupose.timestamp;
      imu_data.linear_acceleration.x = imupose.imu_acc(0);
      imu_data.linear_acceleration.y = imupose.imu_acc(1);
      imu_data.linear_acceleration.z = imupose.imu_acc(2);
      imu_data.angular_velocity.x = imupose.imu_gyro(0);
      imu_data.angular_velocity.y = imupose.imu_gyro(1);
      imu_data.angular_velocity.z = imupose.imu_gyro(2);

      imu_pre.integrateMeasurement(imu_data);
  }

  for (int i = 0; i < imudata.size(); ++i) {
      MotionData imupose = imudata[i];

      std::cout << imupose.timestamp<<" "<<std::endl;

      std::cout <<imupose.twb(0) << " "
                <<imu_pre.Ps[i](0) << " "
                <<imupose.twb(1) << " "
                <<imu_pre.Ps[i](1) << " "
                <<imupose.twb(2) << " "
                <<imu_pre.Ps[i](2) << " "
                <<std::endl;

      std::cout << imupose.imu_velocity(0) << " "
                << imu_pre.Vs[i](0) << " "
                << imupose.imu_velocity(1) << " "
                << imu_pre.Vs[i](1) << " "
                << imupose.imu_velocity(2) << " "
                << imu_pre.Vs[i](2) << " "
                << std::endl;

      Eigen::Quaternionf Qwb(imupose.Rwb);
      Eigen::Quaternionf qwb(imu_pre.Rs[i]);
      std::cout << Qwb.x() << " "
                << qwb.x() << " "
                << Qwb.y() << " "
                << qwb.y() << " "
                << Qwb.z() << " "
                << qwb.z() << " "
                << Qwb.w() << " "
                << qwb.w() << " "
                << std::endl;

  }

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



