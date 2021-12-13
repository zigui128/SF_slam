#ifndef LIDAR_LOCALIZATION_MODELS_IMUSIMULATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_IMUSIMULATION_HPP_

#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

namespace lidar_localization {
struct MotionData
{

    double timestamp;
    Eigen::Matrix3f Rwb;
    Eigen::Vector3f twb;
    Eigen::Vector3f imu_acc;
    Eigen::Vector3f imu_gyro;

    Eigen::Vector3f imu_gyro_bias;
    Eigen::Vector3f imu_acc_bias;

    Eigen::Vector3f imu_velocity;
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3f euler2Rotation( Eigen::Vector3f  eulerAngles);
Eigen::Matrix3f eulerRates2bodyRates(Eigen::Vector3f eulerAngles);


class IMU
{
public:
  IMU(int imu_frequency_);
  void addIMUnoise(MotionData& data);
  MotionData MotionModel(double t);

  Eigen::Vector3f gyro_bias_;
  Eigen::Vector3f acc_bias_;
  int imu_frequency;
  float imu_timestep = 1./imu_frequency;
  // noise
  float gyro_bias_sigma = 1.0e-5;
  float acc_bias_sigma = 0.0001;

  float gyro_noise_sigma = 0.015;    // rad/s * 1/sqrt(hz)
  float acc_noise_sigma = 0.019;      //　m/(s^2) * 1/sqrt(hz)
    
};
}


#endif
