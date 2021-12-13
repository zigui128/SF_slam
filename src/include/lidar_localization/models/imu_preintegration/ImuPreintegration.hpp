
#ifndef LIDAR_LOCALIZATION_MODELS_IMUPREINTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_IMUPREINTEGRATION_HPP_

#include <Eigen/Dense>
#include <vector>
#include "glog/logging.h"

#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/models/imu_preintegration/utility.hpp"

using namespace Eigen;
using namespace std;

namespace lidar_localization {

class PreintegratedImuMeasurements {

public:
  PreintegratedImuMeasurements(Vector3f bas_ = {0,0,0},
                               Vector3f bgs_ = {0,0,0},
                               Vector3f g_ = {0,0,9.81}):
  g(g_), bas(bas_),bgs(bgs_){
    Ps.clear();
    Vs.clear();
    Rs.clear();
  }

  ~PreintegratedImuMeasurements(){

  }


  void SetGravity(Vector3f g_){ g = g_;}

  void integrateMeasurement(const IMUData& imudata);

  void resetIntegrationAndSetBias(const Vector3f &bas_, const Vector3f &bgs_){
       bas = bas_;
       bgs = bgs_;
  }

  void repropogate();

  void predict(const PoseData &p, const Vector3f &bas_, const Vector3f &bgs_);

  void init_integration(const IMUData &imudata, Matrix3f Rs_= Matrix3f::Identity(),
                        Vector3f Ps_ = Vector3f(0.0f, 0.0f, 0.0f),
                        Vector3f Vs_= Vector3f(0.0f, 0.0f, 0.0f));

  public:
    vector<PoseData> ImuIntegratorPose;
  public:
    Vector3f acc_0;
    Vector3f gyr_0;
    Vector3f g;
    Vector3f bas;
    Vector3f bgs;
    vector<Vector3f> Ps;
    vector<Vector3f> Vs;
    vector<Matrix3f> Rs;
    bool initial = false;

};

class ImuOptimize {


};

}






#endif
