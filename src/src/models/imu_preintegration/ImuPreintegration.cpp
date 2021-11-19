
/*
class PreintegratedImuMeasurements {

public:
  vector<ImuPose> ImuIntegrator;

private:
  Vector3d acc_0;
  Vector3d gyr_0;
  Vector3d g;
  Vector3d bas;
  Vector3d bgs;
  vector<P_pose> Ps;
  vector<R_pose> Rs;
  vector<V_pose> Vs;


 public:
  PreintegratedImuMeasurements( Vector3d g_, Vector3d bas_, Vector3d bgs_):
  g(g_), bas(bas_),bgs(bgs_){

  }

  ~PreintegratedImuMeasurements(){

  }

  void predict(ImuPose &p, Imubias &ba){

  }



  void integrateMeasurement(const Vector3d &linear_acceleration, const Vector3d &angular_velocity, double &dt){
    if(ImuIntegrator.size() == 0){
      PoseT = (0,0,0);
      poseR = (1,0,0;0,1,0;0,0,1);

    }
    else {

        int j = ImuIntegrator.size() -1;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs;
        R_pose Rs_ = Rs[j] * Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        P_Pose Ps_ = Ps[j] + dt * Vs[j] + 0.5 * dt * dt * un_acc;
        V_POse Vs_  = Vs[j] + dt * un_acc;

        Rs.push_back(Rs);
        Ps.push_back(Ps);
        Vs.push_back(Vs);

        ImuPose p_;
        ImuIntegrator.push_back(p_);
    }

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }



  void resetIntegrationAndSetBias(Vector3d bas_, Vector3d bgs_){
       bas = bas_;
       bgs = bgs_;
  }

  void repropogate(){}

};


class ImuOptimize {


};*/
