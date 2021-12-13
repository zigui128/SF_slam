#include "lidar_localization/models/imu_preintegration/imu_simulation.hpp"


namespace lidar_localization {
// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3f euler2Rotation( Eigen::Vector3f  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3f RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Matrix3f eulerRates2bodyRates(Eigen::Vector3f eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3f R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}

MotionData IMU::MotionModel(double t)
{

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           
    float K1 = 10;          
    float K = M_PI/ 10;    
    
    // translation
    // twb:  body frame in world frame
    Eigen::Vector3f position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    Eigen::Vector3f dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));
    double K2 = K*K;
    Eigen::Vector3f ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3f eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );
    Eigen::Vector3f eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);


    Eigen::Matrix3f Rwb = euler2Rotation(eulerAngles);
    Eigen::Vector3f imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;

    Eigen::Vector3f gn (0,0,-9.81);
    Eigen::Vector3f imu_acc = Rwb.transpose() * ( ddp -  gn );

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}
}

