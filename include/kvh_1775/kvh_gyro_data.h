/*
 * gyro_data.h
 * gyro data class
 *
 * created May 2016
 * Andrew Spielvogel
 * andrewspielvogel@gmail.com
 */


#ifndef GYRO_DATA_H
#define GYRO_DATA_H

#include <eigen/Eigen/Core>


//class for storing a gyro data packet
class GyroData
{
public:
    Eigen::Vector3d ang;
    Eigen::Vector3d acc;
    Eigen::Vector3d mag;
    std::vector<bool> status;
    float temp;
    unsigned int seq_num;


    double prev_time;
    double diff;
    Eigen::Vector3d bias_acc;
    Eigen::Vector3d bias_ang;
    Eigen::Vector3d bias_z;
    Eigen::Vector3d acc_est;
    Eigen::Vector3d att;

    void est_bias();
    void est_att();
    float k1,k2,k3,k4,k5;

    GyroData(float k1_,float k2_,float k3_,float k4_, float k5_, Eigen::Matrix3d align_);
    virtual ~GyroData(void);
    void set_values (Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, float, std::vector<bool>, unsigned int);

 private:
    FILE *fp_; //log file
    Eigen::Matrix3d Rbar_;
    Eigen::Matrix3d Rd_;
    Eigen::Matrix3d R_align_;

};

#endif
