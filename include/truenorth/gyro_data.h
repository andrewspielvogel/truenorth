/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * 
 * @brief Gyro data class.
 * 
 * Class for storing a gyro data packet.
 */


#ifndef GYRO_DATA_H
#define GYRO_DATA_H

#include <truenorth/att_est.h>
#include <Eigen/Core>
#include <string>

/**
 * @brief Gyro data class.
 * 
 * Class for storing a gyro data packet.
 */
class GyroData
{
public:
    Eigen::Vector3d ang; /**<  Angular velocity. */
    Eigen::Vector3d acc; /**< Linear acceleration. */
    Eigen::Vector3d mag; /**< Magnetometer. */
    std::vector<bool> status; /**< sensor status. */
    float temp; /**< Sensor temperature. */
    unsigned int seq_num; /**< Sequence number. */


    double timestamp; /**< Timestamp. */
    double diff; /**< Time difference between last two data packets. */
    Eigen::Vector3d bias_acc; /**< Linear acceleration bias estimation. */
    Eigen::Vector3d bias_ang; /**< Angular velocity bias estimation. */
    Eigen::Vector3d bias_z; /**< z bias constant estimation. */
    Eigen::Vector3d acc_est; /**< Linear acceleration estimation. */
    Eigen::Vector3d att; /**< Attitude estimation. */

    void est_bias(); /**< Cycle bias estimation. */
    void est_att(); /**< Cycle attitude estimation. */

    /**
     * @brief Constructor.
     *
     * GyroData class constructor.
     * @param k Estimation gains.
     * @param align Alignment rotation between instrument and vehicle.
     * @param log_location Location of log file.
     * @param R0 Initial estimate of Rbar matrix.
     */
    GyroData(Eigen::VectorXd k, Eigen::Matrix3d align, std::string log_location, float lat);
    virtual ~GyroData(void); /**< Destructor. */
    void log(); /**< Log data. */

    float t_start; /**< Start time. */

 private:
    FILE *fp_; /**< Log file. */
    AttEst Rbar_; /**< Rbar matrix estimation. */
    Eigen::Matrix3d R_align_; /**< Instrument coordinate frame to vehicle coordinate frame rotation */
    float k1_; /**< Linear acceleration estimation gain. */
    float k2_; /**< Linear acceleration bias estimation gain. */
    float k3_; /**< Angular velocity bias estimation gain. */
    float k4_; /**< z bias constant estimation gain. */
    float k5_; /**< Local level estimation gain. */
    float k6_;    /**< Heading estimation gain. */


};

#endif
