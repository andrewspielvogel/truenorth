/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Gyro data class.
 * 
 * Class for storing a gyro data packet.
 */


#ifndef GYRO_DATA_H
#define GYRO_DATA_H

#include <Eigen/Core>
#include <vector>
/**
 * @brief Gyro data class.
 * 
 * Class for storing an IMU data packet.
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
    double comp_timestamp; /**< Computer timestamp. */
    double diff; /**< Time difference between last two data packets. */
    double t_start; /**< Start time. */
    double hz; /**< Sampling rate. */

    /**
     * @brief Constructor.
     *
     * GyroData class constructor.
     * @param hz Sampling rate.
     */
    GyroData(int hz); /**< Constructor. */
    virtual ~GyroData(void); /**< Destructor. */

 };

#endif
