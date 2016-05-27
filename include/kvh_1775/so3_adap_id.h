/**
 * @file
 * @date May 2016.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).

 * @brief Class for adaptive identification on SO(3).
 *
 * Algorithm from James C. Kinsey's and 
 * Louis L. Whitcomb's paper: Adaptive Identification on the
 * Group of Rigid-Body Rotations and its Application to Underwater
 * Vehicle Navigation.
 */


#ifndef S03_ADAP_ID_H
#define S03_ADAP_ID_H

#include <Eigen/Core>


/**
 * Class for adaptive identificaiton on SO(3).
 */
class SO3AdapId
{
public:

    SO3AdapId(float); /**< Constructor. */
    virtual ~SO3AdapId(void); /**< Destructor. */
    
    /**
     * @brief Cycle estimation once.
     *
     * Assume input output relation is \f$y = Ru\f$ where \f$R\in SO(3)\f$ is constant
     * @param u Input measurement.
     * @param y Output measurement.
     * @param dt Time between samples.
     */
    void step(Eigen::Vector3d u,Eigen::Vector3d y,float dt);
    Eigen::Matrix3d R; /**< Estimatation of static rotation. */
    float k;    /**< Estimation gain. */
    

};

#endif
