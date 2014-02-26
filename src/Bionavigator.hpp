/*
 * =====================================================================================
 *
 *       Filename:  Bionavigator.hpp
 *
 *    Description:  Header for main Bionavigator class
 *
 *        Version:  1.0
 *        Created:  19/09/13 11:49:16
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */


#ifndef  Bionavigator_INC
#define  Bionavigator_INC

#include "HDCells.hpp"
#include "RotationCellClockwise.hpp"
#include "RotationCellCounterClockwise.hpp"
#include "VisionCells.hpp"
#include "HD_VisionSynapseSet.hpp"
#include "HD_RotationSynapseSet.hpp"
#include "HDSynapseSet.hpp"
#include "GridCells.hpp"
#include "GridCellsSynapseSet.hpp"
#include "GridCells_HD_VelocitySynapseSet.hpp"
#include "VelocityCell.hpp"
#include "GridCells_VisionSynapseSet.hpp"
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <fstream>
#include <cmath>


/*
 * =====================================================================================
 *        Class:  Bionavigator
 *  Description:  The main bionavigator class
 * =====================================================================================
 */
/**
 * @class Bionavigator
 *
 * @brief The main class that runs my system
 *
 * This class can be thought of as the brain, where the different components
 * link together via their synapses and carry out the operation.
 * 
 */
class Bionavigator
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Bionavigator ();                             /**< constructor      */
        Bionavigator ( const Bionavigator &other );   /**< copy constructor */
        ~Bionavigator ();                            /**< destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        Bionavigator& operator = ( const Bionavigator &other ); /* assignment operator */

        /**
         * @brief Initialize the system
         *
         * Check for presence of required nodes and inputs, set up synapses and
         * neuron sets
         *
         * @param None
         *
         * @returns None
         */
        void Init ();

        /**
         * @brief Calibrate the synaptic weights of all synapses before
         * functioning of system
         * 
         * @param None
         *
         * @returns None
         */
        void Calibrate ();

        /**
         * @brief initialize the system to an initial heading
         *
         * Basically, create a peak and let it stabilize so we can begin to
         * process angular velocity inputs.
         *
         * @param None
         *
         * @return void
         */
        void SetInitialDirection ();

        /**
         * @brief initialize the system to an initial location
         *
         * Basically, create a peak and let it stabilize so we can begin to
         * process angular velocity inputs.
         *
         * @param None
         *
         * @return void
         */
        void SetInitialLocation ();

        /**
         * @brief Publish current direction
         *
         * @todo I'll need to add more methods as I look to implement land mark
         * navigation
         *
         * @param rImuMessage reference to the received message
         *
         * @return void
         *
         * @todo When I work on the vision part, I'll need to use a message
         * filter TimeSynchronizer to keep the inputs from these nodes in sync
         *
         * http://wiki.ros.org/message_filters
         */
        void CallbackPublishDirection (const sensor_msgs::Imu::ConstPtr& rImuMessage);

        /**
         * @brief set up ros related stuff
         *
         * Subscribe/advertise. This can be called after the initialization is
         * done, or before, as needed.
         *
         *
         * @param none
         *
         * @return void
         */
        void RosInit ();

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */


        /**
         * @brief Do the work, update the head direction
         *
         * @param None
         *
         * @return None
         */
        void HeadDirection (double angularVelocityY);

        /**
         * @brief Helper function
         * 
         * @param None
         *
         * @return None
         */
        void CalibrateHDSet ();

        /**
         * @brief Do the work, update the location
         *
         * @param None
         *
         * @return None
         */
        void GridLocation (double angularVelocityY);

        /**
         * @brief Helper function
         * 
         * @param None
         *
         * @return None
         */
        void CalibrateGridCellSet ();

        /* ====================  DATA MEMBERS  ======================================= */
        HDCells* mpHDCells;                       /**< Head cell set */
        RotationCellCounterClockwise* mpRotationCellCounterClockwise; /**< Counter clockwise rotation cell */
        RotationCellClockwise* mpRotationCellClockwise; /**< Clockwise rotation cell */
        VisionCells* mpVisionCells;               /**< Vision cell set */
        GridCells* mpGridCells;               /**< Grid cells */
        VelocityCell* mpVelocityCell;           /**< Velocity cell */


        HDSynapseSet* mpHDSynapseSet;             /**< HD synapse set */
        HD_VisionSynapseSet* mpHD_VisionSynapseSet; /**< HD - Vision synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellClockwiseSynapseSet; /**< HD - Clockise rotation cell synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellCounterClockwiseSynapseSet; /**< HD - counter clockwise rotation cell synapse set */
        GridCellsSynapseSet* mpGridCellsSynapseSet; /**< Grid cells synapse set */
        GridCells_VisionSynapseSet* mpGridCells_VisionSynapseSet; /**< Grid cells - vision cell synapse set */
        std::vector<GridCells_HD_VelocitySynapseSet*> mpGridCells_HD_VelocitySynapseSet; /**< Grid cells - velocity cell synapse sets.*/
        //GridCells_HD_VelocitySynapseSet* mpGridCells_HD_VelocitySynapseSet; /**< Grid cells - velocity cell synapse set*/

        ros::Subscriber mSubscriber;            /**< ROS subscriber handle */
        ros::NodeHandle mNodeHandle;            /**< ROS Node Handle */
        ros::Publisher mHeadDirectionPublisher; /**< ROS publisher handle for head direction */
        ros::Publisher mLocationPublisher;      /**< ROS publisher handle for location */


        int mCount;                     /**< Keep a count of number of IMU messages we've processed */
        int mCountTillFreq;
        int mProcessFreq;
        int mPositive;
        int mNegative;

        /*  Head direction specific declarations */
        bool mIsInitialDirectionSet;              /**< Is the network initialized to an initial heading */
        double mHeadDirection;             /**< The head direction */
        double mHeadDirectionPrev;             /**< The head direction */
        double mInitialHeading;
        double mSigmaHD;
        double mScale;                          /**< Scale the firing rate for calibration */
        bool mIsHDCalibrated;                     /**< Is the HD set calibrated */

        /*  Grid cell specific declarations */
        struct location {
            double x;
            double y;
        };
        struct location mLocation;
        struct location mInitialLocation;
        struct location mLocationPrev;
        bool mIsInitialLocationSet;
        double mSigmaG;
        bool mIsGridCellSetCalibrated;                     /**< Is the grid cell set calibrated */

        double mVelocity;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mGridHeatMap;    /**< Trace firing rates of neurons */
        std::ofstream mDebugFile;
}; /* -----  end of class Bionavigator  ----- */


#endif   /* ----- #ifndef Bionavigator_INC  ----- */

