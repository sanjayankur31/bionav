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
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


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
         * @brief Publish current direction
         *
         * @todo I'll need to add more methods as I look to implement land mark
         * navigation
         *
         * @param rImuMessage reference to the received message
         *
         * @return None
         *
         * @todo When I work on the vision part, I'll need to use a message
         * filter TimeSynchronizer to keep the inputs from these nodes in sync
         *
         * http://wiki.ros.org/message_filters
         */
        void CallbackPublishDirection (const sensor_msgs::Imu::ConstPtr& rImuMessage);


    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /**
         * @brief initialize the system to an initial heading
         *
         * Basically, create a peak and let it stabilize so we can beging to
         * process angular velocity inputs.
         *
         * @param None
         *
         * @return None
         */
        void SetInitialDirection ();

        /**
         * @brief Do the work, update the head direction
         *
         * @param None
         *
         * @return mHeadDirection the updated head direction
         */
        void HeadDirection ();

        /* ====================  DATA MEMBERS  ======================================= */
        HDCells* mpHDCells;                       /**< Head cell set */
        RotationCellCounterClockwise* mpRotationCellCounterClockwise; /**< Counter clockwise rotation cell */
        RotationCellClockwise* mpRotationCellClockwise; /**< Clockwise rotation cell */
        VisionCells* mpVisionCells;               /**< Vision cell set */

        HDSynapseSet* mpHDSynapseSet;             /**< HD synapse set */
        HD_VisionSynapseSet* mpHD_VisionSynapseSet; /**< HD - Vision synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellClockwiseSynapseSet; /**< HD - Clockise rotation cell synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellCounterClockwiseSynapseSet; /**< HD - counter clockwise rotation cell synapse set */

        ros::Subscriber mSubscriber;            /**< ROS subscriber handle */
        ros::NodeHandle mNodeHandle;            /**< ROS Node Handle */
        ros::Publisher mHeadDirectionPublisher; /**< ROS publisher handle */

        bool mIsInitialDirectionSet;              /**< Is the network initialized to an initial heading */
        bool mIsCalibrated;                     /**< Is the network calibrated */
        long double mCount;                     /**< Keep a count of number of IMU messages we've processed */
        long double mHeadDirection;             /**< The head direction */

}; /* -----  end of class Bionavigator  ----- */


#endif   /* ----- #ifndef Bionavigator_INC  ----- */

