/*
 * =====================================================================================
 *
 *       Filename:  RotationCellCounterClockwise.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  09/09/2013 05:08:46 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */


#ifndef  RotationCellCounterClockwise_INC
#define  RotationCellCounterClockwise_INC

#include "NeuronSet.hpp"
#include "std_msgs/Float64.h"

/*
 * =====================================================================================
 *        Class:  RotationCellCounterClockwise
 *  Description:  A counter clockwise rotation cell
 * =====================================================================================
 */
/**
 * @class RotationCellCounterClockwise
 *
 * @brief A class for a counter clockwise rotation cell
 *
 * Use an instance of this class for counter clockwise rotation cells
 * 
 */
class RotationCellCounterClockwise: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        RotationCellCounterClockwise ();                             /* constructor      */
        RotationCellCounterClockwise ( const RotationCellCounterClockwise &other );    /* copy constructor */
        ~RotationCellCounterClockwise ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        virtual void UpdateFiringRate () { }
        virtual void UpdateFiringRateTrace () { }

        /**
         * @brief Update firing rate based on inputs from vestibules
         *
         * I don't need a firing rate method that depends on activation for
         * rotation cells. They just take in angular velocity inputs and spew
         * out accordingly.
         * 
         * @param angularVelocty input angular velocity from robots imu system
         *
         * @return firing rate calculated from inputs
         */
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UpdateFiringRate (double angularVelocity );

        /* ====================  OPERATORS     ======================================= */

        RotationCellCounterClockwise& operator = ( const RotationCellCounterClockwise &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        double mAlpha;
        double mBeta;

}; /* -----  end of class RotationCellCounterClockwise  ----- */


#endif   /* ----- #ifndef RotationCellCounterClockwise_INC  ----- */
