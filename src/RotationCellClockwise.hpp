/*
 * =====================================================================================
 *
 *       Filename:  RotationCellClockwise.hpp
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


#ifndef  RotationCellClockwise_INC
#define  RotationCellClockwise_INC

#include "NeuronSet.hpp"
#include "std_msgs/Float64.h"
/*
 * =====================================================================================
 *        Class:  RotationCellClockwise
 *  Description:  A clockwise rotation cell
 * =====================================================================================
 */
/**
 * @class RotationCellClockwise
 *
 * @brief A class for a clockwise rotation cell
 *
 * Use an instance of this class for clockwise rotation cells
 * 
 */
class RotationCellClockwise: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        RotationCellClockwise ();                             /* constructor      */
        RotationCellClockwise ( const RotationCellClockwise &other );   /* copy constructor */
        ~RotationCellClockwise ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        virtual void UpdateFiringRate () {} 
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

        RotationCellClockwise& operator = ( const RotationCellClockwise &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class RotationCellClockwise  ----- */


#endif   /* ----- #ifndef RotationCellClockwise_INC  ----- */
