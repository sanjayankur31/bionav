/*
 * =====================================================================================
 *
 *       Filename:  VelocityCell.hpp
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


#ifndef  VelocityCell_INC
#define  VelocityCell_INC

#include "NeuronSet.hpp"
#include "std_msgs/Float64.h"
/*
 * =====================================================================================
 *        Class:  VelocityCell
 *  Description:  A velocity cell
 * =====================================================================================
 */
/**
 * @class VelocityCell
 *
 * @brief A class for a velocity cell
 *
 */
class VelocityCell: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        VelocityCell ();                             /* constructor      */
        VelocityCell ( const VelocityCell &other );   /* copy constructor */
        ~VelocityCell ();                            /* destructor       */

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

        VelocityCell& operator = ( const VelocityCell &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class VelocityCell  ----- */


#endif   /* ----- #ifndef VelocityCell_INC  ----- */
