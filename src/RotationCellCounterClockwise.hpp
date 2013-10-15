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

        /* ====================  OPERATORS     ======================================= */

        RotationCellCounterClockwise& operator = ( const RotationCellCounterClockwise &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class RotationCellCounterClockwise  ----- */


#endif   /* ----- #ifndef RotationCellCounterClockwise_INC  ----- */
