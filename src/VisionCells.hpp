/*
 * =====================================================================================
 *
 *       Filename:  VisionCells.hpp
 *
 *    Description:  Header file for vision cells
 *
 *        Version:  1.0
 *        Created:  19/09/13 11:43:48
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef  VisionCells_INC
#define  VisionCells_INC

#include "NeuronSet.hpp"

/*
 * =====================================================================================
 *        Class:  VisionCells
 *  Description:  Class representing the visual cortex
 * =====================================================================================
 */
/**
 * @class VisionCells
 *
 * @brief Class representing the processing units of the visual cortex
 *
 * The class is only a high level functional implementation. A complete
 * implementation is out of the scope for this project currently
 * 
 */
class VisionCells: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        VisionCells ();                             /* constructor      */
        VisionCells ( const VisionCells &other );   /* copy constructor */
        ~VisionCells ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        virtual void UpdateFiringRate () { }
        virtual void UpdateFiringRateTrace () { }


        /* ====================  OPERATORS     ======================================= */

        VisionCells& operator = ( const VisionCells &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class VisionCells  ----- */

#endif   /* ----- #ifndef VisionCells_INC  ----- */
