/*
 * =====================================================================================
 *
 *       Filename:  PlaceCell_VelocitySynapseSet.hpp
 *
 *    Description:  Class that defines the synapses between the place cells and
 *    velocity cells
 *
 *        Version:  1.0
 *        Created:  09/09/2013 07:17:15 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  PlaceCell_VelocitySynapseSet_INC
#define  PlaceCell_VelocitySynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  PlaceCell_VelocitySynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class PlaceCell_VelocitySynapseSet
 *
 * @brief Class representing synapses between place cells and the velocity cells
 * 
 */
class PlaceCell_VelocitySynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCell_VelocitySynapseSet ();                             /* constructor      */
        PlaceCell_VelocitySynapseSet ( const PlaceCell_VelocitySynapseSet &other );   /* copy constructor */
        ~PlaceCell_VelocitySynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        /* ====================  OPERATORS     ======================================= */

        PlaceCell_VelocitySynapseSet& operator = ( const PlaceCell_VelocitySynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class PlaceCell_VelocitySynapseSet  ----- */


#endif   /* ----- #ifndef PlaceCell_VelocitySynapseSet_INC  ----- */
