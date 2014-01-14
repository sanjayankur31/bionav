/*
 * =====================================================================================
 *
 *       Filename:  PlaceCells_VelocitySynapseSet.hpp
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

#ifndef  PlaceCells_VelocitySynapseSet_INC
#define  PlaceCells_VelocitySynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  PlaceCells_VelocitySynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class PlaceCells_VelocitySynapseSet
 *
 * @brief Class representing synapses between place cells and the velocity cells
 * 
 */
class PlaceCells_VelocitySynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCells_VelocitySynapseSet ();                             /* constructor      */
        PlaceCells_VelocitySynapseSet ( const PlaceCells_VelocitySynapseSet &other );   /* copy constructor */
        ~PlaceCells_VelocitySynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        /* ====================  OPERATORS     ======================================= */

        PlaceCells_VelocitySynapseSet& operator = ( const PlaceCells_VelocitySynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class PlaceCells_VelocitySynapseSet  ----- */


#endif   /* ----- #ifndef PlaceCells_VelocitySynapseSet_INC  ----- */
