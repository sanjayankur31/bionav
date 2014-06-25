/*
 * =====================================================================================
 *
 *       Filename:  PlaceCells_HD_VelocitySynapseSet.hpp
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

#ifndef  PlaceCells_HD_VelocitySynapseSet_INC
#define  PlaceCells_HD_VelocitySynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  PlaceCells_HD_VelocitySynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class PlaceCells_HD_VelocitySynapseSet
 *
 * @brief Class representing synapses between place cells and the velocity cells
 * 
 */
class PlaceCells_HD_VelocitySynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCells_HD_VelocitySynapseSet ();                             /* constructor      */
        PlaceCells_HD_VelocitySynapseSet ( const PlaceCells_HD_VelocitySynapseSet &other );   /* copy constructor */
        ~PlaceCells_HD_VelocitySynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        /* ====================  OPERATORS     ======================================= */

        PlaceCells_HD_VelocitySynapseSet& operator = ( const PlaceCells_HD_VelocitySynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class PlaceCells_HD_VelocitySynapseSet  ----- */


#endif   /* ----- #ifndef PlaceCells_HD_VelocitySynapseSet_INC  ----- */
