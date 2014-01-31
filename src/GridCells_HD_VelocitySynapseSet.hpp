/*
 * =====================================================================================
 *
 *       Filename:  GridCells_HD_VelocitySynapseSet.hpp
 *
 *    Description:  Class that defines the synapses between the grid cells and
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

#ifndef  GridCells_HD_VelocitySynapseSet_INC
#define  GridCells_HD_VelocitySynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  GridCells_HD_VelocitySynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class GridCells_HD_VelocitySynapseSet
 *
 * @brief Class representing synapses between grid cells and the velocity cells
 * 
 */
class GridCells_HD_VelocitySynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        GridCells_HD_VelocitySynapseSet ();                             /* constructor      */
        GridCells_HD_VelocitySynapseSet ( const GridCells_HD_VelocitySynapseSet &other );   /* copy constructor */
        ~GridCells_HD_VelocitySynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        /* ====================  OPERATORS     ======================================= */

        GridCells_HD_VelocitySynapseSet& operator = ( const GridCells_HD_VelocitySynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class GridCells_HD_VelocitySynapseSet  ----- */


#endif   /* ----- #ifndef GridCells_HD_VelocitySynapseSet_INC  ----- */
