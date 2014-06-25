/*
 * =====================================================================================
 *
 *       Filename:  GridCellsSynapseSet.hpp
 *
 *    Description:  The recurrent synapses of the grid neuron set
 *
 *        Version:  1.0
 *        Created:  09/09/2013 07:02:06 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  GridCellsSynapseSet_INC
#define  GridCellsSynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  GridCellsSynapseSet
 *  Description:  Class representing the synapse set in the grid cell set
 * =====================================================================================
 */
/**
 * @class GridCellsSynapseSet
 *
 * @brief This class represents the recurrent synapse set in the grid
 * set ensemble
 * 
 */
class GridCellsSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        GridCellsSynapseSet ();                             /* constructor      */
        GridCellsSynapseSet ( const GridCellsSynapseSet &other );   /* copy constructor */
        ~GridCellsSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        GridCellsSynapseSet& operator = ( const GridCellsSynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class GridCellsSynapseSet  ----- */


#endif   /* ----- #ifndef GridCellsSynapseSet_INC  ----- */
