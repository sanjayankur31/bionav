/*
 * =====================================================================================
 *
 *       Filename:  PlaceCellSynapseSet.hpp
 *
 *    Description:  The recurrent synapses of the place neuron set
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

#ifndef  PlaceCellSynapseSet_INC
#define  PlaceCellSynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  PlaceCellSynapseSet
 *  Description:  Class representing the synapse set in the place cell set
 * =====================================================================================
 */
/**
 * @class PlaceCellSynapseSet
 *
 * @brief This class represents the recurrent synapse set in the place
 * set ensemble
 * 
 */
class PlaceCellSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCellSynapseSet ();                             /* constructor      */
        PlaceCellSynapseSet ( const PlaceCellSynapseSet &other );   /* copy constructor */
        ~PlaceCellSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        PlaceCellSynapseSet& operator = ( const PlaceCellSynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class PlaceCellSynapseSet  ----- */


#endif   /* ----- #ifndef PlaceCellSynapseSet_INC  ----- */
