/*
 * =====================================================================================
 *
 *       Filename:  PlaceCellsSynapseSet.hpp
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

#ifndef  PlaceCellsSynapseSet_INC
#define  PlaceCellsSynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  PlaceCellsSynapseSet
 *  Description:  Class representing the synapse set in the place cell set
 * =====================================================================================
 */
/**
 * @class PlaceCellsSynapseSet
 *
 * @brief This class represents the recurrent synapse set in the place
 * set ensemble
 * 
 */
class PlaceCellsSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCellsSynapseSet ();                             /* constructor      */
        PlaceCellsSynapseSet ( const PlaceCellsSynapseSet &other );   /* copy constructor */
        ~PlaceCellsSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        PlaceCellsSynapseSet& operator = ( const PlaceCellsSynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class PlaceCellsSynapseSet  ----- */


#endif   /* ----- #ifndef PlaceCellsSynapseSet_INC  ----- */
