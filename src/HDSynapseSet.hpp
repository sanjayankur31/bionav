/*
 * =====================================================================================
 *
 *       Filename:  HDSynapseSet.hpp
 *
 *    Description:  The recurrent synapses of the Head direction neuron set
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

#ifndef  HDSynapseSet_INC
#define  HDSynapseSet_INC

#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  HDSynapseSet
 *  Description:  Class representing the synapse set in the head direction cell set
 * =====================================================================================
 */
/**
 * @class HDSynapseSet
 *
 * @brief This class represents the recurrent synapse set in the head direction
 * set ensemble
 * 
 */
class HDSynapseSet: public Bionav::SynapseSet<Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<long double, Eigen::Dynamic, 1>,Eigen::Matrix<long double, Eigen::Dynamic, 1> >
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HDSynapseSet ();                             /* constructor      */
        HDSynapseSet ( const HDSynapseSet &other );   /* copy constructor */
        ~HDSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        HDSynapseSet& operator = ( const HDSynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class HDSynapseSet  ----- */


#endif   /* ----- #ifndef HDSynapseSet_INC  ----- */
