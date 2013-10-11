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

#include <Eigen/Dense>
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
class HDSynapseSet: public Bionav::SynapseSet<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<double, Eigen::Dynamic, 1>,Eigen::Matrix<double, Eigen::Dynamic, 1> >
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HDSynapseSet ();                             /* constructor      */
        HDSynapseSet ( const HDSynapseSet &other );   /* copy constructor */
        ~HDSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        virtual void UpdateWeight (Eigen::Matrix<double, Eigen::Dynamic, 1> preSynapticFiringRate,Eigen::Matrix<double, Eigen::Dynamic, 1> postSynapticFiringRate );

        /* ====================  OPERATORS     ======================================= */

        HDSynapseSet& operator = ( const HDSynapseSet &other ); /* assignment operator */

        /**
         * @brief Initialize the matrices.
         *
         * The constructor sets a default dimension set. One can modify it and
         * then call Init to resize and initialize the matrices accordingly
         *
         * This assumes that you've set your dimensions. It does no checks at
         * all.
         *
         * @param None
         *
         * @return None
         */
        void Init ();
    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class HDSynapseSet  ----- */


#endif   /* ----- #ifndef HDSynapseSet_INC  ----- */
