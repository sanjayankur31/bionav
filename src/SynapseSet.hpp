/*
 * =====================================================================================
 *
 *       Filename:  SynapseSet.hpp
 *
 *    Description:  Header for SynapseSet class
 *
 *        Version:  1.0
 *        Created:  06/09/13 14:50:37
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef  SynapseSet_INC
#define  SynapseSet_INC

namespace Bionav {

    /*
     * =====================================================================================
     *        Class:  SynapseSet
     *  Description:  
     * =====================================================================================
     */
    /**
     * @class SynapseSet
     * @brief The class represents a set of synapses between two neuron sets.
     * Ideally, each synapse should be modelled separately, but in our case we
     * don't deal with individual synapses. We deal with sets of synapses
     * between sets of neurons.
     */
    class SynapseSet
    {
        public:
            /* ====================  LIFECYCLE     ======================================= */
            SynapseSet ();                             /* constructor */

            /* ====================  ACCESSORS     ======================================= */

            /* ====================  MUTATORS      ======================================= */

            /* ====================  OPERATORS     ======================================= */

        protected:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class SynapseSet  ----- */

}
#endif   /* ----- #ifndef SynapseSet_INC  ----- */
