/*
 * =====================================================================================
 *
 *       Filename:  HD_VisionSynapseSet.hpp
 *
 *    Description:  The HD_Vision synapse set class file
 *
 *        Version:  1.0
 *        Created:  09/10/2013 12:34:17 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef  HD_VisionSynapseSet_INC
#define  HD_VisionSynapseSet_INC


#include "SynapseSet.hpp"
#include <Eigen/Dense>
/*
 * =====================================================================================
 *        Class:  HD_VisionSynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class HD_VisionSynapseSet
 *
 * @brief This class represents the synapses between the visual cells and head
 * direction cell system
 * 
 * @todo This is under WIP. The template arguments are only written in to let
 * it compile properly
 */
class HD_VisionSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HD_VisionSynapseSet ();                             /* constructor      */
        HD_VisionSynapseSet ( const HD_VisionSynapseSet &other );   /* copy constructor */
        ~HD_VisionSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        HD_VisionSynapseSet& operator = ( const HD_VisionSynapseSet &other ); /* assignment operator */

        /**
         * @brief Normalize weights 
         *
         * A special implementation for this synapse set since it's not a matrix, just
         * a vector really.
         *
         * Divide by norm to normalize. 
         * Using this implies our learning rule is not local any more,
         * since it depends on all neurons in the set for normalization
         *
         * @param None
         *
         * @return void
         */
        void Normalize ();

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class HD_VisionSynapseSet  ----- */

#endif   /* ----- #ifndef HD_VisionSynapseSet_INC  ----- */
