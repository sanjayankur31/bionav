/*
 * =====================================================================================
 *
 *       Filename:  PlaceCells_VisionSynapseSet.hpp
 *
 *    Description:  The PlaceCells_Vision synapse set class file
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

#ifndef  PlaceCells_VisionSynapseSet_INC
#define  PlaceCells_VisionSynapseSet_INC


#include "SynapseSet.hpp"
#include <Eigen/Dense>
/*
 * =====================================================================================
 *        Class:  PlaceCells_VisionSynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class PlaceCells_VisionSynapseSet
 *
 * @brief This class represents the synapses between the visual cells and place
 * cell system
 * 
 * @todo This is under WIP. The template arguments are only written in to let
 * it compile properly
 */
class PlaceCells_VisionSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        PlaceCells_VisionSynapseSet ();                             /* constructor      */
        PlaceCells_VisionSynapseSet ( const PlaceCells_VisionSynapseSet &other );   /* copy constructor */
        ~PlaceCells_VisionSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        PlaceCells_VisionSynapseSet& operator = ( const PlaceCells_VisionSynapseSet &other ); /* assignment operator */

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

}; /* -----  end of class PlaceCells_VisionSynapseSet  ----- */

#endif   /* ----- #ifndef PlaceCells_VisionSynapseSet_INC  ----- */
