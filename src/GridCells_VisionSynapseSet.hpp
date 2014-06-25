/*
 * =====================================================================================
 *
 *       Filename:  GridCells_VisionSynapseSet.hpp
 *
 *    Description:  The GridCells_Vision synapse set class file
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

#ifndef  GridCells_VisionSynapseSet_INC
#define  GridCells_VisionSynapseSet_INC


#include "SynapseSet.hpp"
#include <Eigen/Dense>
/*
 * =====================================================================================
 *        Class:  GridCells_VisionSynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class GridCells_VisionSynapseSet
 *
 * @brief This class represents the synapses between the visual cells and grid
 * cell system
 * 
 * @todo This is under WIP. The template arguments are only written in to let
 * it compile properly
 */
class GridCells_VisionSynapseSet: public Bionav::SynapseSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        GridCells_VisionSynapseSet ();                             /* constructor      */
        GridCells_VisionSynapseSet ( const GridCells_VisionSynapseSet &other );   /* copy constructor */
        ~GridCells_VisionSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        GridCells_VisionSynapseSet& operator = ( const GridCells_VisionSynapseSet &other ); /* assignment operator */

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

}; /* -----  end of class GridCells_VisionSynapseSet  ----- */

#endif   /* ----- #ifndef GridCells_VisionSynapseSet_INC  ----- */
