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
 */
class HD_VisionSynapseSet: public Bionav::SynapseSet<>
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

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class HD_VisionSynapseSet  ----- */

#endif   /* ----- #ifndef HD_VisionSynapseSet_INC  ----- */
