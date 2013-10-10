/*
 * =====================================================================================
 *
 *       Filename:  HD_RotationSynapseSet.hpp
 *
 *    Description:  Class that defines the synapses between the head cells and
 *    rotation cells
 *
 *        Version:  1.0
 *        Created:  09/09/2013 07:17:15 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  HD_RotationSynapseSet_INC
#define  HD_RotationSynapseSet_INC

#include <Eigen/Dense>
#include "SynapseSet.hpp"

/*
 * =====================================================================================
 *        Class:  HD_RotationSynapseSet
 *  Description:  
 * =====================================================================================
 */
/**
 * @class HD_RotationSynapseSet
 *
 * @brief Class representing synapses between head cells and the rotation cells
 * 
 */
class HD_RotationSynapseSet: public Bionav::SynapseSet<Eigen::Matrix<long double, Eigen::Dynamic, 1>, long double, Eigen::Matrix<long double, Eigen::Dynamic, 1> >
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HD_RotationSynapseSet ();                             /* constructor      */
        HD_RotationSynapseSet ( const HD_RotationSynapseSet &other );   /* copy constructor */
        ~HD_RotationSynapseSet ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        virtual void UpdateWeight(long double preSynapticFiringRate, Eigen::Matrix<long double, Eigen::Dynamic, 1> postSynapticFiringRate) { }
        void Init ();

        /* ====================  OPERATORS     ======================================= */

        HD_RotationSynapseSet& operator = ( const HD_RotationSynapseSet &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class HD_RotationSynapseSet  ----- */


#endif   /* ----- #ifndef HD_RotationSynapseSet_INC  ----- */
