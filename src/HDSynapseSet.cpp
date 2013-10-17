/*
 * =====================================================================================
 *
 *       Filename:  HDSynapseSet.cpp
 *
 *    Description:  Definition file for HDSynapseSet.hpp
 *
 *        Version:  1.0
 *        Created:  26/09/13 16:34:58
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#include "HDSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::HDSynapseSet ()
{
    mDimensionX = 100;
    mDimensionY = 100;
    mIdentifier = std::string("HDSynapses");
}  /* -----  end of method HDSynapseSet::HDSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::HDSynapseSet ( const HDSynapseSet &other )
{
}  /* -----  end of method HDSynapseSet::HDSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  ~HDSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::~HDSynapseSet ()
{

}  /* -----  end of method HDSynapseSet::~HDSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    HDSynapseSet&
HDSynapseSet::operator = ( const HDSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method HDSynapseSet::operator =  (assignment operator)  ----- */
