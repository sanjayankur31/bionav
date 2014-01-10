/*
 * =====================================================================================
 *
 *       Filename:  PlaceCellSynapseSet.cpp
 *
 *    Description:  Definition file for PlaceCellSynapseSet.hpp
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

#include "PlaceCellSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellSynapseSet
 *      Method:  PlaceCellSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellSynapseSet::PlaceCellSynapseSet ()
{
    mDimensionX = 100;
    mDimensionY = 100;
    mIdentifier = std::string("HDSynapses");
}  /* -----  end of method PlaceCellSynapseSet::PlaceCellSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellSynapseSet
 *      Method:  PlaceCellSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellSynapseSet::PlaceCellSynapseSet ( const PlaceCellSynapseSet &other )
{
}  /* -----  end of method PlaceCellSynapseSet::PlaceCellSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellSynapseSet
 *      Method:  ~PlaceCellSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellSynapseSet::~PlaceCellSynapseSet ()
{

}  /* -----  end of method PlaceCellSynapseSet::~PlaceCellSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    PlaceCellSynapseSet&
PlaceCellSynapseSet::operator = ( const PlaceCellSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method PlaceCellSynapseSet::operator =  (assignment operator)  ----- */
