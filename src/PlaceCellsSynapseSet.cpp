/*
 * =====================================================================================
 *
 *       Filename:  PlaceCellsSynapseSet.cpp
 *
 *    Description:  Definition file for PlaceCellsSynapseSet.hpp
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

#include "PlaceCellsSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellsSynapseSet
 *      Method:  PlaceCellsSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellsSynapseSet::PlaceCellsSynapseSet ()
{
    mDimensionX = 100;
    mDimensionY = 100;
    mIdentifier = std::string("HDSynapses");
}  /* -----  end of method PlaceCellsSynapseSet::PlaceCellsSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellsSynapseSet
 *      Method:  PlaceCellsSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellsSynapseSet::PlaceCellsSynapseSet ( const PlaceCellsSynapseSet &other )
{
}  /* -----  end of method PlaceCellsSynapseSet::PlaceCellsSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellsSynapseSet
 *      Method:  ~PlaceCellsSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
PlaceCellsSynapseSet::~PlaceCellsSynapseSet ()
{

}  /* -----  end of method PlaceCellsSynapseSet::~PlaceCellsSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCellsSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    PlaceCellsSynapseSet&
PlaceCellsSynapseSet::operator = ( const PlaceCellsSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method PlaceCellsSynapseSet::operator =  (assignment operator)  ----- */
