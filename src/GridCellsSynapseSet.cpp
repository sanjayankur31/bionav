/*
 * =====================================================================================
 *
 *       Filename:  GridCellsSynapseSet.cpp
 *
 *    Description:  Definition file for GridCellsSynapseSet.hpp
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

#include "GridCellsSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCellsSynapseSet
 *      Method:  GridCellsSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
GridCellsSynapseSet::GridCellsSynapseSet ()
{
    mDimensionX = 100;
    mDimensionY = 100;
    mIdentifier = std::string("GridCellsSynapseSet");
}  /* -----  end of method GridCellsSynapseSet::GridCellsSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCellsSynapseSet
 *      Method:  GridCellsSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
GridCellsSynapseSet::GridCellsSynapseSet ( const GridCellsSynapseSet &other )
{
}  /* -----  end of method GridCellsSynapseSet::GridCellsSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCellsSynapseSet
 *      Method:  ~GridCellsSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
GridCellsSynapseSet::~GridCellsSynapseSet ()
{

}  /* -----  end of method GridCellsSynapseSet::~GridCellsSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCellsSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    GridCellsSynapseSet&
GridCellsSynapseSet::operator = ( const GridCellsSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method GridCellsSynapseSet::operator =  (assignment operator)  ----- */
