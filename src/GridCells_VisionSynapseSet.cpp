/*
 * =====================================================================================
 *
 *       Filename:  GridCells_VisionSynapseSet.cpp
 *
 *    Description:  Definition file for GridCells_VisionSynapseSet
 *
 *        Version:  1.0
 *        Created:  10/10/13 11:51:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */

#include "GridCells_VisionSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells_VisionSynapseSet
 *      Method:  GridCells_VisionSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
GridCells_VisionSynapseSet::GridCells_VisionSynapseSet ()
{
    mLearningRate = 1;
}  /* -----  end of method GridCells_VisionSynapseSet::GridCells_VisionSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells_VisionSynapseSet
 *      Method:  GridCells_VisionSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
GridCells_VisionSynapseSet::GridCells_VisionSynapseSet ( const GridCells_VisionSynapseSet &other )
{
}  /* -----  end of method GridCells_VisionSynapseSet::GridCells_VisionSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells_VisionSynapseSet
 *      Method:  ~GridCells_VisionSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
GridCells_VisionSynapseSet::~GridCells_VisionSynapseSet ()
{
}  /* -----  end of method GridCells_VisionSynapseSet::~GridCells_VisionSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells_VisionSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    GridCells_VisionSynapseSet&
GridCells_VisionSynapseSet::operator = ( const GridCells_VisionSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method GridCells_VisionSynapseSet::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells_VisionSynapseSet
 *      Method:  GridCells_VisionSynapseSet :: Normalize
 * Description:  
 *--------------------------------------------------------------------------------------
 */
void 
GridCells_VisionSynapseSet::Normalize (){
    /*  Normalize each row individually */
    mDeltaW = mWeightMatrix/mWeightMatrix.norm ();
    mWeightMatrix = mDeltaW;
    ROS_DEBUG("%s: Synaptic weight normalized to [%f,%f]", mIdentifier.c_str (), mWeightMatrix.maxCoeff (), mWeightMatrix.minCoeff ());
}/* -----  end of method GridCells_VisionSynapseSet::Normalize  ----- */

