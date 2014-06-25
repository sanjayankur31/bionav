/*
 * =====================================================================================
 *
 *       Filename:  PlaceCells_VisionSynapseSet.cpp
 *
 *    Description:  Definition file for PlaceCells_VisionSynapseSet
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

#include "PlaceCells_VisionSynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCells_VisionSynapseSet
 *      Method:  PlaceCells_VisionSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCells_VisionSynapseSet::PlaceCells_VisionSynapseSet ()
{
    mLearningRate = 1;
}  /* -----  end of method PlaceCells_VisionSynapseSet::PlaceCells_VisionSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCells_VisionSynapseSet
 *      Method:  PlaceCells_VisionSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
PlaceCells_VisionSynapseSet::PlaceCells_VisionSynapseSet ( const PlaceCells_VisionSynapseSet &other )
{
}  /* -----  end of method PlaceCells_VisionSynapseSet::PlaceCells_VisionSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCells_VisionSynapseSet
 *      Method:  ~PlaceCells_VisionSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
PlaceCells_VisionSynapseSet::~PlaceCells_VisionSynapseSet ()
{
}  /* -----  end of method PlaceCells_VisionSynapseSet::~PlaceCells_VisionSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCells_VisionSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    PlaceCells_VisionSynapseSet&
PlaceCells_VisionSynapseSet::operator = ( const PlaceCells_VisionSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method PlaceCells_VisionSynapseSet::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  PlaceCells_VisionSynapseSet
 *      Method:  PlaceCells_VisionSynapseSet :: Normalize
 * Description:  
 *--------------------------------------------------------------------------------------
 */
void 
PlaceCells_VisionSynapseSet::Normalize (){
    /*  Normalize each row individually */
    mDeltaW = mWeightMatrix/mWeightMatrix.norm ();
    mWeightMatrix = mDeltaW;
    ROS_DEBUG("%s: Synaptic weight normalized to [%f,%f]", mIdentifier.c_str (), mWeightMatrix.maxCoeff (), mWeightMatrix.minCoeff ());
}/* -----  end of method PlaceCells_VisionSynapseSet::Normalize  ----- */

