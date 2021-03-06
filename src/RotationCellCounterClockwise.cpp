/*
 * =====================================================================================
 *
 *       Filename:  RotationCellCounterClockwise.cpp
 *
 *    Description:  Definition File
 *
 *        Version:  1.0
 *        Created:  10/10/13 11:57:39
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */

#include "RotationCellCounterClockwise.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellCounterClockwise
 *      Method:  RotationCellCounterClockwise
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
RotationCellCounterClockwise::RotationCellCounterClockwise ()
{
    mBeta = 1;
    mAlpha = 0;
}  /* -----  end of method RotationCellCounterClockwise::RotationCellCounterClockwise  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellCounterClockwise
 *      Method:  RotationCellCounterClockwise
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
RotationCellCounterClockwise::RotationCellCounterClockwise ( const RotationCellCounterClockwise &other )
{
}  /* -----  end of method RotationCellCounterClockwise::RotationCellCounterClockwise  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellCounterClockwise
 *      Method:  ~RotationCellCounterClockwise
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
RotationCellCounterClockwise::~RotationCellCounterClockwise ()
{
}  /* -----  end of method RotationCellCounterClockwise::~RotationCellCounterClockwise  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellCounterClockwise
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    RotationCellCounterClockwise&
RotationCellCounterClockwise::operator = ( const RotationCellCounterClockwise &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method RotationCellCounterClockwise::operator =  (assignment operator)  ----- */
/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellCounterClockwise
 *      Method:  RotationCellCounterClockwise :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
RotationCellCounterClockwise::UpdateFiringRate (double angularVelocity )
{
    /*
     * Greater than 0 is clockwise for me
     */
    if (angularVelocity > 0)
    {
        double temp = (1.0/(1.0 +( exp(-2.0 * mBeta * ((-1.0 * angularVelocity) -mAlpha)))));

        if (temp > 0)
            mFiringRate << temp;
        else 
            mFiringRate << 0;
    } 
    else 
        mFiringRate << 0;
    //ROS_DEBUG_STREAM (mIdentifier << ": Firing rate is: " << mFiringRate);

    return mFiringRate;
}		/* -----  end of method RotationCellCounterClockwise::UpdateFiringRate  ----- */

