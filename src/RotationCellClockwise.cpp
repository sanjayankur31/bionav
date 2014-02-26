/*
 * =====================================================================================
 *
 *       Filename:  RotationCellClockwise.cpp
 *
 *    Description:  Definition file
 *
 *        Version:  1.0
 *        Created:  10/10/13 11:57:00
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */


#include "RotationCellClockwise.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellClockwise
 *      Method:  RotationCellClockwise
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
RotationCellClockwise::RotationCellClockwise ()
{
    mBeta = 1;
    mAlpha = 0;
}  /* -----  end of method RotationCellClockwise::RotationCellClockwise  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellClockwise
 *      Method:  RotationCellClockwise
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
RotationCellClockwise::RotationCellClockwise ( const RotationCellClockwise &other )
{
}  /* -----  end of method RotationCellClockwise::RotationCellClockwise  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellClockwise
 *      Method:  ~RotationCellClockwise
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
RotationCellClockwise::~RotationCellClockwise ()
{
}  /* -----  end of method RotationCellClockwise::~RotationCellClockwise  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellClockwise
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    RotationCellClockwise&
RotationCellClockwise::operator = ( const RotationCellClockwise &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method RotationCellClockwise::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  RotationCellClockwise
 *      Method:  RotationCellClockwise :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
RotationCellClockwise::UpdateFiringRate (double angularVelocity )
{
    /*
     * Greater than 0 is clockwise for me
     */
    if (angularVelocity > 0)
    {
        double temp = (1.0/(1.0 +( exp(-2.0 * mBeta * (angularVelocity -mAlpha)))));

        if (temp > 0)
            mFiringRate << temp;
        else 
            mFiringRate << 0;
    } 
    else 
        mFiringRate << 0;

    //ROS_DEBUG_STREAM (mIdentifier << ": Firing rate is: " << mFiringRate);
    return mFiringRate;
}		/* -----  end of method RotationCellClockwise::UpdateFiringRate  ----- */

