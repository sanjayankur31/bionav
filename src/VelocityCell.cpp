/*
 * =====================================================================================
 *
 *       Filename:  VelocityCell.cpp
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


#include "VelocityCell.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  VelocityCell
 *      Method:  VelocityCell
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
VelocityCell::VelocityCell ()
{
    mBeta = 0.5;
    mAlpha = 0;
}  /* -----  end of method VelocityCell::VelocityCell  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VelocityCell
 *      Method:  VelocityCell
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
VelocityCell::VelocityCell ( const VelocityCell &other )
{
}  /* -----  end of method VelocityCell::VelocityCell  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VelocityCell
 *      Method:  ~VelocityCell
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
VelocityCell::~VelocityCell ()
{
}  /* -----  end of method VelocityCell::~VelocityCell  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VelocityCell
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    VelocityCell&
VelocityCell::operator = ( const VelocityCell &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method VelocityCell::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  VelocityCell
 *      Method:  VelocityCell :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
VelocityCell::UpdateFiringRate (double velocity )
{
    double temp = (1.0/(1.0 +( exp(-2.0 * mBeta * (velocity -mAlpha)))));
    //ROS_DEBUG_STREAM (mIdentifier << ": Firing rate is: " << mFiringRate);
    if (temp > 0)
        mFiringRate << temp;
    else 
        mFiringRate << 0;

    return mFiringRate;
}		/* -----  end of method VelocityCell::UpdateFiringRate  ----- */

