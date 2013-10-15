/*
 * =====================================================================================
 *
 *       Filename:  VisionCells.cpp
 *
 *    Description:  Definition file for VisionCells
 *
 *        Version:  1.0
 *        Created:  19/09/13 11:47:28
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  
 *
 * =====================================================================================
 */


#include "VisionCells.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  VisionCells
 *      Method:  VisionCells
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
VisionCells::VisionCells ()
{
    mForceFiring = false;
}  /* -----  end of method VisionCells::VisionCells  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VisionCells
 *      Method:  VisionCells
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
VisionCells::VisionCells ( const VisionCells &other )
{
}  /* -----  end of method VisionCells::VisionCells  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VisionCells
 *      Method:  ~VisionCells
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
VisionCells::~VisionCells ()
{
}  /* -----  end of method VisionCells::~VisionCells  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  VisionCells
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    VisionCells&
VisionCells::operator = ( const VisionCells &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method VisionCells::operator =  (assignment operator)  ----- */



