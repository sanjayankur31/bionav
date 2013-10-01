/*
 * =====================================================================================
 *
 *       Filename:  Bionavigator.cpp
 *
 *    Description:  Main definition file for main Bionavigator class
 *
 *        Version:  1.0
 *        Created:  19/09/13 11:52:12
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#include "Bionavigator.hpp"

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
Bionavigator::Bionavigator ()
{
        int head_cell_dimension_x = 100;
        int head_cell_dimension_y = 1;

        mpHDCells = new HDCells ();
        mpRotationCellCounterClockwise  = new RotationCellCounterClockwise ();
        mpRotationCellClockwise = new RotationCellClockwise ();
        mpVisionCells = new VisionCells ();

        mpHDSynapseSet = new HDSynapseSet ();
        mpHD_VisionSynapseSet = new HD_VisionSynapseSet ();
        mpHD_RotationCellClockwiseSynapseSet = new HD_RotationSynapseSet ();
        mpHD_RotationCellCounterClockwiseSynapseSet = new HD_RotationSynapseSet ();

        mpHDCells->SetDimension (head_cell_dimension_x, head_cell_dimension_y);
        mpHDCells->Init ();
        /*  Don't need to do this for rotation cells, since they are two
         *  individual cells at the moment
         */

        mpHDSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_x);
        mpHD_RotationCellCounterClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_y);
        mpHD_RotationCellClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_y);

        /**
         * @todo Vision cell initialization
         * 
         */


}  /* -----  end of method Bionavigator::Bionavigator  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
Bionavigator::Bionavigator ( const Bionavigator &other )
{
}  /* -----  end of method Bionavigator::Bionavigator  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  ~Bionavigator
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
Bionavigator::~Bionavigator ()
{

    /*  delete my brain elements */
    delete mpHDCells;
    delete mpRotationCellCounterClockwise;
    delete mpRotationCellCounterClockwise;
    delete mpVisionCells;

    delete mpHDSynapseSet;
    delete mpHD_VisionSynapseSet;
    delete mpHD_RotationCellClockwiseSynapseSet;
    delete mpHD_RotationCellCounterClockwiseSynapseSet;
}  /* -----  end of method Bionavigator::~Bionavigator  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    Bionavigator&
Bionavigator::operator = ( const Bionavigator &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method Bionavigator::operator =  (assignment operator)  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: Init
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::Init (  )
{
    return ;
}		/* -----  end of method Bionavigator::Init  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: Calibrate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::Calibrate (  )
{
    return ;
}		/* -----  end of method Bionavigator::Calibrate  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: UpdateState
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::UpdateState ( )
{
    return ;
}		/* -----  end of method Bionavigator::UpdateState  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: PublishDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::PublishDirection (  )
{
    return ;
}		/* -----  end of method Bionavigator::PublishDirection  ----- */

