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
    mpHDCells = new HDCells ();
    mpRotationCellCounterClockwise  = new RotationCellCounterClockwise ();
    mpRotationCellClockwise = new RotationCellClockwise ();
    mpVisionCells = new VisionCells ();
    mpGridCells = new GridCells ();
    mpVelocityCell = new VelocityCell ();

    mpHDSynapseSet = new HDSynapseSet ();
    mpGridCellsSynapseSet = new GridCellsSynapseSet ();

    mpHD_VisionSynapseSet = new HD_VisionSynapseSet ();
    mpGridCells_VisionSynapseSet = new GridCells_VisionSynapseSet ();

    mpHD_RotationCellClockwiseSynapseSet = new HD_RotationSynapseSet ();
    mpHD_RotationCellCounterClockwiseSynapseSet = new HD_RotationSynapseSet ();
    
    mpGridCells_HD_VelocitySynapseSet.resize (100);
    for (int i = 0; i < 100; i++)
    {
        mpGridCells_HD_VelocitySynapseSet[i] = new GridCells_HD_VelocitySynapseSet ();
    }


    /*  Index + 1 of whatever cell is to be activated. So must be >= 1 */
    mInitialHeading = 1;
    mInitialLocation.x = 0;
    mInitialLocation.y = 0;
    mCount = 0;
    mCountTillFreq = 0;
    mProcessFreq = 10;
    mPositive = 0;
    mNegative = 0;
    mScale = 1.0;
    mVelocity = 0.0;

    /*  sigma controls the width of the gaussian. Smaller sigma, smaller width */
    mSigmaHD = 10;
    mSigmaG = 0.8;

    mDebugFile.open("0000-Master-debug.txt");


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

    mDebugFile.close ();
    /*  delete my brain elements */
    delete mpHDCells;
    delete mpRotationCellCounterClockwise;
    delete mpRotationCellCounterClockwise;
    delete mpVisionCells;

    delete mpHDSynapseSet;
    delete mpHD_VisionSynapseSet;
    delete mpHD_RotationCellClockwiseSynapseSet;
    delete mpHD_RotationCellCounterClockwiseSynapseSet;

    delete mpGridCells;
    delete mpGridCells_VisionSynapseSet;
    delete mpGridCellsSynapseSet;
    for (int i = 0; i < 100; i++)
    {
        delete mpGridCells_HD_VelocitySynapseSet[i];
    }
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
    /*  NOTE
     *  I always set my weight matrix to (post x pre) dimensions
     *  Follow this standard
     */

    int head_cell_dimension_x = 100;
    int head_cell_dimension_y = 1;

    /*  Set up HD cells */
    mpHDCells->SetDimension (head_cell_dimension_x, head_cell_dimension_y);
    mpHDCells->UpdateDirectionalRange ();
    mpHDCells->SetIdentifier (std::string("HD Cells"));
    mpHDCells->Init ();
    mpHDCells->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpHDCells->Identifier()).c_str ());

    /*  Set up grid cells */
    mpGridCells->SetDimension (head_cell_dimension_x, head_cell_dimension_y);
    mpGridCells->SetIdentifier (std::string("Grid Cells"));
    mpGridCells->Init ();
    mpGridCells->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpGridCells->Identifier()).c_str ());

    /*  Rotation Cells */
    mpRotationCellCounterClockwise->SetDimension(1,1);
    mpRotationCellCounterClockwise->SetIdentifier(std::string("Rotation cell counter clockwise"));
    mpRotationCellCounterClockwise->Init ();
    mpRotationCellCounterClockwise->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpRotationCellCounterClockwise->Identifier()).c_str ());

    mpRotationCellClockwise->SetDimension(1,1);
    mpRotationCellClockwise->SetIdentifier(std::string("Rotation cell clockwise"));
    mpRotationCellClockwise->Init ();
    mpRotationCellClockwise->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpRotationCellClockwise->Identifier()).c_str ());

    /*  Velocity cell */
    mpVelocityCell->SetDimension(1,1);
    mpVelocityCell->SetIdentifier(std::string("Velocity Cell"));
    mpVelocityCell->Init ();
    mpVelocityCell->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpVelocityCell->Identifier()).c_str ());

    /*  For the time being */
    mpVisionCells->SetDimension(1, 1);
    mpVisionCells->SetIdentifier(std::string("Vision cell set"));
    mpVisionCells->Init ();
    ROS_DEBUG("%s: Initialized", (mpVisionCells->Identifier()).c_str ());

    /*  Set up HDSynapseSet */
    mpHDSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_x);
    mpHDSynapseSet->SetIdentifier(std::string("HD - CANN synapse set"));
    mpHDSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHDSynapseSet->Identifier()).c_str ());

    /*  GridCellsSynapseSet */
    mpGridCellsSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_x);
    mpGridCellsSynapseSet->SetIdentifier(std::string("Grid Cells - CANN synapse set"));
    mpGridCellsSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpGridCellsSynapseSet->Identifier()).c_str ());

    /*  This is effective synaptic weight: HDx HDy and the Rotation cell. So,
     *  dimension is 100x100, not 100x1 here. */
    mpHD_RotationCellCounterClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_x);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetIdentifier(std::string("HD - Rotation cell counter clockwise synapse set"));
    mpHD_RotationCellCounterClockwiseSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_RotationCellCounterClockwiseSynapseSet->Identifier()).c_str ());

    mpHD_RotationCellClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_x);
    mpHD_RotationCellClockwiseSynapseSet->SetIdentifier(std::string("HD - Rotation cell clockwise synapse set"));
    mpHD_RotationCellClockwiseSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_RotationCellClockwiseSynapseSet->Identifier()).c_str ());

    /*  Effective synapse set */
    for ( int i = 0; i < head_cell_dimension_x; i++)
    {
        std::ostringstream ss;
        ss << "Grid Cells - HD - velocity cell synapse set - " << i;
        mpGridCells_HD_VelocitySynapseSet[i]->SetDimension(head_cell_dimension_x,head_cell_dimension_x);
                mpGridCells_HD_VelocitySynapseSet[i]->SetIdentifier(ss.str ());
        mpGridCells_HD_VelocitySynapseSet[i]->Init ();
        ROS_DEBUG("%s: Initialized", (mpGridCells_HD_VelocitySynapseSet[i]->Identifier()).c_str ());
    }

    mpHD_VisionSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_y);
    mpHD_VisionSynapseSet->SetIdentifier(std::string("HD - Vision cell synapse set"));
    mpHD_VisionSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_VisionSynapseSet->Identifier()).c_str ());

    mpGridCells_VisionSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_y);
    mpGridCells_VisionSynapseSet->SetIdentifier(std::string("Grid Cells - Vision cell synapse set"));
    mpGridCells_VisionSynapseSet->Init ();

    mGridHeatMap = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (head_cell_dimension_x, head_cell_dimension_y);
    ROS_DEBUG("%s: Initialized", (mpGridCells_VisionSynapseSet->Identifier()).c_str ());


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
    CalibrateHDSet ();
    CalibrateGridCellSet ();

}		/* -----  end of method Bionavigator::Calibrate  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: CalibrateGridCellSet
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::CalibrateGridCellSet (  )
{
    ROS_DEBUG("Calibrating Grid Cell system");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s_hd;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s_g_sq;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_g_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_g_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> max_delta_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> max_delta_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x_hd;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> threesixty_delta_x_hd;
    double preferred_direction_for_iteration;
    const double PI  =3.141592653589793238462;
    double firing_rate_for_training;
    double grid_cells_side_length = sqrt (mpGridCells->DimensionX ());
    ROS_DEBUG("grid cell side length is: %f", grid_cells_side_length);
    double half = 0;


    firing_rate_for_training = 1;

    /*  Ensure all synapses are plastic here */
    mpGridCellsSynapseSet->SetPlastic ();
    for ( int i = 0; i < mpHDCells->DimensionX (); i ++)
        mpGridCells_HD_VelocitySynapseSet[i]->SetPlastic ();

    delta_s_hd.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_s_g_sq.resize(mpGridCells->DimensionX (), mpGridCells->DimensionY ());

    delta_x_hd.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
    threesixty_delta_x_hd.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

    delta_x.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    delta_y.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    max_delta_x.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    max_delta_y.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    delta_g_x.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    delta_g_y.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );

    /*  We set the preferred head directions uniformly */
    preferred_x.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    preferred_y.resize(mpGridCells->DimensionX (),mpGridCells->DimensionY () );
    preferred_directions.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

    /*  Set up preferred directions of head direction cells */
    for (double i = (360.0/mpHDCells->DimensionX ()), j = 0; i <= 360; i+=(360.0/mpHDCells->DimensionX ()), j++)
    {
        preferred_directions (j, 0) = i;
    }

    half = 0.0;
    /*  Set up preferred locations of grid cells */
    for (double j = 0.0; j < grid_cells_side_length; j++)
    {

        for (double k = 0.0; k < grid_cells_side_length; k++)
        {

            preferred_x ((grid_cells_side_length * j) + k, 0) = k + half;
            /*  Let it be normal co-ordinates. Multiply by 3/4 when calculating
             *  distance */
            preferred_y ((grid_cells_side_length * j) + k, 0) = j;
        }

        /*  we need to alternate the x as halves and fulls for grid cells since
         *  they're equilateral trianges */

        if (half == 0)
            half = 0.5;
        else if (half == 0.5)
            half = 0;
    }

    ROS_DEBUG_STREAM("preferred_x is:" << preferred_x.transpose ());
    ROS_DEBUG_STREAM("preferred_y is:" << (preferred_y.transpose ().array () * (sqrt(3.0)/2.0)) );

    mpGridCellsSynapseSet->Init ();
    mpGridCells->Init ();

    mpVelocityCell->EnableForceFire (firing_rate_for_training);

    /*  Right one by one */
    preferred_direction_for_iteration = mInitialHeading - 90;

    if (preferred_direction_for_iteration < 0)
        preferred_direction_for_iteration += 360;

    delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x_hd = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x_hd = (360 - delta_x_hd.array ());
    delta_s_hd = delta_x_hd.cwiseMin (threesixty_delta_x_hd); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s_hd, mSigmaHD);
    for (int i = 0; i <  mpGridCells->DimensionX (); i++)
/*     for (int i = 0; i < 20; i++)
 */
    {
        std::ostringstream ss;
        ss << i;

        ROS_DEBUG("Right Grid cell calibration iteration: %d",i);
        double preferred_x_for_iteration = preferred_x(i, 0);
        double preferred_y_for_iteration = preferred_y(i, 0);
        ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

        delta_s_g_sq = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpGridCells->DimensionX (), 1);

        delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
        max_delta_x = (10 - delta_x.array ());
        max_delta_x =  max_delta_x.array().abs ();

        delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();
        max_delta_y  = ((10) - delta_y.array ()).abs ();

        delta_g_x = delta_x.cwiseMin(max_delta_x);
        ROS_DEBUG_STREAM("delta x final is: " << delta_g_x.transpose ());
        delta_g_y = delta_y.cwiseMin(max_delta_y);
        ROS_DEBUG_STREAM("delta y final is: " << delta_g_y.transpose ());

        /*  Do not square root since its squared again in the firing rate
         *  method */
        delta_s_g_sq = (delta_g_x.array ().square () + (3.0/4.0 * delta_g_y.array (). square ()));
/*         ROS_DEBUG_STREAM("Distance is " << delta_s_g_sq.transpose ());
 */
        /*  Calculate firing rate for this iteration */
        mpGridCells->UpdateFiringRate(delta_s_g_sq, mSigmaG);

/*         ROS_DEBUG_STREAM("Grid cell firing rate is:" << mpGridCells->FiringRate().transpose ());
 */


        /*  Recurrent synapse update: only required once */
        mpGridCellsSynapseSet->UpdateWeight (mpGridCells->FiringRate(), mpGridCells->FiringRate ().transpose ());

        for ( int i = 0; i < mpHDCells->DimensionX (); i ++)
        {
            Eigen::Matrix<double, 1, 1> temp;
            temp = (mpHDCells->FiringRate ())(i,0) * mpVelocityCell->FiringRate ().array ();
            mpGridCells_HD_VelocitySynapseSet[i]->UpdateWeight (mpGridCells->FiringRate (), (mpGridCells->FiringRateTrace () * temp).transpose ());
        }

        /*  Activate trace. This will hold f(t-1) always */
        mpGridCells->UpdateFiringRateTrace ();

/*         mpGridCells->PrintFiringRateToFile((std::string("Calibrating-GridCells-FiringRate-E-") + ss.str () + std::string(".txt")));
 *         mpGridCellsSynapseSet->PrintToFile((std::string("Calibrating-Grid-synapse-E-") + ss.str () + std::string(".txt")));
 *         mpGridCells_HD_VelocitySynapseSet[i]->PrintToFile((std::string("Calibrating-Grid-HD-V-synapse-E-") + ss.str () + std::string(".txt")));
 */
    }

    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-E.txt"));
    mpGridCells_HD_VelocitySynapseSet[50]->PrintToFile(std::string("Calibrated-Grid-HD50-V-synapse-E.txt"));
    mpGridCells_HD_VelocitySynapseSet[25]->PrintToFile(std::string("Calibrated-Grid-HD25-V-synapse-E.txt"));
    mpGridCells_HD_VelocitySynapseSet[75]->PrintToFile(std::string("Calibrated-Grid-HD75-V-synapse-E.txt"));


    /*  Left one by one */
    mpGridCells->Init ();
    mpHDCells->Init ();
    preferred_direction_for_iteration = mInitialHeading + 90;

    if (preferred_direction_for_iteration > 360)
        preferred_direction_for_iteration -= 360;

    delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x_hd = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x_hd = (360 - delta_x_hd.array ());
    delta_s_hd = delta_x_hd.cwiseMin (threesixty_delta_x_hd); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s_hd, mSigmaHD);
/*     for (int i =  0 ;  i > 0; i--)
 */
    for (int i =  mpGridCells->DimensionX () -1 ;  i >= 0; i--)
    {
        std::ostringstream ss;
        ss << i;

        ROS_DEBUG("Left Grid cell calibration iteration: %d",i);
        double preferred_x_for_iteration = preferred_x(i, 0);
        double preferred_y_for_iteration = preferred_y(i, 0);
        ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

        delta_s_g_sq = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpGridCells->DimensionX (), 1);

        delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
        max_delta_x = (10 - delta_x.array ());
        max_delta_x =  max_delta_x.array().abs ();


        delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();
        max_delta_y  = ((10) - delta_y.array ()).abs ();

        delta_g_x = delta_x.cwiseMin(max_delta_x);
        ROS_DEBUG_STREAM("delta x final is: " << delta_g_x.transpose ());
        delta_g_y = delta_y.cwiseMin(max_delta_y);
        ROS_DEBUG_STREAM("delta y final is: " << delta_g_y.transpose ());

        /*  Do not square root since its squared again in the firing rate
         *  method */
        delta_s_g_sq = (delta_g_x.array ().square () + (3.0/4.0 * delta_g_y.array (). square ()));
/*         ROS_DEBUG_STREAM("Distance is " << delta_s_g_sq.transpose ());
 */
        /*  Calculate firing rate for this iteration */
        mpGridCells->UpdateFiringRate(delta_s_g_sq, mSigmaG);

/*         ROS_DEBUG_STREAM("Grid cell firing rate is:" << mpGridCells->FiringRate().transpose ());
 */
        /*  Recurrent synapse update: only required once */
        mpGridCellsSynapseSet->UpdateWeight (mpGridCells->FiringRate(), mpGridCells->FiringRate ().transpose ());

        for ( int i = 0; i < mpHDCells->DimensionX (); i ++)
        {
            Eigen::Matrix<double, 1, 1> temp;
            temp = (mpHDCells->FiringRate ())(i,0) * mpVelocityCell->FiringRate ().array ();
            mpGridCells_HD_VelocitySynapseSet[i]->UpdateWeight (mpGridCells->FiringRate (), (mpGridCells->FiringRateTrace () * temp).transpose ());
        }

        /*  Activate trace. This will hold f(t-1) always */
        mpGridCells->UpdateFiringRateTrace ();

/*         mpGridCells->PrintFiringRateToFile((std::string("Calibrating-GridCells-FiringRate-W-") + ss.str () + std::string(".txt")));
 *         mpGridCellsSynapseSet->PrintToFile((std::string("Calibrating-Grid-synapse-W-") + ss.str () + std::string(".txt")));
 *         mpGridCells_HD_VelocitySynapseSet->PrintToFile((std::string("Calibrating-Grid-HD-V-synapse-W-") + ss.str () + std::string(".txt")));
 */

    }
    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-W.txt"));
    mpGridCells_HD_VelocitySynapseSet[50]->PrintToFile(std::string("Calibrated-Grid-HD50-V-synapse-W.txt"));
    mpGridCells_HD_VelocitySynapseSet[25]->PrintToFile(std::string("Calibrated-Grid-HD25-V-synapse-W.txt"));
    mpGridCells_HD_VelocitySynapseSet[75]->PrintToFile(std::string("Calibrated-Grid-HD75-V-synapse-W.txt"));


    /*  Forward by one */
    mpGridCells->Init ();
    mpHDCells->Init ();

    /*  Since the distance isn't 1 unit, it's root(3) */
/*     firing_rate_for_training = 2* 0.8660254037844386;
 *     mpVisionCells->DisableForceFire ();
 *     mpVisionCells->EnableForceFire(firing_rate_for_training);
 */

    preferred_direction_for_iteration = mInitialHeading;

    delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x_hd = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x_hd = (360 - delta_x_hd.array ());
    delta_s_hd = delta_x_hd.cwiseMin (threesixty_delta_x_hd); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s_hd, mSigmaHD);
    for (int i = 0; i <  grid_cells_side_length; i++)
    {
/*         for (int j = 0; j < 2; j+=2)
 */
        for (int j = 0; j < grid_cells_side_length; j++)
        {

            std::ostringstream ss;
            ss << i << j;

            ROS_DEBUG("Forward Grid cell calibration iteration 1: %d-%d",i,j);
            double preferred_x_for_iteration = preferred_x((10 * j) + i, 0);
            double preferred_y_for_iteration = preferred_y((10 * j) + i, 0);
            ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

            delta_s_g_sq = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpGridCells->DimensionX (), 1);

            delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
            max_delta_x = (10 - delta_x.array ());
            max_delta_x =  max_delta_x.array().abs ();

            delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();
            max_delta_y  = ((10) - delta_y.array ()).abs ();

            delta_g_x = delta_x.cwiseMin(max_delta_x);
            ROS_DEBUG_STREAM("delta x final is: " << delta_g_x.transpose ());

            delta_g_y = delta_y.cwiseMin(max_delta_y);
            ROS_DEBUG_STREAM("delta y final is: " << delta_g_y.transpose ());

            delta_s_g_sq = (delta_g_x.array ().square () + (3.0/4.0 * delta_g_y.array (). square ()));

            /*  Calculate firing rate for this iteration */
            mpGridCells->UpdateFiringRate(delta_s_g_sq, mSigmaG);
            mpGridCellsSynapseSet->UpdateWeight (mpGridCells->FiringRate(), mpGridCells->FiringRate ().transpose ());

            for ( int i = 0; i < mpHDCells->DimensionX (); i ++)
            {
                Eigen::Matrix<double, 1, 1> temp;
                temp = (mpHDCells->FiringRate ())(i,0) * mpVelocityCell->FiringRate ().array ();
                mpGridCells_HD_VelocitySynapseSet[i]->UpdateWeight (mpGridCells->FiringRate (), (mpGridCells->FiringRateTrace () * temp).transpose ());
            }




            /*  Activate trace. This will hold f(t-1) always */
            mpGridCells->UpdateFiringRateTrace ();

/*             mpGridCells->PrintFiringRateToFile((std::string("Calibrating-GridCells-FiringRate-N-") + ss.str () + std::string(".txt")));
 *             mpGridCellsSynapseSet->PrintToFile((std::string("Calibrating-Grid-synapse-N-") + ss.str () + std::string(".txt")));
 *             mpGridCells_HD_VelocitySynapseSet->PrintToFile((std::string("Calibrating-Grid-HD-V-synapse-") + ss.str () + std::string(".txt")));
 */

        }
    }
    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-N1.txt"));
    mpGridCells_HD_VelocitySynapseSet[50]->PrintToFile(std::string("Calibrated-Grid-HD50-V-synapse-N1.txt"));
    mpGridCells_HD_VelocitySynapseSet[25]->PrintToFile(std::string("Calibrated-Grid-HD25-V-synapse-N1.txt"));
    mpGridCells_HD_VelocitySynapseSet[75]->PrintToFile(std::string("Calibrated-Grid-HD75-V-synapse-N1.txt"));


    /*  Backward by one */
    mpGridCells->Init ();
    preferred_direction_for_iteration = mInitialHeading + 180;
    if (preferred_direction_for_iteration >= 360)
        preferred_direction_for_iteration -= 360;

    delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x_hd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x_hd = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x_hd = (360 - delta_x_hd.array ());
    delta_s_hd = delta_x_hd.cwiseMin (threesixty_delta_x_hd); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s_hd, mSigmaHD);
    mpHDCells->PrintFiringRateToFile("HDCells-FiringRate-backward-grid-cell.txt");
    for (int i = grid_cells_side_length -1 ; i >= 0; i--)
    {
        for (int j = grid_cells_side_length -1 ; j >= 0; j--)
        {

            std::ostringstream ss;
            ss << i << j;

            ROS_DEBUG("Backward Grid cell calibration iteration 1: %d-%d",i,j);
            double preferred_x_for_iteration = preferred_x(i, 0);
            double preferred_y_for_iteration = preferred_y((10 * j) + i, 0);
            ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

            delta_s_g_sq = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpGridCells->DimensionX (), 1);

            delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
            max_delta_x = (10 - delta_x.array ());
            max_delta_x =  max_delta_x.array().abs ();

            delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();
            max_delta_y  = ((10) - delta_y.array ()).abs ();

            delta_g_x = delta_x.cwiseMin(max_delta_x);
            ROS_DEBUG_STREAM("delta x final is: " << delta_g_x.transpose ());

            delta_g_y = delta_y.cwiseMin(max_delta_y);
            ROS_DEBUG_STREAM("delta y final is: " << delta_g_y.transpose ());

            delta_s_g_sq = (delta_g_x.array ().square () + (3.0/4.0 * delta_g_y.array (). square ()));

            /*  Calculate firing rate for this iteration */
            mpGridCells->UpdateFiringRate(delta_s_g_sq, mSigmaG);
            mpGridCellsSynapseSet->UpdateWeight (mpGridCells->FiringRate(), mpGridCells->FiringRate ().transpose ());

            for ( int i = 0; i < mpHDCells->DimensionX (); i ++)
            {
                Eigen::Matrix<double, 1, 1> temp;
                temp = (mpHDCells->FiringRate ())(i,0) * mpVelocityCell->FiringRate ().array ();
                mpGridCells_HD_VelocitySynapseSet[i]->UpdateWeight (mpGridCells->FiringRate (), (mpGridCells->FiringRateTrace () * temp).transpose ());
            }

            /*  Activate trace. This will hold f(t-1) always */
            mpGridCells->UpdateFiringRateTrace ();

/*             mpGridCells->PrintFiringRateToFile((std::string("Calibrating-GridCells-FiringRate-S-") + ss.str () + std::string(".txt")));
 *             mpGridCellsSynapseSet->PrintToFile((std::string("Calibrating-Grid-synapse-S-") + ss.str () + std::string(".txt")));
 *             mpGridCells_HD_VelocitySynapseSet->PrintToFile((std::string("Calibrating-Grid-HD-V-synapse-") + ss.str () + std::string(".txt")));
 */

        }
    }
    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-S1.txt"));
    mpGridCells_HD_VelocitySynapseSet[50]->PrintToFile(std::string("Calibrated-Grid-HD50-V-synapse-S1.txt"));
    mpGridCells_HD_VelocitySynapseSet[25]->PrintToFile(std::string("Calibrated-Grid-HD25-V-synapse-S1.txt"));
    mpGridCells_HD_VelocitySynapseSet[75]->PrintToFile(std::string("Calibrated-Grid-HD75-V-synapse-S1.txt"));


/*     mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-final.txt"));
 *     mpGridCellsSynapseSet->Normalize ();
 *     mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-final-normalized.txt"));
 *     return;
 */

    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-final.txt"));
    mpGridCellsSynapseSet->Normalize ();
    mpGridCellsSynapseSet->PrintToFile(std::string("Calibrated-Grid-synapse-final-normalized.txt"));

    for (int i = 0; i < mpHDCells->DimensionX (); i++)
    {
        mpGridCells_HD_VelocitySynapseSet[i]->Normalize ();
    }
    mpGridCells_HD_VelocitySynapseSet[50]->PrintToFile(std::string("Calibrated-Grid-HD50-V-synapse-normalized.txt"));
    mpGridCells_HD_VelocitySynapseSet[25]->PrintToFile(std::string("Calibrated-Grid-HD25-V-synapse-normalized.txt"));
    mpGridCells_HD_VelocitySynapseSet[75]->PrintToFile(std::string("Calibrated-Grid-HD75-V-synapse-normalized.txt"));

/*     ROS_DEBUG("%s calibrated to: [%f,%f]",mpGridCells_HD_VelocitySynapseSet->Identifier().c_str (), mpGridCells_HD_VelocitySynapseSet->Max (), mpGridCells_HD_VelocitySynapseSet->Min ());
 */

    /*  Disable all the force firing */
    mpVelocityCell->DisableForceFire ();
    mpGridCells->DisableForceFire ();

    mIsGridCellSetCalibrated = true;
    ROS_DEBUG("Grid cell Calibration complete");
    return;
}		/* -----  end of method Bionavigator::CalibrateGridCellSet  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: CalibrateHDSet
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::CalibrateHDSet (  )
{
    ROS_DEBUG("Calibrating HD system");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_hd_weights;
    const double PI  =3.141592653589793238462;
    double firing_rate_for_training;
/*     firing_rate_for_training = mScale * (2.0*PI)/mpHDCells->DimensionX ();
 */

    firing_rate_for_training = 1;
    /*  Ensure all synapses are plastic here */
    mpHDSynapseSet->SetPlastic ();
    mpHD_RotationCellClockwiseSynapseSet->SetPlastic ();
    mpHD_RotationCellCounterClockwiseSynapseSet->SetPlastic ();


    delta_s.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_s = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());           /* s^(HD)_i: difference between actual head direction and preferred head direction */

    temp_hd_weights.resize(mpHDSynapseSet->DimensionX (), mpHDSynapseSet->DimensionY ());
    temp_hd_weights = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDSynapseSet->DimensionX (), mpHDSynapseSet->DimensionY ());           /* s^(HD)_i: difference between actual head direction and preferred head direction */

    /*  We set the preferred head directions uniformly */
    preferred_directions.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
    preferred_directions = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (),mpHDCells->DimensionY ());
    for (double i = (360.0/mpHDCells->DimensionX ()), j = 0; i <= 360; i+=(360.0/mpHDCells->DimensionX ()), j++)
    {
        preferred_directions (j, 0) = i;
    }
    
    ROS_DEBUG_STREAM("Preferred directions matrix is:" << preferred_directions.transpose ());

    /*  Anti clockwise */
    mpHDCells->Init ();

    mpRotationCellCounterClockwise->EnableForceFire (firing_rate_for_training);
    mpRotationCellClockwise->DisableForceFire ();
    for (int i=1; i <= mpHDCells->DimensionX (); i++)
    {
        std::ostringstream ss;
        ss << i;

        ROS_DEBUG("Counter Clockwise calibrtion iteration: %d",i);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> threesixty_delta_x;
        double preferred_direction_for_iteration = preferred_directions(i -1,0);
        ROS_DEBUG("Preferred direction for this iteration is: %f",preferred_direction_for_iteration);

        delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
        threesixty_delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

        delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* |x_i - x| */
        threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* 360 - |x_i - x| */

        delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
        threesixty_delta_x = (360 - delta_x.array ());

        delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */
/*         ROS_DEBUG_STREAM("delta_s is:" << delta_s.transpose ());
 */


        /*  Calculate firing rate for this iteration */
        mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
        /*  Head directions train: what they should be for this firing rate */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate(), mpHDCells->FiringRate ().transpose ());

        /*  Rotation cells train: the firing of rotation cells changes head
         *  direction firing from trace (previous) to new */
        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());

        /*  Activate trace. This will hold f(t-1) always */
        mpHDCells->UpdateFiringRateTrace ();

/*         ROS_DEBUG("Firing rate values are [%f,%f]",mpHDCells->FiringRate().maxCoeff (), mpHDCells->FiringRate().minCoeff ());
 *         ROS_DEBUG("Firing rate trace values are [%f,%f]",mpHDCells->FiringRateTrace().maxCoeff (), mpHDCells->FiringRateTrace().minCoeff ());
 */
        mpHDCells->PrintFiringRateToFile((std::string("Calibrating-HDCells-FiringRate-1-") + ss.str () + std::string(".txt")));
        mpHDSynapseSet->PrintToFile((std::string("Calibrating-HD-synapse-1-") + ss.str () + std::string(".txt")));
        mpHD_RotationCellCounterClockwiseSynapseSet->PrintToFile((std::string("Calibrating-RotationCellCounterClockwise-synapse-") + ss.str () + std::string(".txt")));

        /*  Clockwise calibration not needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
    }
    mpRotationCellCounterClockwise->DisableForceFire ();
    ROS_DEBUG("%s calibrated to: [%f,%f]",mpHD_RotationCellCounterClockwiseSynapseSet->Identifier().c_str (), mpHD_RotationCellCounterClockwiseSynapseSet->Max (), mpHD_RotationCellCounterClockwiseSynapseSet->Min ());

/*     mpHDSynapseSet->Normalize ();
 *
 *     Normalize after both iterations
 */
    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-1.txt"));
    mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();

    mpHD_RotationCellCounterClockwiseSynapseSet->PrintToFile(std::string("Calibrated-RotationCellCounterClockwise-synapse.txt"));
    temp_hd_weights = mpHDSynapseSet->WeightMatrix ();

    /*
     * Clockwise
     */
    mpHDSynapseSet->Init ();
    mpHDCells->Init ();

    mpRotationCellClockwise->EnableForceFire (firing_rate_for_training);
/*     ROS_DEBUG_STREAM("" << mpRotationCellClockwise->Identifier ().c_str () << ": Firing rate is: " << mpRotationCellClockwise->FiringRate ());
 */
    for (int i = mpHDCells->DimensionX (); i >= 1 ; i--)
    {
        std::ostringstream ss;
        ss << mpHDCells->DimensionX () - i;

        ROS_DEBUG("Clockwise calibrtion iteration: %d",i);
        Eigen::Matrix<double, Eigen::Dynamic, 1> delta_x;
        Eigen::Matrix<double, Eigen::Dynamic, 1> threesixty_delta_x;
        double preferred_direction_for_iteration = preferred_directions(i -1,0);
        ROS_DEBUG("Preferred direction for this iteration is: %f",preferred_direction_for_iteration);

        delta_x.resize(mpHDCells->DimensionX (), 1);
        threesixty_delta_x.resize(mpHDCells->DimensionX (), 1);

        delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* |x_i - x| */
        threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* 360 - |x_i - x| */

        delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
        threesixty_delta_x = (360 - delta_x.array ());

        delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */
/*         ROS_DEBUG_STREAM("delta_s is:" << delta_s.transpose ());
 */

        /*  Calculate firing rate for this iteration */
        mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
        /*  Head directions train: what they should be for this firing rate */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate(), mpHDCells->FiringRate ().transpose ());

        /*  Rotation cells train: the firing of rotation cells changes head
         *  direction firing from trace (previous) to new */
        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());

        /*  Activate trace. This will hold f(t-1) always */
        mpHDCells->UpdateFiringRateTrace ();

        mpHDCells->PrintFiringRateToFile((std::string("Calibrating-HDCells-FiringRate-2-") + ss.str () + std::string(".txt")));
        mpHDSynapseSet->PrintToFile((std::string("Calibrating-HD-synapse-2-") + ss.str () + std::string(".txt")));
        mpHD_RotationCellClockwiseSynapseSet->PrintToFile((std::string("Calibrating-RotationCellClockwise-synapse-") + ss.str () + std::string(".txt")));

/*         ROS_DEBUG("Firing rate values are [%f,%f]",mpHDCells->FiringRate().maxCoeff (), mpHDCells->FiringRate().minCoeff ());
 *         ROS_DEBUG("Firing rate trace values are [%f,%f]",mpHDCells->FiringRateTrace().maxCoeff (), mpHDCells->FiringRateTrace().minCoeff ());
 */

        /*  Counter clockwise stuff needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
    }

    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-2.txt"));
    mpHDSynapseSet->AddToWeight(temp_hd_weights);
    mpHDSynapseSet->Normalize ();

    mpHD_RotationCellClockwiseSynapseSet->Normalize ();

    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-final.txt"));

    ROS_DEBUG("%s calibrated to: [%f,%f]",mpHD_RotationCellClockwiseSynapseSet->Identifier().c_str (), mpHD_RotationCellClockwiseSynapseSet->Max (), mpHD_RotationCellClockwiseSynapseSet->Min ());

    /*  Set the inhibition rate */
    //mpHDCells->InhibitionRate (0.2 * mpHDSynapseSet->WeightMatrix ().maxCoeff ());

    /*  Disable all the force firing */
    mpRotationCellClockwise->DisableForceFire ();
    mpRotationCellCounterClockwise->DisableForceFire ();
    mpHDCells->DisableForceFire ();

    /*  Set synapses to stiff. These synapses don't need any modification
     *  in the future. */
/*     mpHDSynapseSet->SetStiff ();
 *     mpHD_RotationCellClockwiseSynapseSet->SetStiff ();
 *     mpHD_RotationCellCounterClockwiseSynapseSet->SetStiff ();
 */
    mpHD_RotationCellClockwiseSynapseSet->PrintToFile(std::string("Calibrated-RotationCellClockwise-synapse.txt"));

    /*  Switch to LTP/D */
    mpHDSynapseSet->LearningRate (0.00);
    mpHD_RotationCellClockwiseSynapseSet->LearningRate (0.0);
    mpHD_RotationCellCounterClockwiseSynapseSet->LearningRate(0.0);

    mIsHDCalibrated = true;
    ROS_DEBUG("HD Calibration complete");
}		/* -----  end of method Bionavigator::CalibrateHDSet  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: SetInitialDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::SetInitialDirection ( )
{
    ROS_INFO("Setting initial reference direction to %f", mInitialHeading);
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s;
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> threesixty_delta_x;
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> initial_direction_matrix; /**< @f$I^{V}_i@f$  */

    /*  For the message to be published */
    std_msgs::Float64 msg;

    initial_direction_matrix.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());


//    threesixty_delta_x.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
//    threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* 360 - |x_i - x| */
//    delta_s.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY ());
//    delta_s = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());           /* s^(HD)_i: difference between actual head direction and preferred head direction */
//    delta_x.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
//    delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* |x_i - x| */
//    preferred_directions.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
//    preferred_directions = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (),mpHDCells->DimensionY ());
//
//    /*  Set up preferred directions of all head cells */
//    for (double i = (1.0* 360.0/mpHDCells->DimensionX ()), j = 0; i <= 360; i+=(360.0/mpHDCells->DimensionX ()), j++)
//    {
//        preferred_directions (j, 0) = i;
//    }

    /*  Disable all input completely */
    mpRotationCellCounterClockwise->DisableForceFire ();
    mpRotationCellClockwise->DisableForceFire ();
    mpHDCells->DisableForceFire ();
    mpHDCells->Init ();
    /*
     * Set the synaptic weights of the vision head synapses to the weights
     * learned during training for the cann recurrent network.
     *
     * This will ensure a peak at the preferred direction.
     */
    mpHD_VisionSynapseSet->SetPlastic ();
    /*  Since neuron 0 has preferred direction of 3.6 and not 0 */
    initial_direction_matrix = (mpHDSynapseSet->WeightMatrix ()).col (((mInitialHeading * mpHDCells->DimensionX ())/360.0) -1);
    mpHD_VisionSynapseSet->AddToWeight((initial_direction_matrix.array ()/initial_direction_matrix.maxCoeff ()).matrix());

    mpHD_VisionSynapseSet->AddToWeight(initial_direction_matrix.array ());
    mpHD_VisionSynapseSet->PrintToFile(std::string("Before-Forced-vision-synapse.txt"));
    mpHD_VisionSynapseSet->Normalize ();
    mpHDCells->PrintFiringRateToFile(std::string("Before-Forced-HDCells-FiringRate.txt"));
    mpHDCells->PrintActivationToFile(std::string("Before-Forced-HDCells-Activation.txt"));
    mpHD_VisionSynapseSet->SetStiff ();
    mpVisionCells->EnableForceFire (1.0);

    /*
     * Find a good number of iterations. Optimize it.
     */
    ROS_INFO("Forcing an initial direction to %f", mInitialHeading);
    for (double i = 0; i < 10 ; i++ ) 
    {
//        mpHDCells->UpdateActivation (initial_direction_matrix, mpHDSynapseSet->WeightMatrix ());
        std::ostringstream ss;
        ss << i;

/*         mpHDCells->InhibitionRate ((0.05 *mpHDSynapseSet->WeightMatrix().maxCoeff ()));
 */
        mpHDCells->InhibitionRate (0.02);
        mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
        mpHDCells->FiringRates ();

        mpHDCells->PrintFiringRateToFile((std::string("Forced-HDCells-FiringRate-") + ss.str () + std::string(".txt")));
        mpHDCells->PrintActivationToFile((std::string("Forced-HDCells-Activation-") + ss.str () + std::string(".txt")));

        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());
        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());
        mpHDSynapseSet->Normalize ();
        mpHD_RotationCellClockwiseSynapseSet->Normalize ();
        mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();

    }
    mHeadDirection = mpHDCells->CurrentHeadDirection ();
    ROS_DEBUG("Head direction is now: %f",mHeadDirection);
/*     mpHDCells->PrintFiringRateToFile(std::string("Forced-HDCells-FiringRate.txt"));
 *     mpHDCells->PrintActivationToFile(std::string("Forced-HDCells-Activation.txt"));
 *     mpHDSynapseSet->PrintToFile(std::string("Forced-HD-synapse.txt"));
 *     mpHD_RotationCellCounterClockwiseSynapseSet->PrintToFile(std::string("Forced-HD-RotationCellCounterClockwise-synapse.txt"));
 */
    msg.data = mHeadDirection;
    mHeadDirectionPublisher.publish(msg);
    mpVisionCells->DisableForceFire ();

    /*
     * Find a good number of loops for this
     */
    ROS_INFO("Stabilizing activity packet");
    for (int j = 0; j < 200 ; j++) 
    {
        mpHDCells->InhibitionRate (0.02);
        mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
        mpHDCells->UpdateFiringRate ();
        mpHDCells->UpdateFiringRateTrace ();
        mpHDCells->FiringRates ();

        mHeadDirection = mpHDCells->CurrentHeadDirection ();
        ROS_DEBUG("Head direction is now: %f",mHeadDirection);

        /*  Learning still occurs! */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());
        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());
        mpHDSynapseSet->Normalize ();
        mpHD_RotationCellClockwiseSynapseSet->Normalize ();
        mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();


        ROS_DEBUG("Inhibition rate set: %f",mpHDCells->InhibitionRate());
/*         msg.data = mHeadDirection;
 *         mHeadDirectionPublisher.publish(msg);
 */
        if((j == 0 ) || (j%10 == 9))
        {
            std::ostringstream ss;
            ss << j;
            mpHDCells->PrintFiringRateToFile((std::string("Stabilized-HDCells-FiringRate-") + ss.str () + std::string(".txt")));
            mpHDCells->PrintActivationToFile((std::string("Stabilized-HDCells-Activation-") + ss.str () + std::string(".txt")));
/*             mpHDSynapseSet->PrintToFile(std::string("Stabilized-HD-synapse.txt"));
 */
        }
    }

    mIsInitialDirectionSet = true;
    ROS_INFO("Initial direction set to %f", mHeadDirection);
    mpHDCells->PrintFiringRateToFile(std::string("Stabilized-HDCells-FiringRate.txt"));
    mpHDCells->PrintActivationToFile(std::string("Stabilized-HDCells-Activation.txt"));
    mpHDSynapseSet->PrintToFile(std::string("Stabilized-HD-synapse.txt"));
    mpHD_RotationCellCounterClockwiseSynapseSet->PrintToFile(std::string("Stabilized-HD-RotationCellCounterClockwise-synapse.txt"));
    mpHD_RotationCellClockwiseSynapseSet->PrintToFile(std::string("Stabilized-HD-RotationCellClockwise-synapse.txt"));
}		/* -----  end of method Bionavigator::SetInitialDirection  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: SetInitialLocation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::SetInitialLocation ( )
{
    ROS_INFO("Setting initial reference location to %f, %f in neuron terms, not absolute location terms", mInitialLocation.x, mInitialLocation.y);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> initial_location_matrix; /**< @f$I^{V}_i@f$  */

    /*  For the message to be published */
    std_msgs::Float64 msg;

    initial_location_matrix.resize(mpGridCells->DimensionX (), mpGridCells->DimensionY ());

    /*  Disable all input completely */
    mpVelocityCell->DisableForceFire ();
    mpGridCells->DisableForceFire ();
    mpHDCells->DisableForceFire ();
    mpGridCells->Init ();
    /*  Make sure they aren't firing */

    /*
     * Set the synaptic weights of the vision head synapses to the weights
     * learned during training for the cann recurrent network.
     *
     * This will ensure a peak at the preferred direction.
     */
    mpGridCells_VisionSynapseSet->SetPlastic ();
    initial_location_matrix = (mpGridCellsSynapseSet->WeightMatrix ()).col (10 * mInitialLocation.x + mInitialLocation.y);
    mpGridCells_VisionSynapseSet->AddToWeight(initial_location_matrix.array ());
    mpGridCells_VisionSynapseSet->PrintToFile(std::string("GridCells-Before-Forced-vision-synapse.txt"));
/*     mpGridCells_VisionSynapseSet->Normalize ();
 */
    mpGridCells->PrintFiringRateToFile(std::string("Before-Forced-GridCells-FiringRate.txt"));
    mpGridCells->PrintActivationToFile(std::string("Before-Forced-GridCells-Activation.txt"));
    mpGridCells_VisionSynapseSet->SetStiff ();
    mpVisionCells->EnableForceFire (1.0);
    mpGridCells->InhibitionRate (0.01);

    /*
     * Find a good number of iterations. Optimize it.
     */
    ROS_INFO("Forcing an initial location");
    for (double i = 0; i < 30 ; i++ ) 
    {
        std::ostringstream ss;
        ss << i;


        /*  Passing all matrices, but the velocity synapse vector.. NOTE */
        mpGridCells->UpdateActivation(mpVelocityCell->FiringRate(), mpVisionCells->FiringRate(), mpHDCells->FiringRate(), mpGridCells_HD_VelocitySynapseSet, mpGridCellsSynapseSet->WeightMatrix(),mpGridCells_VisionSynapseSet->WeightMatrix()  );
        mpGridCells->FiringRates ();

        mpGridCells->PrintFiringRateToFile((std::string("Forced-GridCells-FiringRate-") + ss.str () + std::string(".txt")));
        mpGridCells->PrintActivationToFile((std::string("Forced-GridCells-Activation-") + ss.str () + std::string(".txt")));

/*         mpGridSynapseSet->UpdateWeight (mpGridCells->FiringRateTrace (), mpGridCells->FiringRate ().transpose ());
 *         mpGridSynapseSet->Normalize ();
 */

    }
    mLocation.x = mpGridCells->CurrentLocation ()/10.0;
    mLocation.y = (int)(mpGridCells->CurrentLocation ()) % 10;
    ROS_DEBUG("Location is now: [%f, %f]",mLocation.x, mLocation.y);
/*     mpGridCells->PrintFiringRateToFile(std::string("Forced-GridCells-FiringRate.txt"));
 *     mpGridCells->PrintActivationToFile(std::string("Forced-GridCells-Activation.txt"));
 *     mpGridSynapseSet->PrintToFile(std::string("Forced-Grid-synapse.txt"));
 *     mpGrid_RotationCellCounterClockwiseSynapseSet->PrintToFile(std::string("Forced-Grid-RotationCellCounterClockwise-synapse.txt"));
 */
/*     msg.data = mHeadLocation;
 *     mHeadLocationPublisher.publish(msg);
 */
    mpVisionCells->DisableForceFire ();


    /*
     * Find a good number of loops for this
     */
    ROS_INFO("Stabilizing activity packet");
    for (int j = 0; j < 200 ; j++) 
/*     for (int j = 0; j < 15 ; j++) 
 */

    {
        /*  Passing all matrices, but the velocity synapse vector.. NOTE */
        mpGridCells->UpdateActivation(mpVelocityCell->FiringRate(), mpVisionCells->FiringRate(), mpHDCells->FiringRate(), mpGridCells_HD_VelocitySynapseSet, mpGridCellsSynapseSet->WeightMatrix(),mpGridCells_VisionSynapseSet->WeightMatrix() );
        mpGridCells->UpdateFiringRate ();
        mpGridCells->UpdateFiringRateTrace ();
        mpGridCells->FiringRates ();

        mLocation.x = floor(mpGridCells->CurrentLocation ()/10);
        mLocation.y = (int)(mpGridCells->CurrentLocation ()) % 10;
        ROS_DEBUG("Location is now: neuron: %f at [%f, %f] in neuron terms",mpGridCells->CurrentLocation (), mLocation.x, mLocation.y);

        /*  Learning still occurs! */
/*         mpGridSynapseSet->UpdateWeight (mpGridCells->FiringRateTrace (), mpGridCells->FiringRate ().transpose ());
 *         mpGridSynapseSet->Normalize ();
 * 
 */

/*         msg.data = mHeadLocation;
 *         mHeadLocationPublisher.publish(msg);
 */
        if((j == 0 ) || (j%10 == 9))
        {
            std::ostringstream ss;
            ss << j;
            mpGridCells->PrintFiringRateToFile((std::string("Stabilized-GridCells-FiringRate-") + ss.str () + std::string(".txt")));
            mpGridCells->PrintActivationToFile((std::string("Stabilized-GridCells-Activation-") + ss.str () + std::string(".txt")));
/*             mpGridSynapseSet->PrintToFile(std::string("Stabilized-Grid-synapse.txt"));
 */
        }
    }

    mIsInitialLocationSet = true;
    ROS_DEBUG("Location set to: [%f, %f] in neuron terms",mLocation.x, mLocation.y);
    mpGridCells->PrintFiringRateToFile(std::string("Stabilized-GridCells-FiringRate.txt"));
    mpGridCells->PrintActivationToFile(std::string("Stabilized-GridCells-Activation.txt"));
    mpGridCellsSynapseSet->PrintToFile(std::string("Stabilized-Grid-synapse.txt"));
}		/* -----  end of method Bionavigator::SetInitialLocation  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: HeadDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::HeadDirection (double angularVelocityZ )
{
/*     ROS_DEBUG("Angular velocity received: %f",angularVelocityZ);
 */
    mpHDCells->InhibitionRate (0.02);
    mpRotationCellClockwise->UpdateFiringRate (angularVelocityZ);
    mpRotationCellCounterClockwise->UpdateFiringRate (angularVelocityZ);

    mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
    mpHDCells->UpdateFiringRate ();
    mpHDCells->UpdateFiringRateTrace ();
    mpHDCells->FiringRates ();


/*     mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate (), mpHDCells->FiringRate ().transpose ());
 *     mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
 *     mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());
 *     mpHDSynapseSet->Normalize ();
 *     mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();
 *     mpHD_RotationCellClockwiseSynapseSet->Normalize ();
 * 
 *     ROS_DEBUG("Inhibition rate set: %f",mpHDCells->InhibitionRate());
 */

    mHeadDirectionPrev = mHeadDirection;
    mHeadDirection = mpHDCells->CurrentHeadDirection ();
/*     ROS_DEBUG_STREAM ("Head direction is now: " << mHeadDirection << " for firing rates: " << std::fixed << mpRotationCellClockwise->FiringRate () << ", " << std::fixed << mpRotationCellCounterClockwise->FiringRate ());
 * 
 *     mDebugFile << "HD:\t" << std::fixed << mHeadDirection << "\t" << "FR: " << "\t" << std::fixed << mpRotationCellClockwise->FiringRate () << "\t" << std::fixed << (-1.0 * mpRotationCellCounterClockwise->FiringRate ().array ()) << std::endl;
 */
    if (mHeadDirection == -1)
    {
        ROS_DEBUG("%s: Something went wrong. Head direction received -1", (mpHDCells->Identifier()).c_str ());
    }

/*     if (mHeadDirection != mHeadDirectionPrev && mHeadDirection == mInitialHeading)
 *         ROS_INFO("%s: Back at initial heading", mpHDCells->Identifier().c_str ());
 */
}		/* -----  end of method Bionavigator::HeadDirection  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: GridLocation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::GridLocation (double velocity )
{
/*     ROS_DEBUG("Angular velocity received: %f",angularVelocityZ);
 */
    mpVelocityCell->UpdateFiringRate (velocity);

    mpGridCells->UpdateActivation(mpVelocityCell->FiringRate(), mpVisionCells->FiringRate(), mpHDCells->FiringRate(), mpGridCells_HD_VelocitySynapseSet, mpGridCellsSynapseSet->WeightMatrix(),mpGridCells_VisionSynapseSet->WeightMatrix()  );
    mpGridCells->UpdateFiringRate ();
    mpGridCells->UpdateFiringRateTrace ();
    mpGridCells->FiringRates ();


    mLocationPrev = mLocation;
    mLocation.x = mpGridCells->CurrentLocation ()/10.0;
    mLocation.y = (int)(mpGridCells->CurrentLocation ()) % 10;
    ROS_DEBUG_STREAM ("Grid location is now: " << mLocation.x << ", " << mLocation.y <<  " for " << "Activation values: [" << mpGridCells->Activation().maxCoeff() << "," << mpGridCells->Activation().minCoeff() << "]" <<  " HD: " << mHeadDirection << " and rotational firing rates: " << std::fixed << mpRotationCellClockwise->FiringRate () << ", " << std::fixed << mpRotationCellCounterClockwise->FiringRate () << " and velocity: " << velocity << " firing rate: " <<  mpVelocityCell->FiringRate ());

    mDebugFile << "GL:\t" << mLocation.x << "\t" << mLocation.y << "\t" <<  "HD:\t" << std::fixed << mHeadDirection << "\t" << "FR: " << "\t" << std::fixed << mpRotationCellClockwise->FiringRate () << "\t" << std::fixed << (-1.0 * mpRotationCellCounterClockwise->FiringRate ().array ()) << "\tVEL:\t" << mpVelocityCell->FiringRate () <<  std::endl;
    if (mLocation.x == -1 || mLocation.y == -1)
    {
        ROS_DEBUG("%s: Something went wrong. Grid location received -1", (mpGridCells->Identifier()).c_str ());
    }

    mGridHeatMap = mGridHeatMap.cwiseMax(mpGridCells->FiringRate ());
    mDebugFile << "Grid heat map:" << mGridHeatMap.transpose () << std::endl;

/*     if (mHeadDirection != mHeadDirectionPrev && mHeadDirection == mInitialHeading)
 *         ROS_INFO("%s: Back at initial heading", mpHDCells->Identifier().c_str ());
 */
    return ;
}		/* -----  end of method Bionavigator::GridLocation  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: CallbackPublishDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::CallbackPublishDirection (const sensor_msgs::Imu::ConstPtr& rImuMessage)
{
    double angular_velocity;

    /*
     * Make sure the system is calibrated
     */
    if( mIsHDCalibrated == false )
    {
        CalibrateHDSet ();
    }

    if ( mIsGridCellSetCalibrated == false )
    {
        CalibrateGridCellSet ();
    }


    /*  Make sure initial direction was set before
     *  we begin processing inputs
     */
    if( mIsInitialDirectionSet == false)
    {
        SetInitialDirection ();
    }

    if (mIsInitialDirectionSet == false)
    {
        SetInitialLocation ();
    }

    /*
     * Calculate the new head direction
     */
    mCount++;
    if (mCount%mProcessFreq ==0)
    {
        HeadDirection (rImuMessage->angular_velocity.z);

        /*  Assume frequency is 100Hz, and we process every 10th packet only */
        /*  v = ut + 0.5at^2 */
        mVelocity = (mVelocity + (rImuMessage->linear_acceleration.y * 0.1));
        GridLocation (mVelocity);

        /*
         * You have to add the data to the struct before you publish it
         */
        std_msgs::Float64 head_message;
        head_message.data = mHeadDirection;
        mHeadDirectionPublisher.publish(head_message);

/*         std_msgs::Float64MultiArray grid_message;
 *         grid_message.layout.dim[0].label = "x location";
 *         grid_message.layout.dim[0].size = 1;
 *         grid_message.layout.dim[0].stride = 1;
 *         grid_message.layout.dim[1].label = "y location";
 *         grid_message.layout.dim[1].size = 1;
 *         grid_message.data[0] = mLocation.x;
 *         grid_message.data[1] = mLocation.y;
 */
    }

    if(mCount%100 == 0)
    {
        std::ostringstream ss;
        ss << mCount;
        mpHDCells->PrintFiringRateToFile((std::string("Running-HDCells-FiringRate-") + ss.str () + std::string(".txt")));
        mpHDCells->PrintActivationToFile((std::string("Running-HDCells-Activation-") + ss.str () + std::string(".txt")));
        mpGridCells->PrintFiringRateToFile((std::string("Running-GridCells-FiringRate-") + ss.str () + std::string(".txt")));
        mpGridCells->PrintActivationToFile((std::string("Running-GridCells-Activation-") + ss.str () + std::string(".txt")));
/*         mpHDSynapseSet->PrintToFile((std::string("Running-HD-synapse-") + ss.str () + std::string(".txt")));
 */

    }

}		/* -----  end of method Bionavigator::CallbackPublishDirection  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: RosInit
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::RosInit ( )
{
    /*
     * Subscribe to ros node
     */
    mSubscriber = mNodeHandle.subscribe("torso_lift_imu/data", 50, &Bionavigator::CallbackPublishDirection, this);
    ROS_ASSERT(mSubscriber);
    ROS_INFO("Subscribed to torso_lift_imu/data");

    /*  Advertise what we want to publish */
    mHeadDirectionPublisher = mNodeHandle.advertise<std_msgs::Float64>("head_direction",10);
    ROS_ASSERT(mHeadDirectionPublisher);
    ROS_INFO("Publishing to /head_direction");

    mLocationPublisher = mNodeHandle.advertise<std_msgs::Float64MultiArray>("grid_location",10);
    ROS_ASSERT(mLocationPublisher);
    ROS_INFO("Publishing to /grid_location");
}		/* -----  end of method Bionavigator::RosInit  ----- */

