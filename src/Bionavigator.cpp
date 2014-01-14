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

    mpHDSynapseSet = new HDSynapseSet ();
    mpHD_VisionSynapseSet = new HD_VisionSynapseSet ();
    mpHD_RotationCellClockwiseSynapseSet = new HD_RotationSynapseSet ();
    mpHD_RotationCellCounterClockwiseSynapseSet = new HD_RotationSynapseSet ();


    mInitialHeading = 180;
    mCount = 0;
    mCountTillFreq = 0;
    mProcessFreq = 10;
    mPositive = 0;
    mNegative = 0;
    mScale = 1.0;

    /*  sigma controls the width of the gaussian. Smaller sigma, smaller width */
    mSigmaHD = 10;

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

    /*  Set up place cells */
    mpPlaceCells->SetDimension (head_cell_dimension_x, head_cell_dimension_y);
    mpPlaceCells->SetIdentifier (std::string("Place Cells"));
    mpPlaceCells->Init ();
    mpPlaceCells->DisableForceFire ();
    ROS_DEBUG("%s: Initialized", (mpPlaceCells->Identifier()).c_str ());

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

    /*  PlaceCellsSynapseSet */
    mpPlaceCellsSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_x);
    mpPlaceCellsSynapseSet->SetIdentifier(std::string("Place Cells - CANN synapse set"));
    mpPlaceCellsSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpPlaceCellsSynapseSet->Identifier()).c_str ());

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
    mpPlaceCells_HD_VelocitySynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_x);
    mpPlaceCells_HD_VelocitySynapseSet->SetIdentifier(std::string("Place Cells - HD - velocity cell synapse set"));
    mpPlaceCells_HD_VelocitySynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpPlaceCells_HD_VelocitySynapseSet->Identifier()).c_str ());

    mpHD_VisionSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_y);
    mpHD_VisionSynapseSet->SetIdentifier(std::string("HD - Vision cell synapse set"));
    mpHD_VisionSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_VisionSynapseSet->Identifier()).c_str ());

    mpPlaceCells_VisionSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_y);
    mpPlaceCells_VisionSynapseSet->SetIdentifier(std::string("Place Cells - Vision cell synapse set"));
    mpPlaceCells_VisionSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpPlaceCells_VisionSynapseSet->Identifier()).c_str ());


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
    CalibratePlaceCellSet ();
}		/* -----  end of method Bionavigator::Calibrate  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: CalibratePlaceCellSet
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::CalibratePlaceCellSet (  )
{
    ROS_DEBUG("Calibrating Place Cell system");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s_hd;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s_p;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_y;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> threesixty_delta_x;
    double preferred_direction_for_iteration;
    const double PI  =3.141592653589793238462;
    double firing_rate_for_training;
    double place_cells_side_length = sqrt (mpPlaceCells->DimensionX ());
    firing_rate_for_training = 1;

    /*  Ensure all synapses are plastic here */
    mpPlaceCellsSynapseSet->SetPlastic ();
    mpPlaceCells_HD_VelocitySynapseSet->SetPlastic ();

    delta_s_hd.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_s_p.resize(mpPlaceCells->DimensionX (), mpPlaceCells->DimensionY ());

    delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
    threesixty_delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

    /*  We set the preferred head directions uniformly */
    preferred_x.resize(mpPlaceCells->DimensionX (),mpPlaceCells->DimensionY () );
    preferred_y.resize(mpPlaceCells->DimensionX (),mpPlaceCells->DimensionY () );
    preferred_directions.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

    /*  Set up preferred directions of head direction cells */
    for (double i = (360.0/mpHDCells->DimensionX ()), j = 0; i <= 360; i+=(360.0/mpHDCells->DimensionX ()), j++)
    {
        preferred_directions (j, 0) = i;
    }

    /*  Set up preferred locations of place cells */
    for (double j = 0; j < place_cells_side_length; j++)
    {
        for (double k = 0; k < place_cells_side_length; k++)
        {
            preferred_x ((place_cells_side_length * j) + k, 0) = j;
            preferred_y ((place_cells_side_length * j) + k, 0) = k;
        }
    }

    mpPlaceCellsSynapseSet->Init ();
    mpPlaceCells->Init ();

    mpVelocityCell->EnableForceFire (firing_rate_for_training);

    /*  Forward one by one: head cell with initial direction */
    preferred_direction_for_iteration = mInitialHeading;
    delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x = (360 - delta_x.array ());
    delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
    for (int i = 0; i <  mpPlaceCells->DimensionX (); i++)
    {
        std::ostringstream ss;
        ss << i;

        ROS_DEBUG("Place cell calibration iteration: %d",i);
        double preferred_x_for_iteration = preferred_x(i, 0);
        double preferred_y_for_iteration = preferred_y(i, 0);
        ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

        delta_s_p = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpPlaceCells->DimensionX (), 1);

        delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
        delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();

        delta_s_p = (delta_x.array ().sq () + delta_y.array (). sq ()).array ().sqrt ();

        /*  Calculate firing rate for this iteration */
        mpPlaceCells->UpdateFiringRate(delta_s_p, mSigmaP);
        /*  Recurrent synapse update: only required once */
        mpPlaceCellsSynapseSet->UpdateWeight (mpPlaceCells->FiringRate(), mpPlaceCells->FiringRate ().transpose ());

        mpPlaceCells_HD_VelocitySynapseSet->UpdateWeight (mpPlaceCells->FiringRate (), (mpPlaceCells->FiringRateTrace () * mpHDCells->FiringRate () * mpVelocityCell->FiringRate ()).transpose ());

        /*  Activate trace. This will hold f(t-1) always */
        mpPlaceCells->UpdateFiringRateTrace ();

        mpPlaceCells->PrintFiringRateToFile((std::string("Calibrating-PlaceCells-FiringRate-1-") + ss.str () + std::string(".txt")));
        mpPlaceCellsSynapseSet->PrintToFile((std::string("Calibrating-Place-synapse-1-") + ss.str () + std::string(".txt")));
        mpPlaceCells_HD_VelocitySynapseSet->PrintToFile((std::string("Calibrating-Place-HD-V-synapse-1-") + ss.str () + std::string(".txt")));

    }
    mpPlaceCellsSynapseSet->PrintToFile(std::string("Calibrated-Place-synapse-1.txt"));

    /*  right one by one: head cell with initial direction - 90 */
    preferred_direction_for_iteration = mInitialHeading - 90;

    if (preferred_direction_for_iteration < 0)
        preferred_direction_for_iteration += 360;

    delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
    threesixty_delta_x = (360 - delta_x.array ());
    delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

    mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
    for (int i = 0; i <  place_cells_side_length; i++)
    {
        for (int j = 0; j < place_cells_side_length; j++)
        {

            std::ostringstream ss;
            ss << ((place_cells_side_length * i) + j);

            ROS_DEBUG("Place cell calibration iteration: %d",j);
            double preferred_x_for_iteration = preferred_x(j, 0);
            double preferred_y_for_iteration = preferred_y(i, 0);
            ROS_DEBUG("Preferred x,y for this iteration is: %f, %f",preferred_x_for_iteration, preferred_y_for_iteration);

            delta_s_p = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpPlaceCells->DimensionX (), 1);

            delta_x = (preferred_x.array () - preferred_x_for_iteration).abs ();
            delta_y = (preferred_y.array () - preferred_y_for_iteration).abs ();

            delta_s_p = (delta_x.array ().sq () + delta_y.array (). sq ()).array ().sqrt ();

            /*  Calculate firing rate for this iteration */
            mpPlaceCells->UpdateFiringRate(delta_s_p, mSigmaP);
            /*  Recurrent synapse update: only required once */
            mpPlaceCellsSynapseSet->UpdateWeight (mpPlaceCells->FiringRate(), mpPlaceCells->FiringRate ().transpose ());

            mpPlaceCells_HD_VelocitySynapseSet->UpdateWeight (mpPlaceCells->FiringRate (), (mpPlaceCells->FiringRateTrace () * mpHDCells->FiringRate () * mpVelocityCell->FiringRate ()).transpose ());

            /*  Activate trace. This will hold f(t-1) always */
            mpPlaceCells->UpdateFiringRateTrace ();

            mpPlaceCells->PrintFiringRateToFile((std::string("Calibrating-PlaceCells-FiringRate-2-") + ss.str () + std::string(".txt")));
            mpPlaceCellsSynapseSet->PrintToFile((std::string("Calibrating-Place-synapse-2-") + ss.str () + std::string(".txt")));
            mpPlaceCells_HD_VelocitySynapseSet->PrintToFile((std::string("Calibrating-Place-HD-V-synapse-") + ss.str () + std::string(".txt")));
        }
        mpPlaceCellsSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-2.txt"));
    }

/*     mpPlaceCells_VelocitySynapseSet->Normalize ();
 */
    mpPlaceCellsSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-final.txt"));

    ROS_DEBUG("%s calibrated to: [%f,%f]",mpPlaceCells_VelocitySynapseSet->Identifier().c_str (), mpPlaceCells_VelocitySynapseSet->Max (), mpPlaceCells_VelocitySynapseSet->Min ());

    /*  Disable all the force firing */
    mpVelocityCell->DisableForceFire ();
    mpPlaceCells->DisableForceFire ();

    mpPlaceCells_VelocitySynapseSet->PrintToFile(std::string("Calibrated-RotationCellClockwise-synapse.txt"));

    /*  Switch to LTP/D */
    mpPlaceCellsSynapseSet->LearningRate (0.00);
    mpPlaceCells_VelocitySynapseSet->LearningRate (0.0);

    mIsPlaceCellSetCalibrated = true;
    ROS_DEBUG("Place cell Calibration complete");
}		/* -----  end of method Bionavigator::CalibratePlaceCellSet  ----- */

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
 */
    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-1.txt"));
/*     mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();
 */
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
/*     mpHDSynapseSet->Normalize ();
 */
    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse-2.txt"));
    mpHDSynapseSet->AddToWeight(temp_hd_weights);

/*     mpHD_RotationCellClockwiseSynapseSet->Normalize ();
 */
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

//    delta_x = (preferred_directions.array () - mInitialHeading).abs ();
//    threesixty_delta_x = (360 - delta_x.array ());
//
//    delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

    /*  Set up matrices for the equation */
    /*  25 -> height of bells peak. I'm trying to stretch it vertically to
     *  increase the difference in the max and min values of the activation
     *  that is set up. The width of the gaussian is controlled by sigma.  */
//    initial_direction_matrix = (((((delta_s.array ().abs2 ())/(2.0*mSigmaHD*mSigmaHD))* -1.0).exp ())).matrix (); /* r^(HD)_i at time 0 */
    /*
     * Set the synaptic weights of the vision head synapses to the weights
     * learned during training for the cann recurrent network.
     *
     * This will ensure a peak at the preferred direction.
     */
    mpHD_VisionSynapseSet->SetPlastic ();
/*     initial_direction_matrix = (mpHDSynapseSet->WeightMatrix ()).row (((mInitialHeading * mpHDCells->DimensionX ())/360.0) -1.0).transpose ();
 */
    initial_direction_matrix = (mpHDSynapseSet->WeightMatrix ()).col (((mInitialHeading * mpHDCells->DimensionX ())/360.0));
    mpHD_VisionSynapseSet->AddToWeight((initial_direction_matrix.array ()/initial_direction_matrix.maxCoeff ()).matrix());

/*     initial_direction_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (),mpHDCells->DimensionY ());
 *     initial_direction_matrix(49,0) = 10;
 */
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
        mpHDCells->InhibitionRate (0.2);
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
        mpHDCells->InhibitionRate (0.2);
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
 *      Method:  Bionavigator :: HeadDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::HeadDirection (double angularVelocityZ )
{
/*     ROS_DEBUG("Angular velocity received: %f",angularVelocityZ);
 */
    mpHDCells->InhibitionRate (0.2);
    mpRotationCellClockwise->UpdateFiringRate (angularVelocityZ);
    mpRotationCellCounterClockwise->UpdateFiringRate (angularVelocityZ);

    mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
    mpHDCells->UpdateFiringRate ();
    mpHDCells->UpdateFiringRateTrace ();
    mpHDCells->FiringRates ();


    mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate (), mpHDCells->FiringRate ().transpose ());
    mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
    mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());
    mpHDSynapseSet->Normalize ();
    mpHD_RotationCellCounterClockwiseSynapseSet->Normalize ();
    mpHD_RotationCellClockwiseSynapseSet->Normalize ();

    ROS_DEBUG("Inhibition rate set: %f",mpHDCells->InhibitionRate());

    mHeadDirectionPrev = mHeadDirection;
    mHeadDirection = mpHDCells->CurrentHeadDirection ();
    ROS_DEBUG_STREAM ("Head direction is now: " << mHeadDirection << " for firing rates: " << std::fixed << mpRotationCellClockwise->FiringRate () << ", " << std::fixed << mpRotationCellCounterClockwise->FiringRate ());

    mDebugFile << "HD:\t" << std::fixed << mHeadDirection << "\t" << "FR: " << "\t" << std::fixed << mpRotationCellClockwise->FiringRate () << "\t" << std::fixed << (-1.0 * mpRotationCellCounterClockwise->FiringRate ().array ()) << std::endl;
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
    if( mIsCalibrated == false )
    {
        Calibrate ();
    }


    /*  Make sure initial direction was set before
     *  we begin processing inputs
     */
    if( mIsInitialDirectionSet == false)
    {
        SetInitialDirection ();
    }



    /*  Only process every fifth packet. 
     *  This needs to be tinkered with and optimised
     */
/*     mpAngularVelocityArray[mCountTillFreq] = rImuMessage->angular_velocity.z;
 *     mCount++;
 *     mCountTillFreq++;
 *     if (mCountTillFreq  == mProcessFreq)
 *     {
 *         mCountTillFreq = 0;
 * 
 *         for (int i = 0; i< mProcessFreq; i++)
 *         {
 *             angular_velocity += mpAngularVelocityArray[i];
 *             mpAngularVelocityArray[i] = 0.0;
 *         }
 *         angular_velocity /= (1.0 * mProcessFreq);
 */
        /*
         * Calculate the new head direction
         */
    mCount++;
    if (mCount%mProcessFreq ==0)
    {
        HeadDirection (rImuMessage->angular_velocity.z);

        //HeadDirection (angular_velocity);

        if (rImuMessage->angular_velocity.z > 0)
       // if (angular_velocity> 0)
            mPositive++;
        else
            mNegative++;

        /*
         * You have to add the data to the struct before you publish it
         */
        std_msgs::Float64 msg;
        msg.data = mHeadDirection;
        mHeadDirectionPublisher.publish(msg);
/*         ROS_DEBUG("Angular velocity stats: [+%d,-%d]", mPositive, mNegative);
 */
    }

    if(mCount%100 == 0)
    {
        std::ostringstream ss;
        ss << mCount;
        mpHDCells->PrintFiringRateToFile((std::string("Running-HDCells-FiringRate-") + ss.str () + std::string(".txt")));
        mpHDCells->PrintActivationToFile((std::string("Running-HDCells-Activation-") + ss.str () + std::string(".txt")));
        mpHDSynapseSet->PrintToFile((std::string("Running-HD-synapse-") + ss.str () + std::string(".txt")));

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

}		/* -----  end of method Bionavigator::RosInit  ----- */

