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
    mPositive = 0;
    mNegative = 0;

    /*  sigma controls the width of the gaussian. Smaller sigma, smaller width */
    mSigmaHD = 10;


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

    mpHD_VisionSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_y);
    mpHD_VisionSynapseSet->SetIdentifier(std::string("HD - Vision cell synapse set"));
    mpHD_VisionSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_VisionSynapseSet->Identifier()).c_str ());



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
    mpHDSynapseSet->SetBounding(0.0);
    mpHD_RotationCellClockwiseSynapseSet->SetBounding(0.0);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetBounding(0.0);
    mpHDSynapseSet->SetDecay(0.0);
    mpHD_RotationCellClockwiseSynapseSet->SetDecay(0.0);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetDecay(0.0);

    ROS_DEBUG("Calibrating system");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
    const double PI  =3.141592653589793238462;
    double firing_rate_for_training = (2.0*PI)/mpHDCells->DimensionX ();

    /*  Ensure all synapses are plastic here */
    mpHDSynapseSet->SetPlastic ();
    mpHD_RotationCellClockwiseSynapseSet->SetPlastic ();
    mpHD_RotationCellCounterClockwiseSynapseSet->SetPlastic ();


    delta_s.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());
    delta_s = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ());           /* s^(HD)_i: difference between actual head direction and preferred head direction */

    /*  We set the preferred head directions uniformly */
    preferred_directions.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
    preferred_directions = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (),mpHDCells->DimensionY ());
    for (double i = (1.0* 360.0/mpHDCells->DimensionX ()), j = 0; i <= 360; i+=(360.0/mpHDCells->DimensionX ()), j++)
    {
        preferred_directions (j, 0) = i;
    }
    

    /*  Anti clockwise */
    mpHDCells->Init ();

    mpRotationCellCounterClockwise->EnableForceFire (firing_rate_for_training);
/*     ROS_DEBUG_STREAM("" << mpRotationCellCounterClockwise->Identifier ().c_str () << ": Firing rate is: " << mpRotationCellCounterClockwise->FiringRate ());
 */
    mpRotationCellClockwise->DisableForceFire ();
    for (int i = mpHDCells->DimensionX (); i >= 1;  i--)
    {
        ROS_DEBUG("Counter Clockwise calibrtion iteration: %d",i);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_x;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> threesixty_delta_x;
        double preferred_direction_for_iteration = preferred_directions(i -1,0);

        delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );
        threesixty_delta_x.resize(mpHDCells->DimensionX (),mpHDCells->DimensionY () );

        delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* |x_i - x| */
        threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()); /* 360 - |x_i - x| */

        delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
        threesixty_delta_x = (360 - delta_x.array ());

        delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

        /*  For rotation cell connections only! */
        mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
        mpHDCells->UpdateFiringRateTrace ();

        /*  Pass transposed etc matrices. The update weight will only do simple
         *  multiplication. This is clearer */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());

        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());

        /*  Clockwise calibration not needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
    }
    mpRotationCellCounterClockwise->DisableForceFire ();
    ROS_DEBUG("%s calibrated to: [%f,%f]",mpHD_RotationCellCounterClockwiseSynapseSet->Identifier().c_str (), mpHD_RotationCellCounterClockwiseSynapseSet->Max (), mpHD_RotationCellCounterClockwiseSynapseSet->Min ());

    /*
     * Clockwise
     */
    mpHDCells->Init ();
    mpRotationCellClockwise->EnableForceFire (firing_rate_for_training);
/*     ROS_DEBUG_STREAM("" << mpRotationCellClockwise->Identifier ().c_str () << ": Firing rate is: " << mpRotationCellClockwise->FiringRate ());
 */
    for (int i = 1; i <= mpHDCells->DimensionX (); i++)
    {
        ROS_DEBUG("Clockwise calibrtion iteration: %d",i);
        Eigen::Matrix<double, Eigen::Dynamic, 1> delta_x;
        Eigen::Matrix<double, Eigen::Dynamic, 1> threesixty_delta_x;
        double preferred_direction_for_iteration = preferred_directions(i -1,0);

        delta_x.resize(mpHDCells->DimensionX (), 1);
        threesixty_delta_x.resize(mpHDCells->DimensionX (), 1);

        delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* |x_i - x| */
        threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* 360 - |x_i - x| */

        delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
        threesixty_delta_x = (360 - delta_x.array ());

        delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

        /*  For rotation cell connections only! */
        mpHDCells->UpdateFiringRate(delta_s, mSigmaHD);
        mpHDCells->UpdateFiringRateTrace ();

        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());

        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());

        /*  Counter clockwise stuff needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
    }
    ROS_DEBUG("%s calibrated to: [%f,%f]",mpHD_RotationCellClockwiseSynapseSet->Identifier().c_str (), mpHD_RotationCellClockwiseSynapseSet->Max (), mpHD_RotationCellClockwiseSynapseSet->Min ());
/*     mHDCannWeightMatrix *= (1.0/3.6);
 */

    /*  rescale  */
/*     mpHDSynapseSet->Rescale (0.8);
 *     mpHD_RotationCellClockwiseSynapseSet->Rescale (0.8);
 *     mpHD_RotationCellCounterClockwiseSynapseSet->Rescale (0.8);
 */

    /*  WHY RESCALE? */

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
    mIsCalibrated = true;

    ROS_DEBUG("Calibration complete");
    mpHDSynapseSet->PrintToFile(std::string("Calibrated-HD-synapse.txt"));
    mpHD_RotationCellCounterClockwiseSynapseSet->PrintToFile(std::string("Calibrated-HD-RotationCellCounterClockwise-synapse.txt"));
    mpHDSynapseSet->SetBounding(1.0);
    mpHD_RotationCellClockwiseSynapseSet->SetBounding(1.0);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetBounding(1.0);
    mpHDSynapseSet->SetDecay(0.2);
    mpHD_RotationCellClockwiseSynapseSet->SetDecay(0.2);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetDecay(0.2);
}		/* -----  end of method Bionavigator::Calibrate  ----- */

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
//    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> initial_direction_matrix; /**< @f$I^{V}_i@f$  */

    /*  For the message to be published */
    std_msgs::Float64 msg;

//    initial_direction_matrix.resize(mpHDCells->DimensionX (), mpHDCells->DimensionY ());


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
    mpHD_VisionSynapseSet->AddToWeight((mpHDSynapseSet->WeightMatrix ()).col (((mInitialHeading * mpHDCells->DimensionX ())/360.0) -1.0).array ());
    mpHD_VisionSynapseSet->Rescale (0.8);
    mpHD_VisionSynapseSet->SetStiff ();
    mpVisionCells->EnableForceFire ();

    /*
     * Find a good number of iterations. Optimize it.
     */
    ROS_INFO("Forcing an initial direction to %f", mInitialHeading);
    for (double i = 0; i < 1 ; i++ ) 
    {
//        mpHDCells->UpdateActivation (initial_direction_matrix, mpHDSynapseSet->WeightMatrix ());

        mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
        mpHDCells->UpdateFiringRate ();
    }
    mHeadDirection = mpHDCells->CurrentHeadDirection ();
    ROS_DEBUG("Head direction is now: %f",mHeadDirection);
    msg.data = mHeadDirection;
    mHeadDirectionPublisher.publish(msg);
    mpVisionCells->DisableForceFire ();

    /*
     * Find a good number of loops for this
     */
    ROS_INFO("Stabilizing activity packet");
    for (double j = 0; j < 100 ; j++) 
    {
        mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
/*         mpHDCells->UpdateActivation ( Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), mpHDCells->DimensionY ()), mpHDSynapseSet->WeightMatrix ());
 */
        mpHDCells->UpdateFiringRate ();
        mpHDCells->UpdateFiringRateTrace ();
        mHeadDirection = mpHDCells->CurrentHeadDirection ();
        ROS_DEBUG("Head direction is now: %f",mHeadDirection);

        /*  Learning still occurs! */

        /* Pre synaptic firing should be slightly less than post synaptic
         * firing since it's slightly decayed by now, for learning to occur as
         * per hebb's learning rule
         */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());
        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());

        mpHDCells->InhibitionRate ((0.14*mpHDSynapseSet->WeightMatrix().maxCoeff ()));
        ROS_DEBUG("Inhibition rate set: %f",mpHDCells->InhibitionRate());
/*         msg.data = mHeadDirection;
 *         mHeadDirectionPublisher.publish(msg);
 */
    }

    mIsInitialDirectionSet = true;
    ROS_INFO("Initial direction set to %f", mHeadDirection);
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
    mpRotationCellClockwise->UpdateFiringRate (angularVelocityZ);
    mpRotationCellCounterClockwise->UpdateFiringRate (angularVelocityZ);

    mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );
    mpHDCells->UpdateFiringRate ();
    mpHDCells->UpdateFiringRateTrace ();

    mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRateTrace (), mpHDCells->FiringRate ().transpose ());
    mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellCounterClockwise->FiringRate ()).transpose ());
    mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace () * mpRotationCellClockwise->FiringRate ()).transpose ());

    mpHDCells->InhibitionRate ((0.14*mpHDSynapseSet->WeightMatrix().maxCoeff ()));
    ROS_DEBUG("Inhibition rate set: %f",mpHDCells->InhibitionRate());

    mHeadDirectionPrev = mHeadDirection;
    mHeadDirection = mpHDCells->CurrentHeadDirection ();
    ROS_DEBUG_STREAM ("Head direction is now: " << mHeadDirection << " for firing rates: " << std::fixed << mpRotationCellClockwise->FiringRate () << ", " << std::fixed << mpRotationCellCounterClockwise->FiringRate ());

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
    mCount++;
    if ((mCount%5) == 0)
    {
        /*
         * Calculate the new head direction
         */
        HeadDirection (rImuMessage->angular_velocity.z);

        if (rImuMessage->angular_velocity.z > 0)
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

