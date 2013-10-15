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
    int head_cell_dimension_x = 100;
    int head_cell_dimension_y = 1;

    /*  Set up HD cells */
    mpHDCells->SetDimension (head_cell_dimension_x, head_cell_dimension_y);
    mpHDCells->UpdateDirectionalRange ();
    mpHDCells->SetIdentifier (std::string("HD Cells"));
    mpHDCells->Init ();
    ROS_DEBUG("%s: Initialized", (mpHDCells->Identifier()).c_str ());
    /*  Don't need to do this for rotation cells, since they are two
     *  individual cells at the moment
     */

    /*  Set up HDSynapseSet */
    mpHDSynapseSet->SetDimension(head_cell_dimension_x, head_cell_dimension_x);
    mpHDSynapseSet->SetIdentifier(std::string("HD - CANN synapse set"));
    mpHDSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHDSynapseSet->Identifier()).c_str ());


    mpHD_RotationCellCounterClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_y);
    mpHD_RotationCellCounterClockwiseSynapseSet->SetIdentifier(std::string("HD - Rotation cell counter clockwise synapse set"));
    mpHD_RotationCellCounterClockwiseSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_RotationCellCounterClockwiseSynapseSet->Identifier()).c_str ());

    mpHD_RotationCellClockwiseSynapseSet->SetDimension(head_cell_dimension_x,head_cell_dimension_y);
    mpHD_RotationCellClockwiseSynapseSet->SetIdentifier(std::string("HD - Rotation cell clockwise synapse set"));
    mpHD_RotationCellClockwiseSynapseSet->Init ();
    ROS_DEBUG("%s: Initialized", (mpHD_RotationCellClockwiseSynapseSet->Identifier()).c_str ());

    /**
     * @todo Vision cell initialization
     * 
     */


    /*
     * Subscribe to ros node
     */
    mSubscriber = mNodeHandle.subscribe("torso_lift_imu/data", 50, &Bionavigator::CallbackPublishDirection, this);
    ROS_ASSERT(mSubscriber);
    ROS_DEBUG("Subscribed to torso_lift_imu/data");

    /*  Can I use a double? */
    mHeadDirectionPublisher = mNodeHandle.advertise<std_msgs::Float64>("head_direction",10);
    ROS_ASSERT(mHeadDirectionPublisher);
    ROS_DEBUG("Publishing to /head_direction");


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
    ROS_DEBUG("Calibrating system");
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_s;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preferred_directions;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_w;

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
    mpRotationCellCounterClockwise->EnableForceFire ();
    mpRotationCellClockwise->DisableForceFire ();
    for (int i = 1; i <= mpHDCells->DimensionX (); i++)
    {
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
        mpHDCells->UpdateFiringRate(delta_s);
        mpHDCells->UpdateFiringRateTrace ();

        /*  Pass transposed etc matrices. The update weight will only do simple
         *  multiplication. This is clearer */
        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate (), mpHDCells->FiringRate ().transpose ());

        mpHD_RotationCellCounterClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace ().transpose () * mpRotationCellCounterClockwise->FiringRate ()));
        //mpHD_RotationCellCounterClockwiseSynapseSet->AddToWeight(delta_w.array ());

        /*  Not needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
        //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_w = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero ( mpHDCells->DimensionX (), mpHDCells->DimensionX () ); /* deltaW^(RC)_(ij) and delta_W^(ROT)_(ij) 1 and 2 separately */
        //delta_w = mLearningRate * mpHDCells->mFiringRateMatrix * mpHDCells->mFiringRateTraceMatrix.transpose () * mRotationCellClockwise->FiringRate (); /* deltaW^(ROT)_(ij2) -> equation 11 */
        //mHC_RCCounterClockwiseWeightMatrix += delta_w;
    }
    /*
     * Clockwise
     */
    mpRotationCellCounterClockwise->DisableForceFire ();
    mpRotationCellClockwise->EnableForceFire ();
    for (int i = 1; i <= mpHDCells->DimensionX (); i++)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 1> delta_x;
        Eigen::Matrix<double, Eigen::Dynamic, 1> threesixty_delta_x;
        double preferred_direction_for_iteration = preferred_directions(i -1,0);

        delta_x.resize(mpHDCells->DimensionX (), 1);
        threesixty_delta_x.resize(mpHDCells->DimensionX (), 1);

        delta_w.resize(mpHDCells->DimensionX (), mpHDCells->DimensionX ());
        delta_w = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero ( mpHDCells->DimensionX (), mpHDCells->DimensionX () ); /* deltaW^(RC)_(ij) and delta_W^(ROT)_(ij) 1 and 2 separately */
        delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* |x_i - x| */
        threesixty_delta_x = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mpHDCells->DimensionX (), 1); /* 360 - |x_i - x| */

        delta_x = (preferred_directions.array () - preferred_direction_for_iteration).abs ();
        threesixty_delta_x = (360 - delta_x.array ());

        delta_s = delta_x.cwiseMin (threesixty_delta_x); /* We have s^(HD)_i */

        /*  For rotation cell connections only! */
        mpHDCells->UpdateFiringRate(delta_s);
        mpHDCells->UpdateFiringRateTrace ();

        mpHDSynapseSet->UpdateWeight (mpHDCells->FiringRate (), mpHDCells->FiringRate ().transpose ());

        mpHD_RotationCellClockwiseSynapseSet->UpdateWeight (mpHDCells->FiringRate (), (mpHDCells->FiringRateTrace ().transpose () * mpRotationCellClockwise->FiringRate ()));

        /*  Not needed in this cycle. Save some computations, instead of it
         *  multiplying be zero in the end */
        //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> delta_w = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero ( mpHDCells->DimensionX (), mpHDCells->DimensionX () ); /* deltaW^(RC)_(ij) and delta_W^(ROT)_(ij) 1 and 2 separately */
        //delta_w = mLearningRate * mpHDCells->mFiringRateMatrix * mpHDCells->mFiringRateTraceMatrix.transpose () * mRotationCellClockwise->FiringRate (); /* deltaW^(ROT)_(ij2) -> equation 11 */
        //mHC_RCCounterClockwiseWeightMatrix += delta_w;
    }
/*     mHDCannWeightMatrix *= (1.0/3.6);
 */

    /*  rescale  */
/*     double temp = mHDCannWeightMatrix.maxCoeff ();
 *     mHDCannWeightMatrix /= temp;
 * 
 *     temp = mHC_RCClockwiseWeightMatrix.maxCoeff ();
 *     mHC_RCClockwiseWeightMatrix /= temp;
 * 
 *     temp = mHC_RCCounterClockwiseWeightMatrix.maxCoeff ();
 *     mHC_RCCounterClockwiseWeightMatrix /= temp;
 * 
 *     mHC_RCClockwiseWeightMatrix *= 0.8;
 *     mHC_RCCounterClockwiseWeightMatrix *= 0.8;
 *     mHDCannWeightMatrix *= 0.8;
 */

    /*  Set the inhibition rate */
    mInhibitionRate = (0.4 * mpHDSynapseSet->Max ());

    /*  Disable all the force firing */
    mpRotationCellClockwise->DisableForceFire ();
    mpRotationCellCounterClockwise->DisableForceFire ();

    mIsCalibrated = true;
}		/* -----  end of method Bionavigator::Calibrate  ----- */



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


    /*
     * Calculate the new head direction
     */
/*     HeadDirection ();
 */

/*     mHeadDirectionPublisher.publish(mHeadDirection);
 */

    /*
     * You have to add the data to the struct before you publish it
     */
    std_msgs::Float64 msg;
    msg.data = 0.002;
    mHeadDirectionPublisher.publish(msg);




}		/* -----  end of method Bionavigator::CallbackPublishDirection  ----- */



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
    mIsInitialDirectionSet = true;
}		/* -----  end of method Bionavigator::SetInitialDirection  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  Bionavigator
 *      Method:  Bionavigator :: HeadDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
Bionavigator::HeadDirection ( )
{
    mpHDCells->UpdateActivation(mpRotationCellClockwise->FiringRate(), mpRotationCellCounterClockwise->FiringRate(), mpVisionCells->FiringRate(), mpHD_RotationCellClockwiseSynapseSet->WeightMatrix(), mpHD_RotationCellCounterClockwiseSynapseSet->WeightMatrix(),mpHDSynapseSet->WeightMatrix(), mpHD_VisionSynapseSet->WeightMatrix()  );

    mHeadDirection = mpHDCells->CurrentHeadDirection ();

    if (mHeadDirection == -1)
    {
        ROS_DEBUG("%s: Something went wrong. Head direction received -1", (mpHDCells->Identifier()).c_str ());
    }
}		/* -----  end of method Bionavigator::HeadDirection  ----- */


