/*
 * =====================================================================================
 *
 *       Filename:  GridCells.cpp
 *
 *    Description:  Definition file for GridCells.hpp
 *
 *        Version:  1.0
 *        Created:  26/09/13 15:45:24
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#include "GridCells.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
GridCells::GridCells ()
{
    mIdentifier = "Grid Cells";
    mDimensionX = 100;
    mDimensionY = 1;                            /* This has to be 1 at the moment. Other values are not supported */
    mHasTrace = true;
    mTau = 1.0;
    mC_P_HD_Vel = (double)(mDimensionX * mDimensionX);
    mC_P = (double)(mDimensionX);
    mC_P_V = (double)(mDimensionX);
/*     mPhi0 = (double)(23.0 * mC_P);
 */
    mPhi0 = (double)(10.0 * mC_P);
/*     mPhi1 = (double)(1000.0 * mC_P_HD_Vel);
 */
    mPhi1 = (double)(10.0 * mC_P_HD_Vel);
    mPhi2 = (double)(1000.0 * mC_P_V);
    mAlpha = 1.5;
    mBeta = 3;
    mDeltaT = 0.0001;
    mEta = 0.1;


}  /* -----  end of method GridCells::GridCells  (constructor)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: Init
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
GridCells::Init ( )
{
    NeuronSet::Init ();
    mActivation.resize(mDimensionX,mDimensionY);
    mActivation = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mDimensionX, mDimensionY);
}		/* -----  end of method GridCells::Init  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
GridCells::GridCells ( const GridCells &other )
{
}  /* -----  end of method GridCells::GridCells  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  ~GridCells
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
GridCells::~GridCells ()
{
}  /* -----  end of method GridCells::~GridCells  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    GridCells&
GridCells::operator = ( const GridCells &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method GridCells::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: UpdateActivation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
GridCells::UpdateActivation (
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> velocityCellFiringRate,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellFiringRate,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellFiringRates,
        std::vector<GridCells_HD_VelocitySynapseSet*> velocityCellSynapses,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> gridCellSynapses,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
        )
{
    /*  Because you can't use and modify a matrix simultaneously */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix;
    temp_matrix.resize(mDimensionX,mDimensionY);
    temp_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix1;
    temp_matrix1.resize(mDimensionX,mDimensionY);
    temp_matrix1 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix2;
    temp_matrix2.resize(mDimensionX,mDimensionY);
    temp_matrix2 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix3;
    temp_matrix3.resize(mDimensionX,mDimensionY);
    temp_matrix3 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix4;
    temp_matrix4.resize(mDimensionX,mDimensionY);
    temp_matrix4 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix5;
    temp_matrix5.resize(mDimensionX,mDimensionY);
    temp_matrix5 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);


    mDeltaT = mTau/100.0;
    for (double i = 0; i < 1; i += mDeltaT ) {

        /*         temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_P) * ((gridCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + ((mDeltaT/mTau * mPhi1/mC_P_HD_Vel)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ())) + (((mDeltaT/mTau * mPhi2/mC_P_HD_Vel) * ((visionCellSynapses * mFiringRate) * visionCellFiringRate)).matrix ());
        */

        /*  Cut out vision for the time being  */
        /*     temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_P) * ((gridCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + ((mDeltaT/mTau * mPhi1/mC_P_HD_Vel)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ()));
        */

        /*  Introduce a leak variable here */
        temp_matrix = (1.00 * (1.0 - mDeltaT/mTau) * mActivation);

        /*  Should inhibition rate be same for all neurons, or should it be
         *  different for each neuron? */
        temp_matrix1 = ((mDeltaT/mTau * mPhi0/mC_P) * ((gridCellSynapses. array () - mInhibitionRate).matrix () * mFiringRate));
        for (int j  = 0 ; j < mDimensionX; j ++)
        {
            temp_matrix5 = velocityCellSynapses[i]->WeightMatrix();
            temp_matrix2 = temp_matrix4 + ((mDeltaT/mTau * mPhi1/mC_P_HD_Vel)*((((temp_matrix5.array () - mInhibitionRate).matrix () * mFiringRate)* headCellFiringRates(i,0)) * velocityCellFiringRate).matrix ());
            temp_matrix4 = temp_matrix2;
        }

        temp_matrix3 = ((mDeltaT/mTau * mPhi2/mC_P_V) * (visionCellSynapses * visionCellFiringRate)).matrix ();

        mActivation = temp_matrix + temp_matrix1 + temp_matrix2 + temp_matrix3 + temp_matrix4;

        UpdateFiringRate ();
        UpdateFiringRateTrace ();
    }

    ROS_DEBUG("%s: Recurrent term: [%f, %f]" , mIdentifier.c_str (), temp_matrix.maxCoeff (), temp_matrix.minCoeff ());
/*     ROS_DEBUG("%s: Velocity term: [%f,%f]" , mIdentifier.c_str (), temp_matrix2.maxCoeff (), temp_matrix2.minCoeff ());
 */
    ROS_DEBUG("%s: Vision term: [%f,%f]" , mIdentifier.c_str (), temp_matrix3.maxCoeff (), temp_matrix3.minCoeff ());

    ROS_DEBUG("%s: Firing rate term: [%f,%f]" , mIdentifier.c_str (),temp_matrix1.maxCoeff (), temp_matrix1.minCoeff ());
    ROS_DEBUG("%s: Activation values: [%f, %f]", mIdentifier.c_str (),mActivation.maxCoeff (), mActivation.minCoeff ());
}		/* -----  end of method GridCells::UpdateActivation  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
GridCells::UpdateFiringRate ( )
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix;
    temp_matrix.resize(mDimensionX,mDimensionY);
    temp_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix1;
    temp_matrix1.resize(mDimensionX,mDimensionY);
    temp_matrix1 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix2;
    temp_matrix2.resize(mDimensionX,mDimensionY);
    temp_matrix2 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix3;
    temp_matrix3.resize(mDimensionX,mDimensionY);
    temp_matrix3 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);
    /*  Rescale the activation to get a firing rate to get some of it in
     *  negative */
/*     mFiringRate = (1*(((1 + ((((mActivation.array () - (1.2 * mActivation.minCoeff ())) -mAlpha))*(-2 * mBeta)).exp ()).inverse ()))).matrix ();
 */
/*     mFiringRate = (1*(((1 + (((mActivation.array () - mAlpha))*(-2 * mBeta))array().exp ()).array().inverse ()))).matrix ();
 */

    /*  Use alpha to ensure that my firing rate value is always between 0.5 and 1, even when Activation goes negative */
    mAlpha = 0.0;
    temp_matrix1 = -1.0 * mBeta * (mActivation.array () - mAlpha);
    temp_matrix2 = (temp_matrix1.array().exp ());
    temp_matrix3 = (temp_matrix2.array() + 1.0);
    mFiringRate = temp_matrix3.array ().inverse ();

    /*  Do not let firing rate be negative. Less than zero means no firing */
    for (int i = 0; i < mDimensionX; i++)
    {
        if (mFiringRate(i,0) < 0)
            mFiringRate(i,0) = 0;
    }


}		/* -----  end of method GridCells::UpdateFiringRate  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
GridCells::UpdateFiringRate (Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> deltaSsq , double sigmaP)
{
    mFiringRate = ((((deltaSsq.array () +1)/(2.0*sigmaP * sigmaP))* -1.0).exp ()).matrix ();

/*     mFiringRate = deltaS.array ();
 */
}		/* -----  end of method GridCells::UpdateFiringRate  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: UpdateFiringRateTrace
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
GridCells::UpdateFiringRateTrace ( )
{
/*     mFiringRateTrace = ((1.0 - mEta) * mFiringRate) + (mEta * mFiringRateTrace);
 */
    mFiringRateTrace = ((1.0 - mEta) * mFiringRate); /* rTrace^(HD)_i -> equation 7 */
}		/* -----  end of method GridCells::UpdateFiringRateTrace  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: CurrentLocation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    double
GridCells::CurrentLocation ( )
{
    double max_value = mFiringRate.maxCoeff ();

    /**
     * @note Is there a better way of calculating this? Without iterating
     * maybe?
     */
    for (int k = 0; k < mDimensionX; k +=1)
    {
        if (mFiringRate(k,0) == max_value)
            return k;

    }
    return -1;
}		/* -----  end of method GridCells::CurrentLocation  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  GridCells
 *      Method:  GridCells :: CurrentLocation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    double
GridCells::CurrentLocation (double dummy)
{
    double max_value = mActivation.maxCoeff ();

    /**
     * @note Is there a better way of calculating this? Without iterating
     * maybe?
     */
    for (int k = 0; k < mDimensionX; k +=1)
    {
        if (mActivation(k,0) == max_value)
            return k;
    }
    return -1;
}		/* -----  end of method GridCells::CurrentLocation  ----- */

