/*
 * =====================================================================================
 *
 *       Filename:  HDCells.cpp
 *
 *    Description:  Definition file for HDCells.hpp
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

#include "HDCells.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
HDCells::HDCells ()
{
    mIdentifier = "HD Cells";
    mDimensionX = 100;
    mDimensionY = 1;                            /* This has to be 1 at the moment. Other values are not supported */
    mHasTrace = true;
    mTau = 1;
    mC_HD_ROT = mDimensionX * 2;
    mC_HD = mDimensionX;
    mPhi0 = 8 * mC_HD;
    mPhi1 = 400;
    mPhi2 = 0;
    mAlpha = 0;
    mBeta = 0.1;
    mDeltaT = 0.0001;


}  /* -----  end of method HDCells::HDCells  (constructor)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: Init
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::Init ( )
{
    NeuronSet::Init ();
    mActivation.resize(mDimensionX,mDimensionY);
    mActivation = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mDimensionX, mDimensionY);
}		/* -----  end of method HDCells::Init  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
HDCells::HDCells ( const HDCells &other )
{
}  /* -----  end of method HDCells::HDCells  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  ~HDCells
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
HDCells::~HDCells ()
{
}  /* -----  end of method HDCells::~HDCells  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    HDCells&
HDCells::operator = ( const HDCells &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method HDCells::operator =  (assignment operator)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateActivation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::UpdateActivation (
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> clockwiseRotationCellFiringRate,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> counterClockwiseRotationCellFiringRate,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellFiringRate,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> clockwiseRotationCellSynapses,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> counterClockwiseRotationCellSynapses,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
        )
{
    /*  Because you can't use and modify a matrix simultaneously */
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix;
    temp_matrix.resize(mDimensionX,mDimensionY);
    temp_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    for (double i = 0; i < 1; i += mDeltaT ) {
        temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + (((mDeltaT/mTau * mPhi0/mC_HD) * headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate) + ((mDeltaT/mTau * mPhi1/mC_HD_ROT)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ())) + (((mDeltaT/mTau * mPhi2/mC_HD_ROT) * (visionCellSynapses * mFiringRate) * visionCellFiringRate).matrix ());
        mActivation = temp_matrix;
    }
}		/* -----  end of method HDCells::UpdateActivation  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateActivation
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::UpdateActivation (
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> initialDirectionMatrix,
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix;
    temp_matrix.resize(mDimensionX,mDimensionY);
    temp_matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    for (double i = 0; i < 1; i += mDeltaT ) 
    {
        temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_HD) * ((headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + ((mDeltaT/mTau * initialDirectionMatrix));
        mActivation = temp_matrix;
    }
    ROS_DEBUG("%s: Activation values: [%f, %f]",mIdentifier.c_str (), mActivation.maxCoeff (), mActivation.minCoeff ());

}		/* -----  end of method HDCells::UpdateActivation  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::UpdateFiringRate ( )
{
    /*  Rescale the activation to get a firing rate to get some of it in
     *  negative */
    mFiringRate = (1*(((1 + ((((mActivation.array () - (1.2 * mActivation.minCoeff ())) -mAlpha))*(-2 * mBeta)).exp ()).inverse ()))).matrix ();
    ROS_DEBUG("%s: Firing rate values: [%f, %f]", mIdentifier.c_str (),mFiringRate.maxCoeff (), mFiringRate.minCoeff ());
}		/* -----  end of method HDCells::UpdateFiringRate  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateFiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::UpdateFiringRate (Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> deltaS , double sigmaHD)
{
        mFiringRate = ((((deltaS.array ().abs2 () +1)/(2.0*sigmaHD * sigmaHD))* -1.0).exp ()).matrix (); /* r^(HD)_i -> equation 4 */
}		/* -----  end of method HDCells::UpdateFiringRate  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateFiringRateTrace
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::UpdateFiringRateTrace ( )
{
        mFiringRateTrace = (1 - mEta) * mFiringRate+ mEta * mFiringRateTrace; /* rTrace^(HD)_i -> equation 7 */
}		/* -----  end of method HDCells::UpdateFiringRateTrace  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: CurrentHeadDirection
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    double
HDCells::CurrentHeadDirection ( )
{
    double max_value = mFiringRate.maxCoeff ();

    /**
     * @note Is there a better way of calculating this? Without iterating
     * maybe?
     */
    for (int k = 0; k < mDimensionX; k +=1)
    {
        if (mFiringRate(k,0) == max_value)
            return ((k+1) * mDirectionalRange);
    }
    return -1;
}		/* -----  end of method HDCells::CurrentHeadDirection  ----- */

