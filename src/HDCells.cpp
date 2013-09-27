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


}  /* -----  end of method HDCells::HDCells  (constructor)  ----- */

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
 *      Method:  HDCells :: Init
 * Description:  Initialize matrices after dimensions have been set
 *--------------------------------------------------------------------------------------
 */
    void
HDCells::Init ( )
{
    mFiringRate.resize(mDimensionX,mDimensionY);
    mFiringRateTrace.resize(mDimensionX,mDimensionY);
    
    mFiringRateTrace = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic, 1>::Zero (mDimensionX, mDimensionY);
    mFiringRate = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic, 1>::Zero (mDimensionX, mDimensionY);
}		/* -----  end of method HDCells::Init  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: UpdateActivations
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
UpdateActivation (
        long double clockwiseRotationCellFiringRate,
        long double counterclockwiseRotationCellFiringRate,
        Eigen::Matrix<long double, Eigen::Dynamic, 1> visionCellFiringRate,
        Eigen::Matrix<long double, Eigen::Dynamic, 1> clockwiseRotationCellSynapses,
        Eigen::Matrix<long double, Eigen::Dynamic, 1> counterClockwiseRotationCellSynapses,
        Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses,
        Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
        )
{
    /*  Because you can't use and modify a matrix simultaneously */
    Eigen::Matrix<long double, Eigen::Dynamic, 1> temp_matrix;
    temp_matrix.resize(mDimension,1);

    for (long double i = 0; i < 1; i += mDeltaT ) {

        temp_matrix = (((1.0 - mDeltaT/mTau)*mActivation) + ((mDeltaT/mTau)*(mPhi0/mC_HD)*((headCellSynapses.array() - mInhibitionRate).matrix () * mFiringRate)) + (mDeltaT/mTau)*(mPhi1/mC_HD_ROT)*(((clockwiseRotationCellSynapses * mFiringRate).array() * clockwiseRotationCellFiringRate).matrix () + (((counterClockwiseRotationCellSynapses * mFiringRate).array() * counterclockwiseRotationCellFiringRate).matrix ())) + ((mDeltaT/mTau) * (mPhi2/mC_HD) * (visionCellSynapses.array () * mFiringRate.array ()) * visionCellFiringRate).matrix());
        mActivation = temp_matrix;
    }
}		/* -----  end of method HDCells::UpdateActivations  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDCells
 *      Method:  HDCells :: FiringRate
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    Eigen::Matrix<long double, Eigen::Dynamic, 1>
HDCells::FiringRate ( )
{
    mFiringRate = ((((1 + (((mActivation.array () -mAlpha))*(-2 * mBeta)).exp ()).inverse ())).matrix ());

    return mFiringRate;
}		/* -----  end of method HDCells::FiringRate  ----- */

