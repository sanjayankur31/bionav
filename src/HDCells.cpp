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
    mTau = 1.0;
    mC_HD_ROT = (double)(mDimensionX * 2);
    mC_HD = (double)(mDimensionX);
    mC_HD_V = (double)(mDimensionX);
/*     mPhi0 = (double)(23.0 * mC_HD);
 */
    mPhi0 = (double)(1.0 * mC_HD);
/*     mPhi1 = (double)(1000.0 * mC_HD_ROT);
 */
    mPhi1 = (double)(1.0 * mC_HD_ROT);
    mPhi2 = (double)(16.0 * mC_HD_V);
    mAlpha = 0.0;
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

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix1;
    temp_matrix1.resize(mDimensionX,mDimensionY);
    temp_matrix1 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix2;
    temp_matrix2.resize(mDimensionX,mDimensionY);
    temp_matrix2 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> temp_matrix3;
    temp_matrix3.resize(mDimensionX,mDimensionY);
    temp_matrix3 = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);

    mDeltaT = 1.0/50;

    for (double i = 0; i < 1; i += mDeltaT ) {

        /*         temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_HD) * ((headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + ((mDeltaT/mTau * mPhi1/mC_HD_ROT)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ())) + (((mDeltaT/mTau * mPhi2/mC_HD_ROT) * ((visionCellSynapses * mFiringRate) * visionCellFiringRate)).matrix ());
        */

        /*  Cut out vision for the time being  */
        /*     temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_HD) * ((headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + ((mDeltaT/mTau * mPhi1/mC_HD_ROT)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ()));
        */

        temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation);

        temp_matrix1 = ((mDeltaT/mTau * mPhi0/mC_HD) * ((headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate));
        temp_matrix2 = ((mDeltaT/mTau * mPhi1/mC_HD_ROT)*((((clockwiseRotationCellSynapses * mFiringRate)* clockwiseRotationCellFiringRate).matrix ()) + ((counterClockwiseRotationCellSynapses * mFiringRate)* counterClockwiseRotationCellFiringRate).matrix ()));
        temp_matrix3 = ((mDeltaT/mTau * mPhi2/mC_HD_V) * (visionCellSynapses * visionCellFiringRate).matrix ());

        mActivation = temp_matrix + temp_matrix1 + temp_matrix2 + temp_matrix3;
    }
/* 
 *     ROS_DEBUG("Recurrent term: [%f, %f]" , temp_matrix.maxCoeff (), temp_matrix.minCoeff ());
 *     ROS_DEBUG("Vestibular term: [%f,%f]" , temp_matrix2.maxCoeff (), temp_matrix2.minCoeff ());
 *     ROS_DEBUG("Vision term: [%f,%f]" , temp_matrix3.maxCoeff (), temp_matrix3.minCoeff ());
 */
      ROS_DEBUG("Firing rate term: [%f,%f]" , temp_matrix1.maxCoeff (), temp_matrix1.minCoeff ());
      ROS_DEBUG("%s: Activation values: [%f, %f]", mIdentifier.c_str (),mActivation.maxCoeff (), mActivation.minCoeff ());
  

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
        temp_matrix = ((1.0 - mDeltaT/mTau) * mActivation) + ((mDeltaT/mTau * mPhi0/mC_HD) * ((headCellSynapses.array () - mInhibitionRate).matrix () * mFiringRate)) + (((mDeltaT/mTau)*initialDirectionMatrix));
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
    temp_matrix1 = (-1.0 * mActivation.array ());
    temp_matrix2 = (temp_matrix1.array().exp ());
    temp_matrix3 = (temp_matrix2.array() +1.0);
    mFiringRate = temp_matrix3.array ().inverse ();
        

/*     ROS_DEBUG("Firing term1: [%f, %f]" , temp_matrix.maxCoeff (), temp_matrix.minCoeff ());
 *     ROS_DEBUG("Firing term2: [%f,%f]" , temp_matrix2.maxCoeff (), temp_matrix2.minCoeff ());
 *     ROS_DEBUG("Firing term3: [%f,%f]" , temp_matrix3.maxCoeff (), temp_matrix3.minCoeff ());
 */
/*     ROS_DEBUG("%s: Firing rate values: [%f, %f]", mIdentifier.c_str (),mFiringRate.maxCoeff (), mFiringRate.minCoeff ());
 */

    //mFiringRate = (1+((-1*mActivation.array ()).array().exp ()).array()).inverse ().matrix ();
/*     ROS_DEBUG("%s: Firing rate values: [%f, %f]", mIdentifier.c_str (),mFiringRate.maxCoeff (), mFiringRate.minCoeff ());
 */
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
        mFiringRateTrace = ((1.0 - mEta) * mFiringRate) + (mEta * mFiringRateTrace); /* rTrace^(HD)_i -> equation 7 */
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

