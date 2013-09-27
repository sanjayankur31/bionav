/*
 * =====================================================================================
 *
 *       Filename:  HDSynapseSet.cpp
 *
 *    Description:  Definition file for HDSynapseSet.hpp
 *
 *        Version:  1.0
 *        Created:  26/09/13 16:34:58
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
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::HDSynapseSet ()
{
    mDimensionX = 100;
    mDimensionY = 100;
    mIdentifier = std::string("HDSynapses");
    mIsPlastic = true;
    mEta = 0.9;

}  /* -----  end of method HDSynapseSet::HDSynapseSet  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet
 * Description:  copy constructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::HDSynapseSet ( const HDSynapseSet &other )
{
}  /* -----  end of method HDSynapseSet::HDSynapseSet  (copy constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  ~HDSynapseSet
 * Description:  destructor
 *--------------------------------------------------------------------------------------
 */
HDSynapseSet::~HDSynapseSet ()
{
}  /* -----  end of method HDSynapseSet::~HDSynapseSet  (destructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  operator =
 * Description:  assignment operator
 *--------------------------------------------------------------------------------------
 */
    HDSynapseSet&
HDSynapseSet::operator = ( const HDSynapseSet &other )
{
    if ( this != &other ) {
    }
    return *this;
}  /* -----  end of method HDSynapseSet::operator =  (assignment operator)  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet :: Init
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDSynapseSet::Init (  )
{
    mWeightMatrix.resize(mDimensionX, mDimensionY);
    mWeightMatrix = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);
}		/* -----  end of method HDSynapseSet::Init  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  HDSynapseSet
 *      Method:  HDSynapseSet :: UpdateWeight
 * Description:  
 *--------------------------------------------------------------------------------------
 */
    void
HDSynapseSet::UpdateWeight (Eigen::Matrix<long double, Eigen::Dynamic, 1> preSynapticFiringRate,Eigen::Matrix<long double, Eigen::Dynamic, 1> postSynapticFiringRate )
{
    if (mIsPlastic == true)
    {
        WeightMatrixType delta_w;
        delta_w.resize(mDimensionX, mDimensionY);
        delta_w = mLearningRate * preSynapticFiringRate * postSynapticFiringRate.transpose ();

        mWeightMatrix += delta_w;
        ROS_DEBUG("%s: Synaptic weight updated.", mIdentifier);
    }
    else 
    {
        ROS_DEBUG("%s: Unable to modify stiff synapses!", mIdentifier);
    }
    return ;
}		/* -----  end of method HDSynapseSet::UpdateWeight  ----- */

