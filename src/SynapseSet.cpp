/*
 * =====================================================================================
 *
 *       Filename:  SynapseSet.cpp
 *
 *    Description:  Main method body file
 *
 *        Version:  1.0
 *        Created:  09/06/2013 07:33:19 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#include "SynapseSet.hpp"


/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
Bionav::SynapseSet::SynapseSet ()
{
    /*  Default values for constants */
    mHasTrace = false;
    mIsPlastic = false;
    mEta = 0;
    mIdentifier = std::string("SynapseSet");
}  /* -----  end of method SynapseSet::SynapseSet  (constructor)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet :: EnableTrace
 * Description:  Enable the trace matrix
 *--------------------------------------------------------------------------------------
 */
    inline
    void
Bionav::SynapseSet::EnableTrace ( )
{
    if (mHasTrace == true) {
        ROS_DEBUG ("%s: Trace is already enabled!", mIdentifier);
    }
    else{
        mHasTrace = true;
        ROS_DEBUG ("%s: Trace enabled", mIdentifier);
    }
    return ;
}		/* -----  end of method SynapseSet::EnableTrace  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet :: DisableTrace
 * Description:  Disable the trace matrix
 *--------------------------------------------------------------------------------------
 */
    inline
    void
Bionav::SynapseSet::DisableTrace ( )
{
    if (mHasTrace == false) {
        ROS_DEBUG ("%s: Trace is already disabled!", mIdentifier);
    }
    else{
        mHasTrace = false;
        ROS_DEBUG ("%s: Trace disabled", mIdentifier);
    }
}		/* -----  end of method SynapseSet::DisableTrace  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet :: SetPlastic
 * Description:  Set synapse set as plastic
 *--------------------------------------------------------------------------------------
 */
    inline
    void
Bionav::SynapseSet::SetPlastic ( )
{
    if (mIsPlastic == true) {
        ROS_DEBUG ("%s: Synapse is already modifiable!", mIdentifier);
    }
    else {
        mIsPlastic = true;
        ROS_DEBUG ("%s: Synapse set as modifiable!", mIdentifier);
    }
}		/* -----  end of method SynapseSet::SetPlastic  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet :: SetStiff
 * Description:  Set synapse set as stiff
 *--------------------------------------------------------------------------------------
 */
    inline
    void
Bionav::SynapseSet::SetStiff ( )
{
    if (mIsStiff == false) {
        ROS_DEBUG ("%s: Synapse is already stiff!", mIdentifier);
    }
    else {
        mIsStiff = false;
        ROS_DEBUG ("%s: Synapse set as stiff!", mIdentifier);
    }
}		/* -----  end of method SynapseSet::SetStiff  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  SynapseSet
 *      Method:  SynapseSet :: SetIdentifier
 * Description:  Set the identifier of this synapse set
 *--------------------------------------------------------------------------------------
 */
    inline
    void
Bionav::SynapseSet::SetIdentifier ( std::string identifier )
{
    if(mIdentifier != std::string ("SynapseSet")) {
        ROS_DEBUG ("%s: Identifier already set. Unable to comply!", mIdentifier)
    }
    else {
        mIdentifier = identifier;
        ROS_DEBUG("%s: New identifier set.", mIdentifier);
    }
}		/* -----  end of method SynapseSet::SetIdentifier  ----- */

