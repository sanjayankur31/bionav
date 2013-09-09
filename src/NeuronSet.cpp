/*
 * =====================================================================================
 *
 *       Filename:  NeuronSet.cpp
 *
 *    Description:  Definition file for class methods
 *
 *        Version:  1.0
 *        Created:  09/08/2013 02:09:07 PM
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
 *       Class:  NeuronSet
 *      Method:  NeuronSet
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
Bionav::NeuronSet::NeuronSet ()
{
    /*  A default place holder name that must be changed
     *
     *  I set it here for two reasons:
     *
     *  1. It'll tell me if I haven't changed it
     *  2. It'll be used to ensure that the name is only set once
     *
    */
    mIdentifier = std::string("NeuronSet");

}  /* -----  end of method NeuronSet::NeuronSet  (constructor)  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  NeuronSet
 *      Method:  NeuronSet :: SetIdentifier
 * Description:  Set the identifier for this neuron set
 *--------------------------------------------------------------------------------------
 */
    void
Bionav::NeuronSet::SetIdentifier ( std::string identifier )
{
    if(mIdentifier != std::string ("NeuronSet")) {
        ROS_DEBUG ("%s: Identifier already set. Unable to comply!", mIdentifier)
    }
    else {
        mIdentifier = identifier;
        ROS_DEBUG("%s: New identifier set.", mIdentifier);
    }
    return ;
}		/* -----  end of method NeuronSet::SetIdentifier  ----- */

