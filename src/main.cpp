/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  Main file for bionav
 *
 *        Version:  1.0
 *        Created:  05/09/13 17:08:37
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */


#include "main.hpp"

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  main function for bionav
 * =====================================================================================
 */
    int
main ( int argc, char **argv )
{
    ros::init(argc, argv, "bionav", ros::init_options::NoSigintHandler);

    Bionavigator* bionavigator = new Bionavigator ();
    bionavigator->Init ();
    bionavigator->RosInit ();

    /*  It might be possible to store these values to use in later runs. The
     *  training is the same in each run. No point doing it again and again. */
    bionavigator->Calibrate ();
    /*  Set location first because we don't want head direction cells
     *  projecting on grid cells at this point. The velocity cell isn't firing
     *  so this shouldn't be a problem any way */
    bionavigator->SetInitialLocation ();
    bionavigator->SetInitialDirection ();


    ros::spin();



    return 0;
}				/* ----------  end of function main  ---------- */


