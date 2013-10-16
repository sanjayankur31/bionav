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

    /*  It might be possible to store these values to use in later runs. The
     *  training is the same in each run. No point doing it again and again. */
    bionavigator->Calibrate ();
    bionavigator->SetInitialDirection ();

    ros::spin();

    return 0;
}				/* ----------  end of function main  ---------- */


