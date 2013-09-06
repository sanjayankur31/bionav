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
    ros::init(argc, argv, "bionav");

    Bionav bionav;

    ros::spin();

    return 0;
}				/* ----------  end of function main  ---------- */

