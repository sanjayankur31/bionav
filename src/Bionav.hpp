/*
 * =====================================================================================
 *
 *       Filename:  Bionav.h
 *
 *    Description:  Header file for the Bionav class
 *
 *        Version:  1.0
 *        Created:  05/09/13 17:26:23
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  Bionav_INC
#define  Bionav_INC

#include <Eigen/Dense>

/*
 * =====================================================================================
 *        Class:  Bionav
 *  Description:  
 * =====================================================================================
 */
/**
 * @brief The main class for Bionav
 *
 * Other classes such as feature classes and roation class will plug into this
 * one. This does all the work.
 * 
 */
class Bionav
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Bionav ();                             /**< constructor */
        ~Bionav ();                             /**< destructor */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

}; /* -----  end of class Bionav  ----- */


#endif   /* ----- #ifndef Bionav_INC  ----- */
