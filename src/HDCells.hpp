/*
 * =====================================================================================
 *
 *       Filename:  HDCells.hpp
 *
 *    Description:  Header file for Head direction cell set
 *
 *        Version:  1.0
 *        Created:  09/09/2013 06:38:36 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  HDCells_INC
#define  HDCells_INC

#include "NeuronSet.hpp"
#include <Eigen/Dense>

/*
 * =====================================================================================
 *        Class:  HDCells
 *  Description:  
 * =====================================================================================
 */
/**
 * @class HDCells
 *
 * @brief Class representing head direction cells
 *
 * @todo Decide on value of time constant
 * @todo Calibrate constants
 * @todo Verify forumule implementations
 */

class HDCells: public Bionav::NeuronSet<Eigen::Matrix<long double, Eigen::Dynamic, 1> >
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HDCells ();                             /* constructor      */
        HDCells ( const HDCells &other );   /* copy constructor */
        ~HDCells ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */
        /**
         * @brief Initialize the matrices.
         *
         * The constructor sets a default dimension set. One can modify it and
         * then call Init to resize and initialize the matrices accordingly
         *
         * This assumes that you've set your dimensions. It does no checks at
         * all.
         *
         * @param None
         *
         * @return None
         */
        void Init ();

        /**
         * @brief Update the activation value at each step. 
         *
         * This is the major worker method that does the step integration.
         *
         * @param clockwiseRotationCellFiringRate
         * @param counterclockwiseRoatationCellFiringRate
         * @param visionCellFiringRate
         *
         * @param clockwiseRotationCellSynapses
         * @param counterclockwiseRotationCellSynapses
         * @param headCellSynapses
         * @param visionCellSynapses
         *
         * @return void
         *
         *
         * @note Will need correction. Just a place holder for the formula.
         * Some constants still haven't been defined. I need to see where
         * they'll fall, or whether I should just insert them already scaled
         *
         * For the time being, I assume a single vision cell
         *
         */
        void UpdateActivation (
                long double clockwiseRotationCellFiringRate,
                long double counterclockwiseRotationCellFiringRate,
                //Eigen::Matrix<long double, Eigen::Dynamic, 1> visionCellFiringRate,
                long double,
                Eigen::Matrix<long double, Eigen::Dynamic, 1> clockwiseRotationCellSynapses,
                Eigen::Matrix<long double, Eigen::Dynamic, 1> counterClockwiseRotationCellSynapses,
                Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses,
                Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
                );

        /**
         * @brief Return the head direction of the network
         *
         * @param None
         *
         * @return Current head direction of network
         */
        long double CurrentHeadDirection ();

        /**
         * @brief Update the "directional range" of the system
         *
         * This isn't exactly the same as the directional firing range of head
         * cells. That is learnt during training. This variable just stores the
         * degrees each cell denotes, as a convenience.
         * 
         * @param None
         *
         * @return void
         */
        void UpdateDirectionalRange () {
            mDirectionalRange = 360/mDimensionX;
            ROS_DEBUG("%s: directional range updated to: %Lf", mIdentifier.c_str (), mDirectionalRange);
        }

        virtual void UpdateFiringRate ();
        virtual void UpdateFiringRateTrace ();

        /* ====================  OPERATORS     ======================================= */

        HDCells& operator = ( const HDCells &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        long double mDeltaT;                    /**< For integration step */
        long double mTau;                       /**< Time constant */
        long double mAlpha;                     /**< Alpha in firing rate equation */
        long double mBeta;                      /**< Beta in firing rate equation */
        long double mDirectionalRange;          /**< Directional range of a cell */

        /*  Handle these. Put them in their classes */
        long double mPhi0;
        long double mC_HD;
        long double mInhibitionRate;
        long double mPhi1;
        long double mC_HD_ROT;
        long double mPhi2;

}; /* -----  end of class HDCells  ----- */


#endif   /* ----- #ifndef HDCells_INC  ----- */
