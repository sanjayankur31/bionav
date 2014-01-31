/*
 * =====================================================================================
 *
 *       Filename:  GridCells.hpp
 *
 *    Description:  Header file for the Grid cells set
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

#ifndef  GridCells_INC
#define  GridCells_INC

#include "NeuronSet.hpp"
#include "GridCells_HD_VelocitySynapseSet.hpp"
#include <Eigen/Dense>

/*
 * =====================================================================================
 *        Class:  GridCells
 *  Description:  
 * =====================================================================================
 */
/**
 * @class GridCells
 *
 * @brief Class representing grid cells
 *
 * @todo Decide on value of time constant
 * @todo Calibrate constants
 * @todo Verify forumule implementations
 */

class GridCells: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        GridCells ();                             /* constructor      */
        GridCells ( const GridCells &other );   /* copy constructor */
        ~GridCells ();                            /* destructor       */

        /* ====================  ACCESSORS     ======================================= */
        /**
         * @brief Accessor to inhibition rate
         *
         * @param None
         *
         * @return inhibitionRate The inhibition rate
         */
        inline double InhibitionRate () { return mInhibitionRate; }

        /* ====================  MUTATORS      ======================================= */

        inline void InhibitionRate (double inhibitionRate) { mInhibitionRate = inhibitionRate; }

        /**
         * @brief Update the activation value at each step. 
         *
         * This is the major worker method that does the step integration.
         *
         * @param velocityCellFiringRate
         * @param visionCellFiringRate
         * @param headCellFiringRates
         *
         * @param velocityCellSynapses
         * @param gridCellSynapses
         * @param visionCellSynapses
         *
         * @return void
         *
         *
         * @note Will need correction. Just a grid holder for the formula.
         * Some constants still haven't been defined. I need to see where
         * they'll fall, or whether I should just insert them already scaled
         *
         * For the time being, I assume a single vision cell
         *
         */
        void UpdateActivation (
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> velocityCellFiringRate,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellFiringRate,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellFiringRates,
                std::vector<GridCells_HD_VelocitySynapseSet*> velocityCellSynapses,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> gridCellSynapses,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
                );

        /**
         * @brief Return the current locationencoded by the network
         *
         * @param None
         *
         * @return Current location of the network
         */
        double CurrentLocation ();
        /**
         * @brief Return the current location of the network using activation
         * values rather than firing rate values
         *
         * @param double dummy input to differentiate the two methods
         *
         * @return Current location of network
         */
        double CurrentLocation (double dummy);

        virtual void UpdateFiringRate ();
        virtual void UpdateFiringRateTrace ();

        /**
         * @brief Update firing rate for training
         *
         * This requires a different equation, hence an overridden method
         *
         * @param deltaS Term in equation
         *
         * @return void
         */
        void UpdateFiringRate (Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> deltaS, double sigmaP);

        /**
         * @brief I need to initialize the activation matrix for GridCells also
         *
         * @param None
         *
         * @return void
         */
        void Init ();

        void FiringRates () 
        {
            ROS_DEBUG("%s: Firing rate values: [%f, %f]. Trace values: [%f,%f]", mIdentifier.c_str(), mFiringRate.maxCoeff (), mFiringRate.minCoeff (), mFiringRateTrace.maxCoeff (), mFiringRateTrace.minCoeff ());
        }

        /* ====================  OPERATORS     ======================================= */

        GridCells& operator = ( const GridCells &other ); /* assignment operator */

    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        double mDeltaT;                    /**< For integration step */
        double mTau;                       /**< Time constant */
        double mAlpha;                     /**< Alpha in firing rate equation */
        double mBeta;                      /**< Beta in firing rate equation */

        /*  Handle these. Put them in their classes */
        double mPhi0;
        double mC_P;
        double mInhibitionRate;
        double mPhi1;
        double mC_P_HD_Vel;
        double mC_P_V;
        double mPhi2;

}; /* -----  end of class GridCells  ----- */


#endif   /* ----- #ifndef GridCells_INC  ----- */
