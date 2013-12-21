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

class HDCells: public Bionav::NeuronSet
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        HDCells ();                             /* constructor      */
        HDCells ( const HDCells &other );   /* copy constructor */
        ~HDCells ();                            /* destructor       */

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
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> clockwiseRotationCellFiringRate,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> counterclockwiseRotationCellFiringRate,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellFiringRate,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> clockwiseRotationCellSynapses,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> counterClockwiseRotationCellSynapses,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> visionCellSynapses
                );

        /**
         * @brief Update activation 
         *
         * Used to set initial direction. Sets a small packet of activity and
         * lets it run to stabilize. Once done, the system starts to take in
         * information from the imu and integrate it to publish a result.
         * 
         * It's only slightly different, in that we give a gaussian visual
         * input to the system to force it to take a profile centered at our
         * required reference direction.
         *
         * Since the rotation cells aren't firing here at all, either during
         * forced firing of head cells, or during stabilization, we just strip
         * the actual UpdateActivation function down to smaller bits. I
         * could've used the same formula, but this will save unnecessary
         * matrix computation, where the middle terms will just end up as 0.
         * 
         * @param initialDirectionMatrix Matrix holding an initial peak
         * @param headCellSynapse The now stiff, head direction cann synapse
         * set
         *
         * @return void
         */
        void UpdateActivation(
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> initialDirectionMatrix,
                Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> headCellSynapses
                );

        /**
         * @brief Return the head direction of the network
         *
         * @param None
         *
         * @return Current head direction of network
         */
        double CurrentHeadDirection ();
        /**
         * @brief Return the head direction of the network using activation
         * values rather than firing rate values
         *
         * @param double dummy input to differentiate the two methods
         *
         * @return Current head direction of network
         */
        double CurrentHeadDirection (double dummy);

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
            ROS_INFO("%s: directional range updated to: %f", mIdentifier.c_str (), mDirectionalRange);
        }

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
        void UpdateFiringRate (Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> deltaS, double sigmaHD);

        /**
         * @brief I need to initialize the activation matrix for HDCells also
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

        HDCells& operator = ( const HDCells &other ); /* assignment operator */

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
        double mDirectionalRange;          /**< Directional range of a cell */

        /*  Handle these. Put them in their classes */
        double mPhi0;
        double mC_HD;
        double mInhibitionRate;
        double mPhi1;
        double mC_HD_ROT;
        double mC_HD_V;
        double mPhi2;
        double mSigmaHD;

}; /* -----  end of class HDCells  ----- */


#endif   /* ----- #ifndef HDCells_INC  ----- */
