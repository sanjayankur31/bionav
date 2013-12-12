/*
 * =====================================================================================
 *
 *       Filename:  SynapseSet.hpp
 *
 *    Description:  Header for SynapseSet class
 *
 *        Version:  1.0
 *        Created:  06/09/13 14:50:37
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */

#ifndef  SynapseSet_INC
#define  SynapseSet_INC

#include <iostream>
#include <fstream>
#include <iomanip>
#include "ros/ros.h"
#include <Eigen/Dense>

namespace Bionav {

    /*
     * =====================================================================================
     *        Class:  SynapseSet
     *  Description:  An abstract class to be extended for all the sets of
     *  synapse required. 
     *
     * =====================================================================================
     */
    /**
     * @class SynapseSet
     *
     * @brief The class represents a set of synapses between two neuron sets.
     *
     * Ideally, each synapse should be modelled separately, but in our case we
     * don't deal with individual synapses. We deal with sets of synapses
     * between sets of neurons.
     *
     * This class provides the basic framework and enforces the definition of a
     * set of mandator methods.
     */
    class SynapseSet
    {
        public:
            /* ====================  LIFECYCLE     ======================================= */
            SynapseSet ()                             /**< constructor */
            {
                /*  Default values for constants */
                mIsPlastic = false;
                mIdentifier = std::string("SynapseSet");
                mLearningRate = 1;
                mDecayRate = 0.03; 
                mWeightIsBound = 1;
            }

            ~SynapseSet () { ;}                             /**< destructor */

            /* ====================  ACCESSORS     ======================================= */
            /**
             * @brief return Y dimension
             *
             * @param None
             *
             * @return mDimensionY Y dimension
             */
            double DimensionY () { return mDimensionY; }

            /**
             * @brief return X dimension
             *
             * @param None
             *
             * @return mDimensionX X dimension
             */
            double DimensionX () { return mDimensionX; }

            /**
             * @brief Is this synapse set modifiable?
             *
             * @param None
             *
             * @return true or false
             */
            inline bool IsPlastic() { return mIsPlastic; }

            /**
             * @brief The identifier
             *
             * @param none
             *
             * @return mIdentifier
             */
            inline std::string Identifier () { return mIdentifier; }

            /**
             * @brief Return the weight matrix for this synapse set
             * 
             * @param None
             *
             * @return mWeightMatrix The weight matrix of this synapse set
             */
            inline Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> WeightMatrix () { return mWeightMatrix; }
            /* ====================  MUTATORS      ======================================= */

            inline void SetBounding (double bounding) { mWeightIsBound = bounding; }
            inline void SetDecay (double decay) {mDecayRate = decay; }


            /* ====================  OPERATORS     ======================================= */
            /*  pure virtual methods that must be overridden by extending
             *  classes */
            /**
             * @brief Update synaptic weights
             *
             * @param preSynapticFiringRate The presynaptic firing rate
             *
             * @param postSynapticFiringRate The postsynaptic firing rate
             *
             * @return void
             */
            void UpdateWeight(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> preSynapticFiringRate, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> postSynapticFiringRate)
            {
                if (mIsPlastic == true)
                {
                    mDeltaW.resize(mDimensionX, mDimensionY);
                    mDeltaW = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);
/*                     double firing_rate_minimum = 0.5 * (postSynapticFiringRate.maxCoeff () + postSynapticFiringRate.minCoeff ());
 */
/*                     mDeltaW = (mLearningRate * (1.0 - (mWeightIsBound * ((mWeightMatrix - mWeightMatrix.minCoeff ())/mWeightMatrix.maxCoeff ()).array ())) * ((preSynapticFiringRate * (postSynapticFiringRate.array () - firing_rate_minimum).matrix()) - (mDecayRate * mWeightMatrix.array ()).matrix () ).array ()).matrix ();
 */

                    mDecayRate = 0;
/*                     mDecayRate = (0.025 * preSynapticFiringRate.maxCoeff () * postSynapticFiringRate.maxCoeff ());
 */
                    ROS_DEBUG ("Decay rate set to: %f",mDecayRate);
                    mDeltaW = (mLearningRate * ((preSynapticFiringRate * postSynapticFiringRate).array () - mDecayRate)). matrix ();
                    mWeightMatrix += mDeltaW;
                    ROS_DEBUG("%s: Synaptic weight updated by [%f, %f]", mIdentifier.c_str (), mDeltaW.maxCoeff (), mDeltaW.minCoeff ());
                    mDeltaW = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(mDimensionX, mDimensionY);
/*                     mDeltaW = mWeightMatrix.array () / mWeightMatrix.norm ();
 */
                }
                else 
                {
                    ROS_FATAL("%s: Unable to modify stiff synapses!", mIdentifier.c_str ());
                }
            }

            /**
             * @brief Normalize weights 
             *
             * Divide by norm to normalize. 
             * Using this implies our learning rule is not local any more,
             * since it depends on all neurons in the set for normalization
             *
             * @param None
             *
             * @return void
             */
            void Normalize (){
                /*  Normalize each row individually */
                for (int i = 0; i < mWeightMatrix.rows (); i++)
                {
                    mDeltaW.row (i) = mWeightMatrix.row (i)/mWeightMatrix.row (i).norm ();
                }
                mWeightMatrix = mDeltaW;
                ROS_DEBUG("%s: Synaptic weight normalized to [%f,%f]", mIdentifier.c_str (), mWeightMatrix.maxCoeff (), mWeightMatrix.minCoeff ());


            }

            /**
             * @brief add to the current weight matrix
             *
             * Comes in handy during calibration, and when hebbian learning occurs
             *
             * @param addition Matrix term to be added to current synaptic
             * weight
             *
             * @return void
             */
            inline void AddToWeight(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> addition)
            {
                if (mIsPlastic == true)
                {
                    ROS_DEBUG_STREAM("Size of additive matrix is: " << addition.rows () << " x " << addition.cols ());
                    ROS_DEBUG_STREAM("Size of left matrix is: " << mWeightMatrix.rows () << " x " << mWeightMatrix.cols ());
                    mWeightMatrix += addition;
                    ROS_DEBUG("%s: Synaptic weight updated.", mIdentifier.c_str ());
                }
                else 
                {
                    ROS_FATAL("%s: Unable to modify stiff synapses!", mIdentifier.c_str ());
                }
            }

            /**
             * @returns maximum value of synaptic weight
             *
             * @param None
             *
             * @return maximum value of synaptic weight
             */
            double Max ()
            {
                return mWeightMatrix.maxCoeff ();
            }

            /**
             * @returns minimum value of synaptic weight
             *
             * @param None
             *
             * @return minimum value of synaptic weight
             */
            double Min ()
            {
                return mWeightMatrix.minCoeff ();
            }


            /**
             * @brief Enable plasticity of synaptic weights
             *
             * @param None
             *
             * @return void
             */
            inline void SetPlastic ()
            {
                if (mIsPlastic == true) {
                    ROS_WARN ("%s: Synapse is already modifiable!", mIdentifier.c_str ());
                }
                else {
                    mIsPlastic = true;
                    ROS_INFO ("%s: Synapse set as modifiable!", mIdentifier.c_str ());
                }
            }

            /**
             * @brief Disable plasticity of synaptic weights
             *
             * @param None
             *
             * @return void
             */
            inline void SetStiff ()
            {
                if (mIsPlastic == false) {
                    ROS_WARN ("%s: Synapse is already stiff!", mIdentifier.c_str ());
                }
                else {
                    mIsPlastic = false;
                    ROS_INFO ("%s: Synapse set as stiff!", mIdentifier.c_str ());
                }
            }

            /**
             * @brief Set the identifier for this synapse set
             *
             * @param identifier
             *
             * @return void */
            inline void SetIdentifier (std::string identifier)
            {
                if(mIdentifier != std::string ("SynapseSet")) {
                    ROS_WARN ("%s: Identifier already set. Unable to comply!", mIdentifier.c_str ());
                }
                else {
                    mIdentifier = identifier;
                    ROS_INFO("%s: New identifier set.", mIdentifier.c_str ());
                }
            }

            /**
             * @brief Set the dimensions of the synapse set
             *
             * @param dimensionX X dimension
             * @param dimensionY Y dimension
             *
             * @return void
             */
            void SetDimension (double dimensionX, double dimensionY)
            {
                mDimensionX = dimensionX;
                mDimensionY = dimensionY;

                ROS_INFO("%s: Dimensions set to %f x %f", mIdentifier.c_str (), mDimensionX, mDimensionY);
                return ;
            }

            /**
             * @brief Initialize matrices
             *
             * @param None
             *
             * @return void
             */
            void Init ( )
            {
                mWeightMatrix.resize(mDimensionX, mDimensionY);
                mWeightMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mDimensionX, mDimensionY);
            }

            /**
             * @brief rescale the synaptic weights to [0,1]
             * 
             * Divides by the largest value, and multiplies by provided value.
             * More [0, multiplier] really.
             *
             * @param multiplier value to rescale to
             *
             * @return void
             */
            void Rescale (double multiplier) 
            {
                double temp = mWeightMatrix.maxCoeff ();
                mWeightMatrix /= temp;
                mWeightMatrix *= multiplier;

/*                 ROS_DEBUG("%s: rescaled to [%f,%f]", mIdentifier.c_str (), mWeightMatrix.maxCoeff (), mWeightMatrix.minCoeff ());
 */
            }

            /**
             * @brief Print to a file
             *
             * I need more ways of debugging the system really.
             *
             * @param fileName File to print to
             *
             * @return void
             */
            void PrintToFile( std::string fileName)
            {
                std::ofstream my_file;
                my_file.open(fileName.c_str (), std::ios_base::binary);

                my_file << mWeightMatrix;
                my_file.close ();
            }

            void LearningRate (double learningRate)
            {
                mLearningRate = learningRate;
                ROS_DEBUG("%s: Learning rate set: %f", mIdentifier.c_str (), mLearningRate);
            }

        protected:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mWeightMatrix;
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mDeltaW;
            double mLearningRate;
            double mDecayRate;
            double mWeightIsBound;
            bool mIsPlastic;                    /**< Is this synapse set plastic or fixed during the run? */
            std::string mIdentifier;            /**< A name for the synapse set */
            double mDimensionX;            /**< X dimension */
            double mDimensionY;            /**< Y dimension */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class SynapseSet  ----- */

}
#endif   /* ----- #ifndef SynapseSet_INC  ----- */
