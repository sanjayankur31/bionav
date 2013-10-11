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
#include "ros/ros.h"

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
    template<class WeightMatrixType, class PreSynapticFiringRateType, class PostSynapticFiringRateType>
    class SynapseSet
    {
        public:
            /* ====================  LIFECYCLE     ======================================= */
            SynapseSet ()                             /**< constructor */
            {
                /*  Default values for constants */
                mIsPlastic = false;
                mEta = 0;
                mIdentifier = std::string("SynapseSet");
            }

            ~SynapseSet () {}                             /**< destructor */

            /* ====================  ACCESSORS     ======================================= */
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
            inline WeightMatrixType WeightMatrix () { return mWeightMatrix; }
            /* ====================  MUTATORS      ======================================= */


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
             * @return None
             */
            virtual void UpdateWeight(PreSynapticFiringRateType preSynapticFiringRate, PostSynapticFiringRateType postSynapticFiringRate) =0;


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
                    ROS_DEBUG ("%s: Synapse is already modifiable!", mIdentifier.c_str ());
                }
                else {
                    mIsPlastic = true;
                    ROS_DEBUG ("%s: Synapse set as modifiable!", mIdentifier.c_str ());
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
                    ROS_DEBUG ("%s: Synapse is already stiff!", mIdentifier.c_str ());
                }
                else {
                    mIsPlastic = false;
                    ROS_DEBUG ("%s: Synapse set as stiff!", mIdentifier.c_str ());
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
                    ROS_DEBUG ("%s: Identifier already set. Unable to comply!", mIdentifier.c_str ());
                }
                else {
                    mIdentifier = identifier;
                    ROS_DEBUG("%s: New identifier set.", mIdentifier.c_str ());
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

                ROS_DEBUG("%s: Dimensions set to %f x %f", mIdentifier.c_str (), mDimensionX, mDimensionY);
                return ;
            }
        protected:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */
            WeightMatrixType mWeightMatrix;
            WeightMatrixType mDeltaW;
            double mEta;                   /**< @f$ \eta @f$ */
            double mLearningRate;
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
