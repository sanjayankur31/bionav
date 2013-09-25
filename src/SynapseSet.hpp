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
            SynapseSet ();                             /**< constructor */
            ~SynapseSet ();                             /**< destructor */

            /* ====================  ACCESSORS     ======================================= */
            /**
             * @brief Does this set carry a trace?
             *
             * @param None
             *
             * @return true or false
             */
            inline bool HasTrace() { return mHasTrace; }

            /**
             * @brief Is this synapse set modifiable?
             *
             * @param None
             *
             * @return true or false
             */
            inline bool IsPlastic() { return mIsPlastic; }

            /* ====================  MUTATORS      ======================================= */


            /* ====================  OPERATORS     ======================================= */
            /*  pure virtual methods that must be overridden by extending
             *  classes */
            /**
             * @brief Update synaptic weights
             *
             * @param PreSynapticFiringRateType preSynapticFiringRate The
             * presynaptic firing rate
             *
             * @param PostSynapticFiringRateType postSynapticFiringRate The
             * postsynaptic firing rate
             *
             * @return None
             */
            void UpdateWeight(PreSynapticFiringRateType preSynapticFiringRate, PostSynapticFiringRateType postSynapticFiringRate) =0;

            /**
             * @brief Update the synaptic trace, if enabled
             *
             * @param None
             *
             * @return None
             * */
            void UpdateWeightTrace() =0;

            /**
             * @brief Enable trace matrix
             *
             * @param None
             *
             * @return void
             */
            inline void EnableTrace ();

            /**
             * @brief Disable trace matrix
             *
             * @param None
             *
             * @return void
             */
            inline void DisableTrace ();

            /**
             * @brief Enable plasticity of synaptic weights
             *
             * @param None
             *
             * @return void
             */
            inline void SetPlastic ();

            /**
             * @brief Disable plasticity of synaptic weights
             *
             * @param None
             *
             * @return void
             */
            inline void SetStiff ();

            /**
             * @brief Set the identifier for this synapse set
             *
             * @param std::string identifier
             *
             * @return void */
            inline void SetIdentifier (std::string identifier);

        protected:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */
            WeightMatrixType mWeightMatrix;
            WeightMatrixType mWeightTraceMatrix;
            bool mHasTrace;                     /**< Does this synapse set need a trace matrix?  */
            long double mEta;                   /**< @f$ \eta @f$ */
            bool mIsPlastic;                    /**< Is this synapse set plastic or fixed during the run? */
            std::string mIdentifier;            /**< A name for the synapse set */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class SynapseSet  ----- */

}
#endif   /* ----- #ifndef SynapseSet_INC  ----- */
