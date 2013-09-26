/*
 * =====================================================================================
 *
 *       Filename:  NeuronSet.hpp
 *
 *    Description:  Header file for NeuronSet class
 *
 *        Version:  1.0
 *        Created:  09/08/2013 02:09:38 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */


#ifndef  NeuronSet_INC
#define  NeuronSet_INC

namespace Bionav {

    /*
     * =====================================================================================
     *        Class:  NeuronSet
     *  Description:  A set of neurons
     * =====================================================================================
     */
    /**
     * @class NeuronSet
     *
     * @brief A set of neurons
     *
     * Each set of neurons has a certain function, depending on what stimulus
     * inputs they process
     *
     * Each set is connected to other sets via plastic or stiff synapses
     * 
     */
    template <class FiringRateType>
    class NeuronSet
    {
        public:
            /* ====================  LIFECYCLE     ======================================= */
            NeuronSet ();                             /**< constructor */
            ~NeuronSet ();                             /**< destructor */

            /* ====================  ACCESSORS     ======================================= */

            /* ====================  MUTATORS      ======================================= */

            /* ====================  OPERATORS     ======================================= */

            /**
             * @brief Set the identifier for this synapse set
             *
             * @param identifier
             *
             * @return void */
            inline void SetIdentifier (std::string identifier);

            /**
             * @brief Set the dimensions of the neuron set
             *
             * @param dimensionX X dimension
             * @param dimensionY Y dimension
             *
             * @return void
             */
            void SetDimension (long double dimensionX, long double dimensionY)
            {
                mDimensionX = dimensionX;
                mDimensionY = dimensionY;
            };

            /**
             * @brief Update the activations
             *
             * @param None
             *
             * @return void 
             */
            void UpdateActivations () =0;

            /**
             * @brief Calculate firing rates
             *
             * @param None
             *
             * @return void
            */
            void FiringRate () =0;

        protected:
            /* ====================  METHODS       ======================================= */


            /* ====================  DATA MEMBERS  ======================================= */
            std::string mIdentifier;            /**< Identifier of this neuron set */
            FiringRateType mActivation;         /**< Activation values of neurons */
            FiringRateType mFiringRate;         /**< Firing rates of neurons */
            long double mDimensionX;            /**< X dimension */
            long double mDimensionY;            /**< Y dimension */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class NeuronSet  ----- */

}
#endif   /* ----- #ifndef NeuronSet_INC  ----- */
