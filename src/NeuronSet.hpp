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
     * @note There isn't a point of implementing a pure abstract
     * UpdateActivation method since each set of cells will have a
     * different set of input synapses. I'd rather not use variadic
     * arguments and just implement it when needed for various cells.
     * In my case, it's only needed for the HDCell ensemble. The rest
     * will just fire as feature detectors.
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
            /**
             * @brief Does this set carry a trace?
             *
             * @param None
             *
             * @return true or false
             */
            inline bool HasTrace() { return mHasTrace; }

            /**
             * @brief The identifier
             *
             * @param none
             *
             * @return mIdentifier
             */
            inline std::string Identifier () { return mIdentifier; }

            /**
             * @brief return Y dimension
             *
             * @param None
             *
             * @return mDimensionY Y dimension
             */
            long double DimensionY () { return mDimensionY; }

            /**
             * @brief return X dimension
             *
             * @param None
             *
             * @return mDimensionX X dimension
             */
            long double DimensionX () { return mDimensionX; }


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
            inline void SetDimension (long double dimensionX, long double dimensionY);

            /**
             * @brief Return firing rates
             *
             * @param None
             *
             * @return FiringRateType firing rate matrix
            */
            inline FiringRateType FiringRate () { return mFiringRate ;}

            /**
             * @brief Return firing rate trace matrix
             *
             * @param None
             *
             * @return FiringRateType the trace firing rate
             * */
            FiringRateType FiringRateTrace() { return mFiringRateTrace; }

            /**
             * @brief Calculate new firing rate matrix
             *
             * @param Nonw
             *
             * @return None
             */
            void UpdateFiringRate () =0;

            /**
             * @brief Calculate new trace
             *
             * @param None
             *
             * @return None
             */
            void UpdateFiringRateTrace () =0;

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

        protected:
            /* ====================  METHODS       ======================================= */


            /* ====================  DATA MEMBERS  ======================================= */
            std::string mIdentifier;            /**< Identifier of this neuron set */
            FiringRateType mActivation;         /**< Activation values of neurons */
            FiringRateType mFiringRate;         /**< Firing rates of neurons */
            FiringRateType mFiringRateTrace;    /**< Trace firing rates of neurons */
            long double mDimensionX;            /**< X dimension */
            long double mDimensionY;            /**< Y dimension */
            bool mHasTrace;                     /**< Does this provide a trace matrix?  */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class NeuronSet  ----- */

}
#endif   /* ----- #ifndef NeuronSet_INC  ----- */
