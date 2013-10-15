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

#include <iostream>
#include "ros/ros.h"
#include <Eigen/Dense>

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
     * All implementations also go here. Read here why:
     * http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
     *
     */
    class NeuronSet
    {
        public:
            /* ====================  LIFECYCLE     ======================================= */
            NeuronSet ()
            {
                /*  A default place holder name that must be changed
                 *
                 *  I set it here for two reasons:
                 *
                 *  1. It'll tell me if I haven't changed it
                 *  2. It'll be used to ensure that the name is only set once
                 *
                */
                mIdentifier = std::string("NeuronSet");

            }

            ~NeuronSet () { ;}                             /**< destructor */

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
            double DimensionY () { return mDimensionY; }

            /**
             * @brief return X dimension
             *
             * @param None
             *
             * @return mDimensionX X dimension
             */
            double DimensionX () { return mDimensionX; }


            /* ====================  MUTATORS      ======================================= */

            /* ====================  OPERATORS     ======================================= */

            /**
             * @brief Set the identifier for this synapse set
             *
             * @param identifier
             *
             * @return void */
            inline void SetIdentifier (std::string identifier)
            {
                if(mIdentifier != std::string ("NeuronSet")) {
                    ROS_DEBUG ("%s: Identifier already set. Unable to comply!", mIdentifier.c_str ());
                }
                else {
                    mIdentifier = identifier;
                    ROS_DEBUG("%s: New identifier set.", mIdentifier.c_str ());
                }
                return ;
            }

            /**
             * @brief Set the dimensions of the neuron set
             *
             * @param dimensionX X dimension
             * @param dimensionY Y dimension
             *
             * @return void
             */
            inline void SetDimension (double dimensionX, double dimensionY)
            {
                mDimensionX = dimensionX;
                mDimensionY = dimensionY;

                ROS_DEBUG("%s: Dimensions set to %f x %f", mIdentifier.c_str (), mDimensionX, mDimensionY);
                return ;
            }


            /**
             * @brief Return firing rates
             *
             * @param None
             *
             * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> firing rate matrix
            */
            inline Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FiringRate () { return mFiringRate ;}

            /**
             * @brief Return firing rate trace matrix
             *
             * @param None
             *
             * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> the trace firing rate
             * */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> FiringRateTrace() { return mFiringRateTrace; }

            /**
             * @brief Calculate new firing rate matrix
             *
             * @param Nonw
             *
             * @return None
             */
            virtual void UpdateFiringRate () =0;

            /**
             * @brief Calculate new trace
             *
             * @param None
             *
             * @return None
             */
            virtual void UpdateFiringRateTrace () =0;

            /**
             * @brief Enable trace matrix
             *
             * @param None
             *
             * @return void
             */
            inline void EnableTrace ()
            {
                if (mHasTrace == true)
                {
                    ROS_DEBUG("%s: Trace already enabled.", mIdentifier.c_str ());
                }
                else 
                {
                    mHasTrace = true;
                    ROS_DEBUG("%s: Trace enabled.",mIdentifier.c_str ());
                }
                return;
            }

            /**
             * @brief Disable trace matrix
             *
             * @param None
             *
             * @return void
             */
            inline void DisableTrace ()
            {
                if(mHasTrace == false)
                {
                    ROS_DEBUG("%s: Trace already disabled.", mIdentifier.c_str ());
                }
                else
                {
                    mHasTrace = false;
                    ROS_DEBUG("%s: Trace disabled.",mIdentifier.c_str ());
                }
                return ;
            }

            /**
             * @brief Initialize the matrices
             *
             * @param None
             *
             * @return None
             */
            void Init ( )
            {
                mFiringRate.resize(mDimensionX,mDimensionY);
                mFiringRateTrace.resize(mDimensionX,mDimensionY);
                
                mFiringRateTrace = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mDimensionX, mDimensionY);
                mFiringRate = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero (mDimensionX, mDimensionY);
            }

            /**
             * @brief Enable maximum firing for all cells
             *
             * Needed in training
             *
             * @param None
             *
             * @return None
             */
            void EnableForceFire ( )
            {
                Init ();
                mFiringRate.array() += 1;
                mFiringRateTrace.array() += 1;
                mForceFiring = true;
            }

            /**
             * @brief Disable all firing. Basically, re-init the cells
             *
             * @param None
             *
             * @return None
             */
            void DisableForceFire ( )
            {
                Init ();
                mForceFiring = false;
            }
        protected:
            /* ====================  METHODS       ======================================= */


            /* ====================  DATA MEMBERS  ======================================= */
            std::string mIdentifier;            /**< Identifier of this neuron set */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mActivation;         /**< Activation values of neurons */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mFiringRate;         /**< Firing rates of neurons */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mFiringRateTrace;    /**< Trace firing rates of neurons */
            double mDimensionX;            /**< X dimension */
            double mDimensionY;            /**< Y dimension */
            bool mHasTrace;                     /**< Does this provide a trace matrix?  */
            bool mForceFiring;

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class NeuronSet  ----- */

}
#endif   /* ----- #ifndef NeuronSet_INC  ----- */
