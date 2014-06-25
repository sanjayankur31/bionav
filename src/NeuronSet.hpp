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
#include <fstream>
#include <iomanip>
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
                mEta = 0.1;

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

            /**
             * @brief Print activation matrix to a file
             *
             * I need more ways of debugging the system really.
             *
             * @param fileName File to print to
             *
             * @return void
             */
            void PrintActivationToFile( std::string fileName)
            {
                std::ofstream my_file;
                my_file.open(fileName.c_str (), std::ios_base::binary);

                my_file << (1.0 * mActivation);
                my_file.close ();
            }

            /**
             * @brief Print firing matrix to a file
             *
             * I need more ways of debugging the system really.
             *
             * @param fileName File to print to
             *
             * @return void
             */
            void PrintFiringRateToFile( std::string fileName)
            {
                std::ofstream my_file;
                my_file.open(fileName.c_str (), std::ios_base::binary);

                my_file << (1.0 * mFiringRate);
                my_file.close ();
            }

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
                    ROS_WARN ("%s: Identifier already set. Unable to comply!", mIdentifier.c_str ());
                }
                else {
                    mIdentifier = identifier;
                    ROS_INFO("%s: New identifier set.", mIdentifier.c_str ());
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

                ROS_INFO("%s: Dimensions set to %f x %f", mIdentifier.c_str (), mDimensionX, mDimensionY);
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
             * @brief Return activation matrix
             *
             * @param None
             *
             * @return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> the activation matrix
             * */
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Activation () { return mActivation; }

            /**
             * @brief Calculate new firing rate matrix
             *
             * @param Nonw
             *
             * @return void
             */
            virtual void UpdateFiringRate () =0;

            /**
             * @brief Calculate new trace
             *
             * @param None
             *
             * @return void
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
                    ROS_WARN("%s: Trace already enabled.", mIdentifier.c_str ());
                }
                else 
                {
                    mHasTrace = true;
                    ROS_INFO("%s: Trace enabled.",mIdentifier.c_str ());
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
                    ROS_WARN("%s: Trace already disabled.", mIdentifier.c_str ());
                }
                else
                {
                    mHasTrace = false;
                    ROS_INFO("%s: Trace disabled.",mIdentifier.c_str ());
                }
                return ;
            }

            /**
             * @brief Initialize the matrices
             *
             * @param None
             *
             * @return void
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
             * @return void
             */
            void EnableForceFire ( )
            {
                if (mForceFiring == false)
                {
                    Init ();
                    mFiringRate.array() += 1;
                    mFiringRateTrace.array() += 1;
                    mForceFiring = true;
                    ROS_DEBUG("%s: Force firing enabled", mIdentifier.c_str ());
                }
                else 
                {
                    ROS_WARN("%s: Force firing already enabled", mIdentifier.c_str ());
                }
            }
            /**
             * @brief Enable maximum firing for all cells
             *
             * Needed in training
             *
             * @param value The value to set the firing rate to
             *
             * @return void
             */
            void EnableForceFire (double value )
            {
                if (mForceFiring == false)
                {
                    Init ();
                    mFiringRate.array() += value;
                    mFiringRateTrace.array() += value;
                    mForceFiring = true;
                    ROS_INFO("%s: Force firing enabled with magnitude %0.8f", mIdentifier.c_str (), value);
                }
                else 
                {
                    ROS_WARN("%s: Force firing already enabled", mIdentifier.c_str ());
                }
            }

            /**
             * @brief Disable all firing. Basically, re-init the cells
             *
             * @param None
             *
             * @return void
             */
            void DisableForceFire ( )
            {
                if (mForceFiring == true)
                {
                    Init ();
                    mForceFiring = false;
                    ROS_INFO("%s: Force firing disabled", mIdentifier.c_str ());
                }
                else 
                {
                    Init ();
                    ROS_WARN("%s: Force firing already disabled", mIdentifier.c_str ());
                }
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
            double mEta;                   /**< @f$ \eta @f$ */

        private:
            /* ====================  METHODS       ======================================= */

            /* ====================  DATA MEMBERS  ======================================= */

    }; /* -----  end of class NeuronSet  ----- */

}
#endif   /* ----- #ifndef NeuronSet_INC  ----- */
