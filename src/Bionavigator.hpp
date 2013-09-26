/*
 * =====================================================================================
 *
 *       Filename:  Bionavigator.hpp
 *
 *    Description:  Header for main Bionavigator class
 *
 *        Version:  1.0
 *        Created:  19/09/13 11:49:16
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ankur Sinha (FranciscoD), ankursinha@fedoraproject.org
 *   Organization:  University of Technology, Sydney
 *
 * =====================================================================================
 */


#ifndef  Bionavigator_INC
#define  Bionavigator_INC

#include "HDCells.hpp"
#include "RotationCellClockwise.hpp"
#include "RotationCellCounterClockwise.hpp"
#include "VisionCells.hpp"
#include "HD_VisionSynapseSet.hpp"
#include "HD_RotationSynapseSet.hpp"


/*
 * =====================================================================================
 *        Class:  Bionavigator
 *  Description:  The main bionavigator class
 * =====================================================================================
 */
/**
 * @class Bionavigator
 *
 * @brief The main class that runs my system
 *
 * This class can be thought of as the brain, where the different components
 * link together via their synapses and carry out the operation.
 * 
 */
class Bionavigator
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        Bionavigator ();                             /**< constructor      */
        Bionavigator ( const Bionavigator &other );   /**< copy constructor */
        ~Bionavigator ();                            /**< destructor       */

        /* ====================  ACCESSORS     ======================================= */

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */

        Bionavigator& operator = ( const Bionavigator &other ); /* assignment operator */

        /**
         * @brief Initialize the system
         *
         * Check for presence of required nodes and inputs, set up synapses and
         * neuron sets
         *
         * @param None
         *
         * @returns None
         */
        void Init ();

        /**
         * @brief Calibrate the synaptic weights of all synapses before
         * functioning of system
         * 
         * @param None
         *
         * @returns None
         */
        void Calibrate ();

        /**
         * @brief Process inputs and update current states of synapses and
         * neuron sets
         *
         * @param None
         *
         * @return None
         */
        void UpdateState ();

        /**
         * @brief Publish current direction
         *
         * @todo I'll need to add more methods as I look to implement land mark
         * navigation
         *
         * @param None
         *
         * @return None
         */
        void PublishDirection ();



    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        HDCells* mpHDCells;                       /**< Head cell set */
        RotationCellCounterClockwise* mpRotationCellCounterClockwise; /**< Counter clockwise rotation cell */
        RotationCellClockwise* mpRotationCellClockwise; /**< Clockwise rotation cell */
        VisionCells* mpVisionCells;               /**< Vision cell set */

        HDSynapseSet* mpHDSynapseSet;             /**< HD synapse set */
        HD_VisionSynapseSet* mpHD_VisionSynapseSet; /**< HD - Vision synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellClockwiseSynapseSet; /**< HD - Clockise rotation cell synapse set */
        HD_RotationSynapseSet* mpHD_RotationCellCounterClockwiseSynapseSet; /**< HD - counter clockwise rotation cell synapse set */

}; /* -----  end of class Bionavigator  ----- */


#endif   /* ----- #ifndef Bionavigator_INC  ----- */

