/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Constantinos Chamzas*/

#ifndef PYRE_EPLANNER_
#define PYRE_EPLANNER_

#include <pyre/class_forward.h>
#include <robowflex_ompl/ompl_interface.h>
#include <pyre/pyre.h>

namespace pyre
{
    CLASS_FORWARD(EPlanner);

    /** \class robowflex::OMPL::EplannerPtr
        \brief A shared pointer wrapper for robowflex::OMPL::Eplanner. */

    /** \class robowflex::OMPL::EplannerConstPtr
        \brief A const shared pointer wrapper for robowflex::OMPL::Eplanner. */

    /** \brief A planner that directly uses \a MoveIt!'s OMPL planning interface.
     */
    class EPlanner : public robowflex::OMPL::OMPLInterfacePlanner
    {
    public:
        /** \brief Constructor.
         *  \param[in] robot The robot to plan for.
         *  \param[in] name Optional namespace for planner.
         */
        EPlanner(const robowflex::RobotPtr &robot, const std::string &name, const PyrePtr &exp = nullptr,
                 const robowflex::ScenePtr &escene = nullptr);

        // non-copyable
        EPlanner(EPlanner const &) = delete;
        void operator=(EPlanner const &) = delete;

        /** \brief Plan a motion given a \a request and a \a scene.
         *  Uses the planning pipeline's generatePlan() method, which goes through planning adapters.
         *  \param[in] scene A planning scene for the same \a robot_ to compute the plan in.
         *  \param[in] request The motion planning request to solve.
         *  \return The motion planning response generated by the planner.
         */
        planning_interface::MotionPlanResponse
        plan(const robowflex::SceneConstPtr &scene,
             const planning_interface::MotionPlanRequest &request) override;

        /** \brief Gets the last simple Setup .*/
        ompl::geometric::SimpleSetupPtr getSS() const;

        /** \brief Gets a new simple Setup from a new planning context.*/
        ompl::geometric::SimpleSetupPtr getSS(const robowflex::SceneConstPtr &scene,
                                              const planning_interface::MotionPlanRequest &request);

        std::map<std::string, Planner::ProgressProperty>
        getProgressProperties(const robowflex::SceneConstPtr & /*scene*/,
                              const planning_interface::MotionPlanRequest & /*request*/) const override

        {
            return {};
        };

        /** \brief Clears the experience database */;
        void clearExperience();

        /** \brief Gets the statistics */;
        const Pyre::Stats getStats() const;

    private:
        PyrePtr exp_;
        robowflex::ScenePtr escene_;
        ompl::geometric::SimpleSetupPtr ess_;
        ompl_interface::ModelBasedPlanningContextPtr econtext_;  ///< Last context.
    };
}  // namespace pyre

#endif
