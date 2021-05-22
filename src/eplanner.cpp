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

/* Author: Constantinos Chamzas  */

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <robowflex_library/io/handler.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>

#include <pyre/eplanner.h>
#include <pyre/pyre.h>

using namespace pyre;
EPlanner::EPlanner(const robowflex::RobotPtr &robot, const std::string &name, const PyrePtr &exp,
                   const robowflex::ScenePtr &escene)
  : robowflex::OMPL::OMPLInterfacePlanner(robot, name), exp_(exp), escene_(escene)
{
}

ompl::geometric::SimpleSetupPtr EPlanner::getSS(const robowflex::SceneConstPtr &scene,
                                                const planning_interface::MotionPlanRequest &request)
{
    auto context = getPlanningContext(scene, request);
    return context->getOMPLSimpleSetup();
}

planning_interface::MotionPlanResponse EPlanner::plan(const robowflex::SceneConstPtr &scene,
                                                      const planning_interface::MotionPlanRequest &request)
{
    planning_interface::MotionPlanResponse response;
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    econtext_ = getPlanningContext(scene, request);
    ess_ = econtext_->getOMPLSimpleSetup();

    if (exp_)
    {
        auto succ = exp_->setBiasedSampler(ess_, escene_, request);
        if (succ)
            ROS_WARN("Planning Using Experience based Planning");
        else
            ROS_WARN("Failed to initialize Custom Sampler, defaulting to Uniform \n Did you set escene?");
    }

    else
        ROS_WARN("Planning using Uniform Sampling!");

    econtext_->setInterpolation(interpolate_);
    econtext_->simplifySolutions(simplify_);
    econtext_->solve(response);
    return response;
}

ompl::geometric::SimpleSetupPtr EPlanner::getSS() const
{
    return ess_;
}
void EPlanner::setSimplifySolutions(const bool &flag)
{
    simplify_ = flag;
}

void EPlanner::setInterpolation(const bool &flag)
{
    interpolate_ = flag;
}
void EPlanner::clearExperience()
{
    if (exp_)
        exp_ = nullptr;
    if (ess_)
    {
        ess_->clear();
        ess_->getStateSpace()->clearStateSamplerAllocator();
    }
}
const Pyre::Stats EPlanner::getStats() const
{
    if (exp_)  // returning the computed stats
        return exp_->getStats();
    else  // returning empty stats
        return Pyre::Stats();
}

// void EPlanner::preRun(const robowflex::SceneConstPtr &scene,
//                      const planning_interface::MotionPlanRequest &request)
//{
//    if (not tfile_.empty())
//    {
//        econtext_ = getPlanningContext(scene, request);
//        ess_ = econtext_->getOMPLSimpleSetup();
//
//        ROS_WARN("Setting Up Thunder Framework");
//        static_cast<ompl::tools::Thunder *>(ess_.get())->setFilePath(tfile_);
//        static_cast<ompl::tools::Thunder *>(ess_.get())->setup();
//    }
//}
