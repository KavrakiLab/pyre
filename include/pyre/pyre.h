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

#ifndef PYRE_PYRE
#define PYRE_PYRE

#include <string>

#include <moveit/planning_pipeline/planning_pipeline.h>

#include <ompl/geometric/SimpleSetup.h>

#include <pyre/class_forward.h>
#include <pyre/database.h>

/** \cond IGNORE */
namespace robowflex
{
    CLASS_FORWARD(Robot);
    CLASS_FORWARD(Scene);
    CLASS_FORWARD(MotionRequestBuilder);
    CLASS_FORWARD(Trajectory);
}  // namespace robowflex
/** \endcond */

namespace pyre
{
    /** \cond IGNORE */
    CLASS_FORWARD(Pyre);
    /** \endcond */

    class Pyre
    {
    public:
        struct Stats
        {
            unsigned long num_local_prims;
            unsigned long num_local_samplers;
            double ret_time;
            Stats() : num_local_prims(0), num_local_samplers(0), ret_time(0)
            {
            }
            Stats(unsigned long nlp, unsigned long nls, double rt)
              : num_local_prims(nlp), num_local_samplers(nls), ret_time(0)
            {
            }
        };

        /** \brief Constructor */
        Pyre();

        /** \brief Process the given experience to pairs of local primitives /local samplers (Entries) */
        virtual std::vector<Entry *>
        processExperience(const robowflex::TrajectoryConstPtr &traj,  //
                          const robowflex::SceneConstPtr &scene,      //
                          const robowflex::MotionRequestBuilderConstPtr &request) = 0;

        /** \brief Stores the current state of experiences. */
        virtual void storeExperiences(const std::string &db_file) = 0;

        /** \brief Loads a set of experience.s */
        virtual void loadExperiences(const std::string &db_file) = 0;

        /** \brief Sets a biased sampler on the given ompl simple setup. */
        virtual bool setBiasedSampler(const ompl::geometric::SimpleSetupPtr &ss,  //
                                      const robowflex::SceneConstPtr &scene,      //
                                      const planning_interface::MotionPlanRequest &request) = 0;

        /** \brief Returns metrics for the experience framework e.g, number of local primitives, local
         * samplers and the retrieval time */
        const Pyre::Stats getStats() const
        {
            return statistics_;
        }

    protected:
        Stats statistics_;
    };

}  // namespace pyre
#endif
