/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Rice University
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

/* Author: Constantinos Chamzas */

#include <pyre/biased_sampler.h>
#include <pyre/math.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

using namespace pyre;

BiasedStateSampler::BiasedStateSampler(const ob::StateSpace *space)
  : StateSampler(space), sampler_(space->allocDefaultStateSampler()), lambda_(0)
{
    ROS_INFO("Custom Sampler: Uniform sampling will be used");
}

BiasedStateSampler::BiasedStateSampler(const ob::StateSpace *space, std::vector<Eigen::VectorXd> &means,
                                       const double lambda, const double std)
  : StateSampler(space), sampler_(space->allocDefaultStateSampler()), GMM(), lambda_(lambda)
{
    set_.clear();

    for (size_t i = 0; i < means.size(); i++)
    {
        std::stringstream ss;
        ss << means[i];
        // only add unique samplers.
        if (set_.insert(ss.str()).second)
        {
            auto mvn = new MVN(means[i], std);
            GMM.add(*mvn, 1);
        }
    }
    if (means.size() < 1)
    {
        ROS_WARN("Custom Sampler is Empty!, defaulting to Uniform!");
        lambda_ = -1;
    }

    ROS_INFO("Custom Sampler: lambda:%f values:%lu /%lu std:%f weights:1 ", lambda_, GMM.size(), means.size(),
             std);
};

BiasedStateSampler::BiasedStateSampler(const ob::StateSpace *space, std::vector<Eigen::VectorXd> &means,
                                       std::vector<Eigen::MatrixXd> covars, const double lambda)
  : StateSampler(space), sampler_(space->allocDefaultStateSampler()), GMM(), lambda_(lambda)
{
    for (size_t i = 0; i < means.size(); i++)
        GMM.add(MVN(means[i], covars[i]), 1);

    ROS_INFO("Custom Sampler: lambda:%f values:%lu variance learned, weights:1", lambda_, means.size());
}

void BiasedStateSampler::sampleUniform(ob::State *state)
{
    if (lambda_ > rng_.uniformReal(0, 1))
    {
        // Sampling a normal dist from the mixture
        auto normal = GMM.sample(rng_.uniformReal(0, 1));
        auto samp = normal.sample();
        for (unsigned int i = 0; i < samp.size(); i++)
            state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[i] = samp[i];
        // enforcing the bounds
        space_->enforceBounds(state);
    }
    else
        sampler_->sampleUniform(state);
}

void BiasedStateSampler::sampleUniformNear(ob::State *state, const ob::State *near, const double distance)
{
    sampler_->sampleUniformNear(state, near, distance);
}

void BiasedStateSampler::sampleGaussian(ob::State *state, const ob::State *mean, double stdDev)
{
    sampler_->sampleGaussian(state, mean, stdDev);
}

unsigned int BiasedStateSampler::gmmSize()
{
    return GMM.size();
}
