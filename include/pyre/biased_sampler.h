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

/* Author: Constantinos Chamzas */

#ifndef PYRE_BIASED_STATE_SAMPLER_
#define PYRE_BIASED_STATE_SAMPLER_

#include <ros/console.h>
#include <unordered_set>

#include <ompl/base/StateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

#include <pyre/math.h>
#include <Eigen/Dense>

namespace ob = ompl::base;
typedef ompl_interface::ModelBasedStateSpace::StateType *StatePtr;

namespace pyre
{
    class BiasedStateSampler : public ob::StateSampler
    {
    public:
        /** \brief Multivariate Normal */
        struct MVN
        {
            Eigen::VectorXd mean;
            Eigen::MatrixXd Q;
            MVN(const Eigen::VectorXd &m, double std) : mean(m)
            {
                // This is much faster than calling the (full variance) constructor.
                Q = Eigen::MatrixXd::Identity(m.size(), m.size()) * std;
            }

            // Adapted from here: http://blog.sarantop.com/notes/mvn
            MVN(Eigen::VectorXd m, Eigen::MatrixXd sigma)
            {
                if (m.size() != sigma.rows() || m.size() != sigma.cols())
                {
                    ROS_ERROR("Dimentions of Covariance matrix and Mean mismatch!");
                    throw "Dimentions of Covariance matrix and Mean mismatch!";
                }

                mean = m;
                // Find the eigen vectors of the covariance matrix
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(sigma);
                Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors().real();

                // Find the eigenvalues of the covariance matrix
                Eigen::MatrixXd eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();

                // Find the transformation matrix, Note this is time consuming.
                Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(eigenvalues);
                Eigen::MatrixXd sqrt_eigenvalues = es.operatorSqrt();
                Q = eigenvectors * sqrt_eigenvalues;
            }
            Eigen::VectorXd sample()
            {
                // sample a vector from a normal with mean 0 and std 1
                auto x = Eigen::VectorXd(mean.size());
                for (unsigned int i = 0; i < x.size(); i++)
                    x[i] = math::gaussian(0, 1);

                // transform the vector for given mean and Gaussian
                return Q * x + mean;
            }
        };

        /** \brief Constructors */
        BiasedStateSampler(const ob::StateSpace *space);
        BiasedStateSampler(const ob::StateSpace *space, std::vector<Eigen::VectorXd> &mean,
                           const double lambda, const double std);
        BiasedStateSampler(const ob::StateSpace *space, std::vector<Eigen::VectorXd> &mean,
                           std::vector<Eigen::MatrixXd> Sigma, const double lambda);

        ~BiasedStateSampler() override = default;

        /** \brief Uniform sampling is ovverided to sample in a biased manner.*/
        void sampleUniform(ob::State *state) override;

        /** \brief UniformNear is NOT biased.*/
        void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override;

        /** \brief sampleGaussian is NOT biased.*/
        void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override;

        /** \brief Returns the size fo the Gaussian Mixture Model (number of local samplers)*/
        unsigned int gmmSize();

    protected:
        /** \brief The sampler to build upon */
        ob::StateSamplerPtr sampler_;

    private:
        ompl::PDF<MVN> GMM;
        std::shared_ptr<ompl::NearestNeighbors<MVN *>> nn_;
        std::unordered_set<std::string> set_;

        /** This must be from to 0 to 1 */
        double lambda_;
    };
}  // namespace pyre

#endif
