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

#include <string>
#include <robowflex_library/robot.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/trajectory.h>

#include <pyre/spark.h>
#include <pyre/database.h>
#include <pyre/biased_sampler.h>
#include <pyre/yaml.h>
#include <pyre/distances.h>

using namespace pyre;

SparkPrimitive::SparkPrimitive()
{
}

double SparkPrimitive::distance(Primitive *o)
{
    auto other = static_cast<SparkPrimitive *>(o);
    double dista = dist::transformDistance(poses.first, other->poses.first, 0.75) +
                   dist::transformDistance(poses.second, other->poses.second, 0.75) +
                   dist::geometryDistance(geometries.first, other->geometries.first) +
                   dist::geometryDistance(geometries.second, other->geometries.second);

    double distb = dist::transformDistance(poses.first, other->poses.second, 0.75) +
                   dist::transformDistance(poses.second, other->poses.first, 0.75) +
                   dist::geometryDistance(geometries.first, other->geometries.second) +
                   dist::geometryDistance(geometries.second, other->geometries.first);

    return std::min(dista, distb);
}

std::string SparkPrimitive::getUID()
{
    auto t1 = poses.first.translation();
    auto t2 = poses.second.translation();
    auto r1 = Eigen::Quaterniond(poses.first.rotation());
    auto r2 = Eigen::Quaterniond(poses.second.rotation());

    std::stringstream ss;
    ss.precision(2);
    ss << t1.x() << t1.y() << t1.z() << t2.x() << t2.y() << t2.z();
    ss << r1.x() << r1.y() << r1.z() << r1.w() << r2.x() << r2.y() << r2.z() << r2.w();
    // Add two times due to symmetry
    ss << t2.x() << t2.y() << t2.z() << t1.x() << t1.y() << t1.z();
    ss << r2.x() << r2.y() << r2.z() << r2.w() << r1.x() << r1.y() << r1.z() << r1.w();
    return ss.str();
}

SparkPrimitive::~SparkPrimitive()
{
}

Spark::Spark(const std::string &config, const DatabasePtr &db) : Pyre(), db_(db)
{
    const auto config_file = io::resolvePackage(config);
    auto &yaml = io::loadFileToYAML(config_file);

    if (yaml.first)
    {
        auto node = yaml.second;

        if (io::isNode(node["dradius"]))
            params_.dradius = node["dradius"].as<double>();
        else
            ROS_ERROR("No dradius distance in %s", config.c_str());
        if (io::isNode(node["dclust"]))
            params_.dclust = node["dclust"].as<double>();
        else
            ROS_ERROR("No dclust distance in YAML file %s", config.c_str());
        if (io::isNode(node["dpairs"]))
            params_.dpairs = node["dpairs"].as<double>();
        else
            ROS_ERROR("No dpairs in YAML file %s", config.c_str());
        if (io::isNode(node["lambda"]))
            params_.lambda = node["lambda"].as<double>();
        else
            ROS_ERROR("No lambda(bias ratio) in YAML file %s", config.c_str());
        if (io::isNode(node["sigma"]))
            params_.sigma = node["sigma"].as<double>();
        else
            ROS_ERROR("No sigma(variance) in YAML file %s", config.c_str());
    }
    else
        ROS_ERROR("Failed to read file %s", config_file.c_str());
}

Spark::Spark(const std::string &config) : Spark(config, std::make_shared<Database>())
{
}

Spark::~Spark()
{
    db_->clear();
}

std::vector<Entry *> Spark::processExperience(const robowflex::TrajectoryConstPtr &traj,
                                              const robowflex::SceneConstPtr &scene,
                                              const robowflex::MotionRequestBuilderConstPtr &request)
{
    auto primitives = extractPrimitives(scene);
    auto new_entries = createEntries(primitives, traj, scene);
    return new_entries;
}

std::vector<SparkPrimitive *> Spark::extractPrimitives(const robowflex::SceneConstPtr &scene)
{
    std::vector<SparkPrimitive *> primitives;
    auto objects = scene->getCollisionObjects();
    for (auto it1 = objects.begin(); it1 != objects.end(); ++it1)
        for (auto it2 = std::next(it1); it2 != objects.end(); ++it2)
            if (scene->distanceBetweenObjects(*it1, *it2) < params_.dpairs)
            {
                if (it1->find("<octomap>") == std::string::npos &&
                    it2->find("<octomap>") == std::string::npos)
                {
                    auto p = new SparkPrimitive();
                    // add first object
                    p->geometries.first = scene->getObjectGeometry(*it1);
                    p->poses.first = scene->getObjectPose(*it1);
                    p->names.first = *it1;

                    // add second object
                    p->geometries.second = scene->getObjectGeometry(*it2);
                    p->poses.second = scene->getObjectPose(*it2);
                    p->names.second = *it2;

                    primitives.emplace_back(p);
                }
            }

    ROS_INFO("Spark: Extracted %lu primitives from the scene with dclust: %f", primitives.size(),
             params_.dclust);

    return primitives;
}

bool Spark::setBiasedSampler(const ompl::geometric::SimpleSetupPtr &ss, const robowflex::SceneConstPtr &scene,
                             const planning_interface::MotionPlanRequest & /*request*/)

{
    if ((db_ == nullptr || db_->size() < 1))
    {
        ROS_ERROR("Spark::setCustomSampler: Database was not initialized or the size is too "  //
                  "small. Cannot Create a CustomSampler");
        return false;
    }

    ss->getStateSpace()->setStateSamplerAllocator([=](const ob::StateSpace *space) {
        auto start = std::chrono::high_resolution_clock::now();

        std::vector<Entry *> list;
        auto primitives = extractPrimitives(scene);
        for (const auto &lp : primitives)
        {
            auto *rentry = new Entry(lp);
            db_->nearestR(rentry, params_.dradius, list);
        }

        auto mu = db_->vectorize(list);
        std::shared_ptr<pyre::BiasedStateSampler> bsampler;

        if (!mu.size())  // If nothing is retrieved default to uniform sampling
            bsampler = std::make_shared<pyre::BiasedStateSampler>(space);
        else
            bsampler = std::make_shared<pyre::BiasedStateSampler>(space, mu, params_.lambda, params_.sigma);

        auto end = std::chrono::high_resolution_clock::now();

        statistics_.num_local_samplers = bsampler->gmmSize();
        statistics_.num_local_prims = list.size();
        statistics_.ret_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.;

        ROS_INFO("The retrieval time is %f seconds", statistics_.ret_time);

        return bsampler;
    });

    return true;
}

std::vector<Entry *> Spark::createEntries(std::vector<SparkPrimitive *> &primitives,
                                          const robowflex::TrajectoryConstPtr &traj,
                                          const robowflex::SceneConstPtr &scene)
{
    ROS_INFO("Spark: Proccesing path into entries");

    auto traj_vec = traj->vectorize();

    std::vector<Entry *> entries;

    for (unsigned int i = 0; i < traj->getNumWaypoints(); i++)
    {
        auto state = std::make_shared<robot_state::RobotState>(traj->getTrajectoryConst()->getWayPoint(i));
        state->update(true);

        for (auto &p : primitives)
        {
            // iterate over the local primitives parts (Pairs in this case)
            auto dist = scene->distanceToObject(*state, p->names.first) +
                        scene->distanceToObject(*state, p->names.second);

            // If the primitive is close enough to the robot add it to the database.
            if (dist < params_.dpairs)
                entries.emplace_back(new Entry(p, traj_vec[i]));
        }
    }

    ROS_INFO("Spark: Path of Length %lu was proccessed to %lu entries ", traj->getNumWaypoints(),
             entries.size());
    return entries;
}

const DatabasePtr Spark::getDB()
{
    return db_;
}

void Spark::storeExperiences(const std::string &db_file)
{
    io::storeDatabase(db_, db_file);
}

void Spark::loadExperiences(const std::string &db_file)
{
    io::loadDatabase(db_, db_file);
}
