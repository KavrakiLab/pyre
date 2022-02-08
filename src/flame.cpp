/***********************************************************************
 * Copyright (C) <2021>  <Rice University>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ************************************************************************/

/* Author: Constantinos Chamzas */

#include <string>
#include <robowflex_library/robot.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/trajectory.h>

#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBaseImpl.h>

#include <pyre/flame.h>
#include <pyre/database.h>
#include <pyre/biased_sampler.h>
#include <pyre/yaml.h>
#include <pyre/distances.h>

using namespace pyre;

FlamePrimitive::FlamePrimitive()
{
}

double FlamePrimitive::distance(Primitive *o)
{
    auto other = static_cast<FlamePrimitive *>(o);
    return dist::octoDistance(grid, other->grid, side_size) + dist::transformDistance(pose, other->pose, 0.5);
}

std::string FlamePrimitive::getUID()
{
    auto t = pose.translation();
    auto r = Eigen::Quaterniond(pose.rotation());

    std::stringstream ss;
    ss.precision(2);

    for (int i = 0; i < side_size; i++)
        for (int j = 0; j < side_size; j++)
            for (int k = 0; k < side_size; k++)
                ss << grid[i][j][k];

    ss << t.x() << t.y() << t.z() << r.x() << r.y() << r.z() << r.w() << std::endl;
    return ss.str();
}

FlamePrimitive::~FlamePrimitive()
{
    delete grid;
}

Flame::Flame(const std::string &config, const DatabasePtr &db) : Pyre(), db_(db)
{
    const auto config_file = io::resolvePackage(config);
    auto &yaml = io::loadFileToYAML(config_file);

    if (yaml.first)
    {
        auto node = yaml.second;

        if (io::isNode(node["lambda"]))
            params_.lambda = node["lambda"].as<double>();
        else
            ROS_ERROR("No lambda(bias ratio) in YAML file %s", config.c_str());
        if (io::isNode(node["sigma"]))
            params_.sigma = node["sigma"].as<double>();
        else
            ROS_ERROR("No sigma(variance) in YAML file %s", config.c_str());
        if (io::isNode(node["octo_max_depth"]))
            params_.octo_max_depth = node["octo_max_depth"].as<double>();
        else
            ROS_ERROR("No octo_max_depth in YAML file %s", config.c_str());
        if (io::isNode(node["octo_min_depth"]))
            params_.octo_min_depth = node["octo_min_depth"].as<double>();
        else
            ROS_ERROR("No octo_min_depth in YAML file %s", config.c_str());
    }
    else
        ROS_ERROR("Failed to read file %s", config_file.c_str());
}

Flame::Flame(const std::string &config) : Flame(config, std::make_shared<Database>())
{
}

Flame::~Flame()
{
    db_->clear();
}

std::vector<Entry *> Flame::processExperience(const robowflex::TrajectoryConstPtr &traj,
                                              const robowflex::SceneConstPtr &scene,
                                              const robowflex::MotionRequestBuilderConstPtr &request)
{
    auto primitives = extractPrimitives(scene);
    auto new_entries = createEntries(primitives, traj, scene);
    return new_entries;
}

std::pair<bool, bool ***> Flame::fillGrid(const octomap::OcTree *tree, Eigen::Vector3d center,
                                          const unsigned int size, const double res)
{
    bool occupied = false;

    bool ***grid = new bool **[size];
    for (size_t i = 0; i < size; i++)
    {
        grid[i] = new bool *[size];
        for (size_t j = 0; j < size; j++)
        {
            grid[i][j] = new bool[size];
            for (size_t k = 0; k < size; k++)
            {
                grid[i][j][k] = false;

                // Search for the cordinate near that cube. Fill with occupied value or not
                auto leaf =
                    tree->search(center.x() + (i - size / 2.) * res + res / 2,  //
                                 center.y() + (j - size / 2.) * res + res / 2,  //
                                 center.z() + (k - size / 2.) * res + res / 2, params_.octo_max_depth);

                // The leaf can be NULL if that space is unknown. If unknown it will be considered empty
                if (leaf != NULL)
                    if (leaf->getOccupancy() >= tree->getOccupancyThres())
                    {
                        occupied = true;
                        grid[i][j][k] = true;
                    }
            }
        }
    }
    return std::pair<bool, bool ***>(occupied, grid);
}

std::vector<FlamePrimitive *> Flame::extractPrimitives(const robowflex::SceneConstPtr &scene)
{
    std::vector<FlamePrimitive *> primitives;
    // get the OctomapMessage
    const auto &octoMap = msgToMap(scene->getMessage().world.octomap.octomap);
    auto offset = math::toEigen(scene->getMessage().world.octomap.origin.position);

    octomap::OcTree *octo = dynamic_cast<octomap::OcTree *>(octoMap);
    if (octo == nullptr)
    {
        ROS_ERROR("Octomap was not read successfully");
        return primitives;
    }

    // The number of voxels(sidenum) in each sid eof the octobox will be  2**d
    int d1 = params_.octo_max_depth - params_.octo_min_depth;  // the depth of the octobox
    int side_size = pow(2, d1);
    // The voxelsize is the resolution*(2**extra_d)
    int d2 = (octo->getTreeDepth() - params_.octo_max_depth);  // the depth of the voxel octobox

    double voxel_res = pow(2, d2) * octo->getResolution();

    for (auto node = octo->begin_tree(), end = octo->end_tree(); node != end; ++node)
    {
        if (node.getDepth() == params_.octo_min_depth)
        {
            auto center = Eigen::Vector3d{node.getX(), node.getY(), node.getZ()};
            auto result = fillGrid(octo, center, side_size, voxel_res);
            // If the grid is occupied
            if (result.first)
            {
                auto p = new FlamePrimitive();

                p->voxel_res = voxel_res;
                p->side_size = side_size;
                p->pose = robowflex::RobotPose(Eigen::Translation3d(center + offset));
                p->grid = result.second;

                primitives.emplace_back(p);
            }
        }
    }

    ROS_INFO("Flame: Extracted %lu primitives from the scene", primitives.size());
    delete octo;

    return primitives;
}

bool Flame::setBiasedSampler(const ompl::geometric::SimpleSetupPtr &ss, const robowflex::SceneConstPtr &scene,
                             const planning_interface::MotionPlanRequest & /*request*/)

{
    if ((db_ == nullptr || db_->size() < 1))
    {
        ROS_ERROR("Flame::setCustomSampler: Database was not initialized or the size is too "  //
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
            // Using any retrieval radius less than 1.
            db_->nearestR(rentry, 0.1, list);
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

std::vector<Entry *> Flame::createEntries(std::vector<FlamePrimitive *> &primitives,
                                          const robowflex::TrajectoryConstPtr &traj,
                                          const robowflex::SceneConstPtr &scene)
{
    ROS_INFO("Flame: Proccesing path into entries");
    auto scene_copy = scene->deepCopy();

    auto traj_vec = traj->vectorize();

    std::vector<Entry *> entries;

    for (unsigned int i = 0; i < traj->getNumWaypoints(); i++)
    {
        auto state = std::make_shared<robot_state::RobotState>(traj->getTrajectoryConst()->getWayPoint(i));
        state->update(true);

        for (auto &p : primitives)
        {
            // Bounding box of the local occupancy grid.
            auto bsize = p->side_size * p->voxel_res;
            const auto &box = robowflex::Geometry::makeBox(bsize, bsize, bsize);

            scene_copy->updateCollisionObject("temp", box, p->pose);
            if (scene_copy->distanceToObject(*state, "temp") <= 0)
                entries.emplace_back(new Entry(p, traj_vec[i]));
            scene_copy->removeCollisionObject("temp");
        }
    }

    ROS_INFO("Flame: Path of Length %lu was proccessed to %lu entries ", traj->getNumWaypoints(),
             entries.size());
    return entries;
}

const DatabasePtr Flame::getDB()
{
    return db_;
}

void Flame::storeExperiences(const std::string &db_file)
{
    io::storeDatabase(db_, db_file);
}

void Flame::loadExperiences(const std::string &db_file)
{
    io::loadDatabase(db_, db_file);
}
