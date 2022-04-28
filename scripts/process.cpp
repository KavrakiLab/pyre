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

#include <ros/ros.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/log.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <pyre/spark.h>
#include <pyre/flame.h>
#include <pyre/parser.h>
#include <pyre/yaml.h>

#include <boost/filesystem.hpp>  // for filesystem paths
#include <robowflex_library/io.h>

namespace rx = robowflex;
using namespace pyre;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "train", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    int start, end;
    std::string dataset, database;

    std::string exec_name = "trainer";

    // Parsing the parametes from the param server
    std::size_t error = 0;
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "database", database);

    parser::shutdownIfError(exec_name, error);

    // Load config.
    auto config = rx::IO::loadFileToYAML(dataset + "/config.yaml");
    if (!config.first)
    {
        ROS_ERROR("Failed to load YAML file `%s`.", (dataset + "/config.yaml").c_str());
        return -1;
    }

    auto robot = std::make_shared<rx::Robot>("robot");
    robot->initializeFromYAML(config.second["robot_description"].as<std::string>());

    std::string group = config.second["planning_group"].as<std::string>();
    std::string spark_config = "package://pyre/configs/spark_params.yaml";
    std::string flame_config = "package://pyre/configs/flame_params.yaml";

    // get the number of preapending zeros (minimum 4)
    int num_samples = config.second["num_samples"].as<int>();
    int dwidth = int(log10(num_samples)) + 1;
    dwidth = dwidth > 4 ? dwidth : 4;

    // Save the database in pyre in the last folder in databases
    // std::string folder = boost::filesystem::path(rx::IO::resolvePath(dataset)).filename().c_str();
    // std::string database = ros::package::getPath("pyre") + "/database/" + folder;

    auto spark = std::make_shared<Spark>(spark_config);
    auto flame = std::make_shared<Flame>(flame_config);

    for (int index = start; index <= end; index++)
    {
        const auto &scene_geom_file = dataset + "scene" + parser::toString(index, dwidth) + ".yaml";
        const auto &scene_sensed_file = dataset + "scene_sensed" + parser::toString(index, dwidth) + ".yaml";
        const auto &request_file = dataset + "request" + parser::toString(index, dwidth) + ".yaml";
        const auto &traj_file = dataset + "path" + parser::toString(index, dwidth) + ".yaml";

        // Create an empty Scene.
        auto scene_geom = std::make_shared<rx::Scene>(robot);
        if (not scene_geom->fromYAMLFile(scene_geom_file))
        {
            ROS_ERROR("Failed to read file: %s for scene", scene_geom_file.c_str());
            continue;
        }

        // Create an empty Scene.
        auto scene_sensed = std::make_shared<rx::Scene>(robot);
        if (not scene_sensed->fromYAMLFile(scene_sensed_file))
        {
            ROS_ERROR("Failed to read file: %s for scene", scene_sensed_file.c_str());
            continue;
        }

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(robot);
        if (not request->fromYAMLFile(request_file))
        {
            ROS_ERROR("Failed to read file: %s for request", request_file.c_str());
            continue;
        }

        const auto &trajectory = std::make_shared<rx::Trajectory>(robot, group);
        if (not trajectory->fromYAMLFile(*request->getStartConfiguration(), traj_file))
        {
            ROS_ERROR("Failed to read file: %s for request", traj_file.c_str());
            continue;
        }

        // Create spark entries
        auto entries_spark = spark->processExperience(trajectory, scene_geom, request);
        // Store spark entries
        io::storeEntries(entries_spark, database + "/sparkdb" + parser::toString(index, dwidth) + ".yaml");

        // Create spark entries
        auto entries_flame = flame->processExperience(trajectory, scene_sensed, request);
        // Store spark entries ROS_INFO("Training for %s is complete", dataset.c_str());
        io::storeEntries(entries_flame, database + "/flamedb" + parser::toString(index, dwidth) + ".yaml");
    }

    ros::shutdown();

    return 0;
}
