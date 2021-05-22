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
#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/log.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <pyre/ebenchmarking.h>
#include <pyre/spark.h>
#include <pyre/flame.h>
#include <pyre/database.h>
#include <pyre/parser.h>
#include <pyre/yaml.h>
#include <pyre/eplanner.h>

#include <boost/filesystem.hpp>  // for filesystem paths
#include <robowflex_library/io.h>

namespace rx = robowflex;
using namespace pyre;

enum Algo
{
    UNIFORM = 0,
    SPARK = 1,
    FLAME = 2,
};

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "benchmark", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    int start, end, algo, reps;
    std::string dataset, database, postfix, planner_name;
    double time;

    std::string exec_name = "benchmark";

    // Parsing the parametes from the param server
    std::size_t error = 0;
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "reps", reps);
    error += !parser::get(exec_name, node, "algo", algo);
    error += !parser::get(exec_name, node, "time", time);
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "database", database);
    error += !parser::get(exec_name, node, "planner_name", planner_name);
    error += !parser::get(exec_name, node, "postfix", postfix);

    parser::shutdownIfError(exec_name, error);

    auto robot = std::make_shared<rx::FetchRobot>();
    robot->initialize(false);
    std::string spark_config = "package://pyre/configs/spark_params.yaml";
    std::string flame_config = "package://pyre/configs/flame_params.yaml";
    std::string ompl_config = "package://pyre/configs/ompl_planning.yaml";

    auto db = std::make_shared<Database>();
    PyrePtr pyre;

    // Load Database and databa
    if (algo == Algo::SPARK || algo == Algo::FLAME)
    {
        io::loadDatabase(db, database + ".yaml");
        if (algo == Algo::SPARK)
            pyre = std::make_shared<Spark>(spark_config, db);
        else if (algo == Algo::FLAME)
            pyre = std::make_shared<Flame>(flame_config, db);
    }

    const auto &bench = std::make_shared<rx::Benchmarker>();

    for (int index = start; index <= end; index++)
    {
        const auto &scene_geom_file = dataset + "scene" + parser::toString(index) + ".yaml";
        const auto &scene_sensed_file = dataset + "scene_sensed" + parser::toString(index) + ".yaml";
        const auto &request_file = dataset + "request" + parser::toString(index) + ".yaml";

        // Create an empty Scene.
        auto scene_geom = std::make_shared<rx::Scene>(robot);
        if (not scene_geom->fromYAMLFile(scene_geom_file))
            ROS_ERROR("Failed to read file: %s for scene", scene_geom_file.c_str());

        // Create an empty Scene.
        auto scene_sensed = std::make_shared<rx::Scene>(robot);
        if (algo == Algo::FLAME)
            if (not scene_sensed->fromYAMLFile(scene_sensed_file))
                ROS_ERROR("Failed to read file: %s for scene", scene_sensed_file.c_str());

        // Create an empty motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(robot);
        if (not request->fromYAMLFile(request_file))
            ROS_ERROR("Failed to read file: %s for request", request_file.c_str());

        request->setConfig(planner_name);
        request->setNumPlanningAttempts(1);
        request->setAllowedPlanningTime(time);

        EPlannerPtr planner;
        // Individual changes for every planner
        switch (algo)
        {
            case Algo::UNIFORM:
                planner = std::make_shared<EPlanner>(robot, "Uniform_" + planner_name + postfix);
                break;
            case Algo::SPARK:
                planner =
                    std::make_shared<EPlanner>(robot, "SPARK_" + planner_name + postfix, pyre, scene_geom);
                break;
            case Algo::FLAME:
                planner =
                    std::make_shared<EPlanner>(robot, "FLAME_" + planner_name + postfix, pyre, scene_sensed);
                break;
            default:
                ROS_ERROR("Algo can have value only in {0,1,2}");
        }

        planner->initialize(ompl_config);
        bench->addBenchmarkingRequest("result" + parser::toString(index), scene_geom, planner, request);
    }

    // The dataset name is the last folder of the dataset path.
    std::string folder = boost::filesystem::path(rx::IO::resolvePath(dataset)).filename().c_str();
    std::string bpath = ros::package::getPath("pyre") + "/benchmark/" + folder + "/";

    ROS_INFO("Starting Benhmarking");

    const auto &options = rx::Benchmarker::Options(reps, 0);
    bench->benchmark({std::make_shared<EBenchmarkOutputter>(bpath)}, options);

    ROS_INFO("Testing for %s is with %s complete", dataset.c_str(), database.c_str());
    ros::shutdown();

    return 0;
}

