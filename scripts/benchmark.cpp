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
#include <robowflex_library/benchmarking.h>
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

rx::Profiler::ComputeMetricCallback getPyreMetricCallBack(const std::string name)
{
    return [name](const rx::PlannerPtr &planner, const rx::SceneConstPtr &scene,
                  const planning_interface::MotionPlanRequest &request,
                  const rx::PlanData &run) -> rx::PlannerMetric {
        const auto &eplanner = std::dynamic_pointer_cast<const EPlanner>(planner);
        if (name == "num_local_prims")
            return eplanner->getStats().num_local_prims;
        else if (name == "num_local_samplers")
            return eplanner->getStats().num_local_samplers;
        else if (name == "ret_time")
            return eplanner->getStats().ret_time;
        else if (name == "pure_time")
            return run.time - eplanner->getStats().ret_time;
        else
            ROS_ERROR("Metric name %s does not exist", name.c_str());
        return 0;
    };
}

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "benchmark", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    int start, end, reps;
    std::string dataset, database, postfix, planner_name, algo;
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
    if (algo == "SPARK" || algo == "FLAME")
    {
        io::loadDatabase(db, database + ".yaml");
        pyre = (algo == "SPARK" ? pyre = std::make_shared<Spark>(spark_config, db) :
                                  std::make_shared<Flame>(flame_config, db));
    }

    // Benchmarker Options
    rx::Profiler::Options options;
    options.metrics = rx::Profiler::WAYPOINTS | rx::Profiler::CORRECT | rx::Profiler::LENGTH;

    rx::Experiment experiment("Experiment" + postfix, options, time, reps);
    // Add custom metrics
    for (const auto &metric : {"num_local_prims", "num_local_samplers", "ret_time", "pure_time"})
        experiment.getProfiler().addMetricCallback(metric, getPyreMetricCallBack(metric));

    // Change default OMPL settings to make planning easier
    rx::OMPL::Settings settings;
    // No need to interpolate
    settings.interpolate_solutions = false;
    // Simplifying adds planning time
    settings.simplify_solutions = false;
    // Since we use joint goals only 1 sample is needed.
    settings.max_goal_samples = 1;

    for (int index = start; index <= end; index++)
    {
        const auto &scene_geom_file = dataset + "scene" + parser::toString(index) + ".yaml";
        const auto &scene_sensed_file = dataset + "scene_sensed" + parser::toString(index) + ".yaml";
        const auto &request_file = dataset + "request" + parser::toString(index) + ".yaml";

        // Load the geometric Scene.
        auto scene_geom = std::make_shared<rx::Scene>(robot);
        if (not scene_geom->fromYAMLFile(scene_geom_file))
            ROS_ERROR("Failed to read file: %s for scene", scene_geom_file.c_str());

        // Load the sensed Scene (Only used by FLAME).
        auto scene_sensed = std::make_shared<rx::Scene>(robot);
        if (algo == "FLAME")
            if (not scene_sensed->fromYAMLFile(scene_sensed_file))
                ROS_ERROR("Failed to read file: %s for scene", scene_sensed_file.c_str());

        // Create the motion planning request.
        auto request = std::make_shared<robowflex::MotionRequestBuilder>(robot);
        if (not request->fromYAMLFile(request_file))
            ROS_ERROR("Failed to read file: %s for request", request_file.c_str());

        // Set the planner name e.g., RRTConnect.
        request->setConfig(planner_name);
        // Only use one thread for planning.
        request->setNumPlanningAttempts(1);
        // Set the allowble planning time.
        request->setAllowedPlanningTime(time);

        EPlannerPtr planner;
        // Individual changes for every planner
        if (algo == "UNIFORM")
            planner = std::make_shared<EPlanner>(robot, "UNIFORM_" + planner_name + postfix);
        else if (algo == "SPARK")
            planner = std::make_shared<EPlanner>(robot, "SPARK_" + planner_name + postfix, pyre, scene_geom);
        else if (algo == "FLAME")
            planner =
                std::make_shared<EPlanner>(robot, "FLAME_" + planner_name + postfix, pyre, scene_sensed);
        else
            ROS_ERROR("Chosen Sampling Strategy can only be UNIFORM, SPARK, FLAME");

        planner->initialize(ompl_config, settings);
        experiment.addQuery(planner->getName(), scene_geom, planner, request);
    }
    // The dataset name is the last folder of the dataset path.
    std::string folder = boost::filesystem::path(rx::IO::resolvePath(dataset)).filename().c_str();
    std::string bpath = ros::package::getPath("pyre") + "/benchmark/" + folder + "/" + algo;

    ROS_INFO("Starting Benchmarking");

    auto results_data = experiment.benchmark(1);

    rx::OMPLPlanDataSetOutputter output(bpath);
    output.dump(*results_data);

    ROS_INFO("Testing for %s is with %s complete", dataset.c_str(), database.c_str());
    ros::shutdown();

    return 0;
}
