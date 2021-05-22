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
#include <boost/lexical_cast.hpp>

#include <moveit/version.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/yaml.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/builder.h>

#include <pyre/ebenchmarking.h>
#include <pyre/eplanner.h>

using namespace pyre;

EBenchmarkOutputter::EBenchmarkOutputter(const std::string &prefix) : prefix_(prefix)
{
}

EBenchmarkOutputter::~EBenchmarkOutputter()
{
}

void EBenchmarkOutputter::dumpResult(const robowflex::Benchmarker::Results &results)
{
    std::ofstream out;
    robowflex::IO::createFile(out, prefix_ + results.name + "_" + results.planner->getName() + ".log");

    out << "MoveIt! version " << MOVEIT_VERSION << std::endl;           // version
    out << "Experiment " << results.name << std::endl;                  // experiment
    out << "Running on " << robowflex::IO::getHostname() << std::endl;  // hostname
    out << "Starting at " << robowflex::IO::getDate() << std::endl;     // date

    // setup
    moveit_msgs::PlanningScene scene_msg;
    const auto &request = results.builder->getRequestConst();

    results.scene->getSceneConst()->getPlanningSceneMsg(scene_msg);

    out << "<<<|" << std::endl;
    out << "|>>>" << std::endl;

    // random seed (fake)
    out << "0 is the random seed" << std::endl;

    // time limit
    out << request.allowed_planning_time << " seconds per run" << std::endl;

    // memory limit
    out << "-1 MB per run" << std::endl;

    // num_runs
    out << results.runs.size() << " runs per planner" << std::endl;

    // total_time

    auto duration = results.finish - results.start;
    double total = (double)duration.total_milliseconds() / 1000.;
    out << total << " seconds spent to collect the data" << std::endl;

    // num_enums / enums
    out << "0 enum types" << std::endl;

    // num_planners
    out << "1 planners" << std::endl;

    // planners_data -> planner_data
    out << results.planner->getName() << std::endl;  // planner_name
    out << "0 common properties" << std::endl;

    out << (results.runs[0].metrics.size() + 7) << " properties for each run" << std::endl;  // run_properties
    out << "timePure REAL" << std::endl;
    out << "timeTotal REAL" << std::endl;
    out << "success BOOLEAN" << std::endl;
    out << "retrievalTime REAL" << std::endl;
    out << "numLocalSamplers INT" << std::endl;
    out << "numLocalPrims INT" << std::endl;
    out << "graphstates INT" << std::endl;

    std::vector<std::reference_wrapper<const std::string>> keys;
    for (const auto &metric : results.runs[0].metrics)
    {
        class toString : public boost::static_visitor<const std::string>
        {
        public:
            const std::string operator()(int /* dummy */) const
            {
                return "INT";
            }

            const std::string operator()(double /* dummy */) const
            {
                return "REAL";
            }

            const std::string operator()(bool /* dummy */) const
            {
                return "BOOLEAN";
            }
        };

        const auto &name = metric.first;
        keys.emplace_back(name);

        out << name << " " << boost::apply_visitor(toString(), metric.second) << std::endl;
    }

    out << results.runs.size() << " runs" << std::endl;
    auto planner = dynamic_cast<const pyre::EPlanner *>(results.planner.get());
    if (not planner)
        ROS_ERROR("EBenchmarkOutputter:: benchmarked planner is not an EPlanner");

    auto stats = planner->getStats();
    auto ss = planner->getSS();
    ompl::base::PlannerData pd(ss->getSpaceInformation());
    ss->getPlannerData(pd);

    for (const auto &run : results.runs)
    {
        out << run.time - stats.ret_time << "; "  //
            << run.time << "; "                   //
            << run.success << "; "                //
            << stats.ret_time << "; "             //
            << stats.num_local_samplers << "; "   //
            << stats.num_local_prims << "; "      //
            << pd.numVertices() << "; ";

        for (const auto &key : keys)
            out << boost::apply_visitor(robowflex::Benchmarker::Results::Run::toString(),
                                        run.metrics.find(key)->second)
                << "; ";

        out << std::endl;
    }

    out << "." << std::endl;
    out.close();
}
