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
#include <pyre/spark.h>
#include <pyre/database.h>
#include <pyre/parser.h>
#include <pyre/yaml.h>

#include <boost/filesystem.hpp>  // for filesystem paths
#include <robowflex_library/io.h>

namespace rx = robowflex;
using namespace pyre;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "merge", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    int start, end, dwidth;
    std::string database, database_out;

    std::string exec_name = "merger";

    // Parsing the parametes from the param server
    std::size_t error = 0;
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "database", database);
    error += !parser::get(exec_name, node, "database_out", database_out);
    // dwidth is the number of digits for the index number. It is needed to correctly read the files.
    error += !parser::get(exec_name, node, "dwidth", dwidth);

    if (database_out.empty())
        database_out = database;

    parser::shutdownIfError(exec_name, error);

    auto db_spark = std::make_shared<Database>();
    auto db_flame = std::make_shared<Database>();

    for (int index = start; index <= end; index++)
    {
        std::vector<EntryPtr> entries_spark;
        io::loadEntries(entries_spark, database + "/sparkdb" + parser::toString(index, dwidth) + ".yaml");
        db_spark->add(entries_spark);

        std::vector<EntryPtr> entries_flame;
        io::loadEntries(entries_flame, database + "/flamedb" + parser::toString(index, dwidth) + ".yaml");
        db_flame->add(entries_flame);
    }

    io::storeDatabase(db_spark, database_out + "/sparkdb" + parser::toString(start) + "_" +
                                    parser::toString(end) + ".yaml");

    io::storeDatabase(db_flame, database_out + "/flamedb" + parser::toString(start) + "_" +
                                    parser::toString(end) + ".yaml");

    ROS_INFO("Merging for %s is complete", database.c_str());

    ros::shutdown();

    return 0;
}
