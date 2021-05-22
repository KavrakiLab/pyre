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
#include <robowflex_library/tf.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/random.h>

#include <pyre/ebenchmarking.h>
#include <pyre/spark.h>
#include <pyre/flame.h>
#include <pyre/database.h>
#include <pyre/parser.h>
#include <pyre/yaml.h>
#include <pyre/eplanner.h>
#include <pyre/math.h>

#include <boost/filesystem.hpp>  // for filesystem paths

namespace rx = robowflex;
using namespace pyre;

enum Algo
{
    SPARK = 0,
    FLAME = 1,
};

enum Show
{
    SCENE = 0,
    DB = 1,
};

visualization_msgs::Marker createFlameMarker(FlamePrimitive *prim, Eigen::Vector4d color, rx::RobotPose,
                                             Eigen::Vector3d scale)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    {
        auto res = prim->voxel_res;
        auto size = prim->side_size;
        auto grid = prim->grid;

        for (int i = 0; i < size; ++i)
            for (int j = 0; j < size; ++j)
                for (int k = 0; k < size; ++k)
                    if (grid[i][j][k])
                    {
                        geometry_msgs::Point p;
                        // The rviz cubes have the center at top left corner, while the
                        // the octomap cells have it in the center.That's why we have res/2.
                        p.x = (i - size / 2.) * res + res / 2;
                        p.y = (j - size / 2.) * res + res / 2;
                        p.z = (k - size / 2.) * res + res / 2;
                        marker.points.push_back(p);
                    }
    }

    marker.header.frame_id = "map";
    marker.frame_locked = true;

    marker.header.stamp = ros::Time().now();
    marker.ns = "/robowflex";
    marker.id = rx::RNG::uniformInt(0, 10000);

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = rx::TF::poseEigenToMsg(prim->pose);
    marker.scale = rx::TF::vectorEigenToMsg(scale);

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    return marker;
}

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "visualize", ros::init_options::AnonymousName);
    ros::NodeHandle node("~");

    int start, end, algo, show;
    std::string dataset, database, planner_name;

    std::string exec_name = "tester";

    // Parsing the parametes from the param server
    std::size_t error = 0;
    error += !parser::get(exec_name, node, "start", start);
    error += !parser::get(exec_name, node, "end", end);
    error += !parser::get(exec_name, node, "algo", algo);
    error += !parser::get(exec_name, node, "show", show);
    error += !parser::get(exec_name, node, "dataset", dataset);
    error += !parser::get(exec_name, node, "database", database);

    parser::shutdownIfError(exec_name, error);

    auto robot = std::make_shared<rx::FetchRobot>();
    robot->initialize(false);
    auto rviz = std::make_shared<rx::IO::RVIZHelper>(robot);

    std::string spark_config = "package://pyre/configs/spark_params.yaml";
    std::string flame_config = "package://pyre/configs/flame_params.yaml";
    std::string ompl_config = "package://pyre/configs/ompl_planning.yaml";

    auto voxel_color = Eigen::Vector4d{1, 0, 0, 0.7};
    auto box_color = Eigen::Vector4d{0, 0, 1, 0.4};

    if (Show::SCENE == show)
    {
        for (int index = start; index <= end; index++)
        {
            const auto &scene_file = dataset + "scene" + (algo == Algo::FLAME ? "_sensed" : "") +
                                     parser::toString(index) + ".yaml";

            // Create an empty Scene.
            auto scene = std::make_shared<rx::Scene>(robot);
            if (not scene->fromYAMLFile(scene_file))
                ROS_ERROR("Failed to read file: %s for scene", scene_file.c_str());

            rviz->updateScene(scene);

            if (algo == Algo::SPARK)
            {
                auto spark = std::make_shared<Spark>(spark_config);
                auto primitives = spark->extractPrimitives(scene);
                // visualize the current decomposition
                for (const auto &prim : primitives)
                {
                    rviz->addGeometryMarker(prim->names.first, prim->geometries.first, "map",
                                            prim->poses.first, voxel_color);
                    rviz->addGeometryMarker(prim->names.first, prim->geometries.first, "map",
                                            prim->poses.first,

                                            voxel_color);

                    rviz->updateMarkers();
                    parser::waitForUser("Visualizing current primitive");
                    rviz->removeAllMarkers();
                    rviz->updateMarkers();
                }
            }
            else if (algo == Algo::FLAME)
            {
                auto flame = std::make_shared<Flame>(flame_config);
                auto primitives = flame->extractPrimitives(scene);
                // visualize the current decomposition
                for (const auto &prim : primitives)
                {
                    auto marker = createFlameMarker(prim, voxel_color, prim->pose,
                                                    {prim->voxel_res, prim->voxel_res, prim->voxel_res});
                    rviz->addMarker(marker, prim->getUID());

                    auto box_size = prim->voxel_res * prim->side_size;
                    auto box = robowflex::Geometry::makeBox(box_size, box_size, box_size);

                    // Add the bounding box of the local primitive
                    rviz->addGeometryMarker(prim->getUID() + "box", box, "map", prim->pose, box_color);
                    rviz->updateMarkers();
                    parser::waitForUser("Visualizing current primitive");
                    rviz->removeAllMarkers();
                    rviz->updateMarkers();
                }
            }
            else
                ROS_ERROR("No Algorithm %d exist", algo);

            rviz->removeAllMarkers();
            rviz->updateMarkers();
        }
    }
    else if (Show::DB == show)
    {
        auto db = std::make_shared<Database>();
        io::loadDatabase(db, database + ".yaml");
        std::vector<Entry *> entries;
        db->toList(entries);

        for (const auto &e : entries)
        {
            if (algo == Algo::SPARK)
            {
                auto prim = static_cast<SparkPrimitive *>(e->key);
                rviz->addGeometryMarker(prim->names.first, prim->geometries.first, "map", prim->poses.first,
                                        voxel_color);
                rviz->addGeometryMarker(prim->names.first, prim->geometries.first, "map", prim->poses.first,

                                        voxel_color);

                rviz->updateMarkers();
                parser::waitForUser("Visualizing current primitive");
            }
            else if (algo == Algo::FLAME)
            {
                auto prim = static_cast<FlamePrimitive *>(e->key);
                auto marker = createFlameMarker(prim, voxel_color, prim->pose,
                                                {prim->voxel_res, prim->voxel_res, prim->voxel_res});
                rviz->addMarker(marker, prim->getUID());

                auto box_size = prim->voxel_res * prim->side_size;
                auto box = robowflex::Geometry::makeBox(box_size, box_size, box_size);

                // Add the bounding box of the local primitive
                rviz->addGeometryMarker(prim->getUID() + "box", box, "map", prim->pose, box_color);
                rviz->updateMarkers();
                parser::waitForUser("Visualizing current primitive");
            }
            auto value = e->value;

            for (const auto &v : value)
            {
                robot->setGroupState("arm_with_torso", math::toVector(v));
                rviz->visualizeCurrentState();

                rviz->updateMarkers();
                parser::waitForUser("Visualizing current value");
            }
            rviz->removeAllMarkers();
            rviz->updateMarkers();
        }
    }
    else
        ROS_ERROR("No show %d is supported", show);
}
