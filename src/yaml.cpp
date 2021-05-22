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

/* Author: Constantinos Chamzas, Zachary Kingston */

// Robowflex
#include <robowflex_library/scene.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/geometry.h>
#include <robowflex_library/adapter.h>
#include <robowflex_library/io.h>

#include <pyre/yaml.h>
#include <pyre/spark.h>
#include <pyre/flame.h>
#include <fstream>

using namespace pyre;
namespace
{
    static const std::string boolToString(bool b)
    {
        return b ? "true" : "false";
    }
    static bool nodeToBool(const YAML::Node &n)
    {
        std::string s = n.as<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s == "true";
    }

    robowflex::RobotPose YAMLToPose(const YAML::Node &node)
    {
        Eigen::Vector3d position{0., 0., 0.};
        if (io::isNode(node["position"]))
        {
            if (node["position"].size() != 3)
                ROS_ERROR(" \"position\" must be three-dimensional");

            position = io::toEigen(node["position"]);
        }
        else
            ROS_ERROR("No \"position\" entry in pose in YAML");

        Eigen::Vector4d ornt{0., 0., 0., 0.};
        if (io::isNode(node["orientation"]))
        {
            if (node["orientation"].size() != 4)
                ROS_ERROR("orientation must be four-dimensional");

            ornt = io::toEigen(node["orientation"]);
        }
        else
            ROS_ERROR("No \"orientation\"in pose in YAML");

        robowflex::RobotPose pose;
        pose.translation() = position;
        pose.linear() = Eigen::Quaterniond(ornt).toRotationMatrix();
        return pose;
    }

    bool ***YAMLToGrid(const YAML::Node &node, unsigned int size)
    {
        bool ***grid = new bool **[size];
        for (size_t i = 0; i < size; i++)
        {
            grid[i] = new bool *[size];
            for (size_t j = 0; j < size; j++)
            {
                grid[i][j] = new bool[size];
                for (size_t k = 0; k < size; k++)
                    grid[i][j][k] = nodeToBool(node[i * size * size + j * size + k]);
            }
        }
        return grid;
    }

    double **YAMLToMatrix(const YAML::Node &node)
    {
        unsigned int size = node.size();
        double **mat = new double *[size];
        int i = -1;
        for (const auto &el : node)
        {
            i++;
            mat[i] = new double[size];
            for (size_t j = 0; j < size; j++)
            {
                mat[i][j] = el[j].as<double>();
            }
        }
        return mat;
    }

    robowflex::GeometryPtr YAMLToGeometry(const YAML::Node &node)
    {
        Eigen::Vector3d dimensions{0., 0., 0.};
        if (io::isNode(node["dimensions"]))
        {
            if (node["dimensions"].size() != 3)
                ROS_ERROR("dimentions must be three-dimensional");

            dimensions = io::toEigen(node["dimensions"]);
        }
        else
            ROS_ERROR("No dimensions entry in geometry in YAML");

        return robowflex::Geometry::makeBox(dimensions[0], dimensions[1], dimensions[2]);
    }

    Entry *YAMLToEntry(const YAML::Node &e)
    {
        Entry *entry;
        if (io::isNode(e["key"]))
        {
            auto key = e["key"];
            if (io::isNode(key["geometries"]))
            {
                auto *prim = new SparkPrimitive();
                if (key["geometries"].IsSequence())
                {
                    prim->geometries.first = YAMLToGeometry(key["geometries"][0]);
                    prim->geometries.second = YAMLToGeometry(key["geometries"][1]);
                }
                else
                    ROS_ERROR("Geometry must be a sequence");

                if (key["poses"].IsSequence())
                {
                    prim->poses.first = YAMLToPose(key["poses"][0]);
                    prim->poses.second = YAMLToPose(key["poses"][1]);
                }
                else
                    ROS_ERROR("Poses must be a sequence");

                // Create an entry with this key.
                entry = new Entry(prim);
            }
            else if (io::isNode(key["grid"]))
            {
                auto *prim = new FlamePrimitive();
                if (io::isNode(key["voxel_res"]))
                    prim->voxel_res = key["voxel_res"].as<double>();
                else
                    ROS_ERROR("Voxel_res node does not exist");
                if (io::isNode(key["side_size"]))
                    prim->side_size = key["side_size"].as<double>();
                else
                    ROS_ERROR("Side_size node does not exist");

                if (io::isNode(key["grid"]))
                    prim->grid = YAMLToGrid(key["grid"], prim->side_size);
                else
                    ROS_ERROR("Grid node does not exist");

                if (io::isNode(key["pose"]))
                    prim->pose = YAMLToPose(key["pose"]);
                else
                    ROS_ERROR("Pose node does not exist");

                // Create an entry with this key.
                entry = new Entry(prim);
            }
            else
                ROS_ERROR("This primitive is not supported");
        }
        else
            ROS_ERROR("Node does not have a key");

        if (e["value"].IsSequence())
            for (const auto &val : e["value"])
                entry->value.emplace_back(io::toEigen(val));
        else
            ROS_ERROR("Value must be a sequence");

        return entry;
    }

    std::shared_ptr<pyre::Database> YAMLToDB(const YAML::Node &node, const DatabasePtr &db)
    {
        std::vector<Entry *> entries;

        for (const auto &e : node)
            entries.emplace_back(YAMLToEntry(e));

        db->add(entries);

        return db;
    }

    void YAMLToAdjMat(const YAML::Node &node, std::vector<std::vector<Entry *>> &adj_mat)
    {
        for (const auto &vec : node)
        {
            std::vector<Entry *> entries;
            for (const auto &e : vec["row"])
                entries.emplace_back(YAMLToEntry(e));
            adj_mat.emplace_back(entries);
        }
    }

    YAML::Node EigenToYAML(const Eigen::Ref<const Eigen::VectorXd> &vec)
    {
        YAML::Node node;
        for (unsigned int i = 0; i < vec.size(); ++i)
            node.push_back(vec[i]);

        return node;
    }

    YAML::Node QuaternionToYAML(const Eigen::Quaternionf &q)
    {
        YAML::Node node;
        node.push_back(q.x());
        node.push_back(q.y());
        node.push_back(q.z());
        node.push_back(q.w());

        return node;
    }

    YAML::Node VectorToYAML(const std::vector<double> &vec)
    {
        YAML::Node node;
        for (unsigned int i = 0; i < vec.size(); ++i)
            node.push_back(vec[i]);

        return node;
    }

    YAML::Node GridToYAML(bool ***grid, int size)
    {
        YAML::Node node;
        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
                for (int k = 0; k < size; k++)
                    node.push_back(grid[i][j][k]);

        return node;
    }

    const YAML::Node GeometryToYAML(const robowflex::GeometryPtr &geom)
    {
        YAML::Node node;
        node["type"] = robowflex::Geometry::ShapeType::toString(geom->getType());

        node["dimensions"] = EigenToYAML(geom->getDimensions());

        return node;
    }

    const YAML::Node PoseToYAML(const robowflex::RobotPose &pose)
    {
        YAML::Node node;
        node["position"] = EigenToYAML(pose.translation());
        node["orientation"] = QuaternionToYAML(Eigen::Quaternionf(pose.linear().cast<float>()));
        return node;
    }

    const YAML::Node SparkPrimitiveToYAML(const SparkPrimitive *prim)
    {
        YAML::Node node;
        node["geometries"].push_back(GeometryToYAML(prim->geometries.first));
        node["geometries"].push_back(GeometryToYAML(prim->geometries.second));
        node["poses"].push_back(PoseToYAML(prim->poses.first));
        node["poses"].push_back(PoseToYAML(prim->poses.second));

        return node;
    }

    const YAML::Node FlamePrimitiveToYAML(const FlamePrimitive *prim)
    {
        YAML::Node node;
        node["voxel_res"] = prim->voxel_res;
        node["side_size"] = prim->side_size;
        node["pose"] = PoseToYAML(prim->pose);
        node["grid"] = GridToYAML(prim->grid, prim->side_size);

        return node;
    }

    const YAML::Node EntryToYAML(const Entry *entry)
    {
        YAML::Node node;
        auto spark_key = dynamic_cast<SparkPrimitive *>(entry->key);
        if (spark_key)
            node["key"] = SparkPrimitiveToYAML(spark_key);

        auto flame_key = dynamic_cast<FlamePrimitive *>(entry->key);
        if (flame_key)
            node["key"] = FlamePrimitiveToYAML(flame_key);

        for (const auto &v : entry->value)
            node["value"].push_back(EigenToYAML(v));

        return node;
    }

    const YAML::Node EntryVecToYAML(const std::vector<Entry *> &vec)
    {
        YAML::Node node;
        for (const auto &entry : vec)
            node["row"].push_back(EntryToYAML(entry));

        return node;
    }

}  // namespace

bool io::isNode(const YAML::Node &node)
{
    bool r = true;
    try
    {
        r = !node.IsNull();
        r = node;

        if (r)
            try
            {
                r = node.as<std::string>() != "~";
            }
            catch (std::exception &e)
            {
            }
    }
    catch (YAML::InvalidNode &e)
    {
        return false;
    }

    return r;
}

const std::string io::resolvePackage(const std::string &path)
{
    return robowflex::IO::resolvePackage(path);
}

const std::string io::resolvePath(const std::string &path)
{
    return robowflex::IO::resolvePath(path);
}

const std::pair<bool, YAML::Node> io::loadFileToYAML(const std::string &path)
{
    YAML::Node file;
    try
    {
        return std::make_pair(true, YAML::LoadFile(path));
    }
    catch (std::exception &e)
    {
        return std::make_pair(false, file);
    }
}

Eigen::VectorXd io::toEigen(const YAML::Node &node)
{
    if (!node.IsSequence())
        ROS_ERROR("Node is not a sequence");

    Eigen::VectorXd x(node.size());
    for (unsigned int i = 0; i < node.size(); ++i)
        x[i] = node[i].as<double>();

    return x;
}
std::vector<double> io::toVector(const YAML::Node &node)
{
    if (!node.IsSequence())
        ROS_ERROR("Node is not a sequence");

    std::vector<double> x;
    for (unsigned int i = 0; i < node.size(); ++i)
        x.push_back(node[i].as<double>());

    return x;
}

bool io::loadDatabase(const DatabasePtr &db, const std::string &filename)
{
    const auto &result = io::loadFileToYAML(resolvePackage(filename));

    if (!result.first)
    {
        ROS_ERROR("Failed to read file %s", resolvePackage(filename).c_str());
        return false;
    }
    else
        ROS_INFO("Database file was read succesfully");

    try
    {
        YAMLToDB(result.second, db);
        ROS_INFO("%d entries loaded for Database %s", db->size(), filename.c_str());
    }
    catch (std::exception &e)
    {
        ROS_ERROR("In YAML File %s %s", resolvePackage(filename).c_str(), e.what());
        return false;
    }

    return true;
}

bool io::loadEntries(std::vector<Entry *> &entries, const std::string &filename)
{
    const auto &result = io::loadFileToYAML(resolvePath(filename));

    if (!result.first)
    {
        ROS_ERROR("Failed to read file %s", resolvePath(filename).c_str());
        return false;
    }
    else
        ROS_INFO("Entries file was read succesfully");

    try
    {
        for (const auto &e : result.second)
            entries.emplace_back(YAMLToEntry(e));

        ROS_INFO("%lu entries loaded from filename %s", entries.size(), filename.c_str());
    }
    catch (std::exception &e)
    {
        ROS_ERROR("In YAML File %s %s", resolvePackage(filename).c_str(), e.what());
        return false;
    }

    return true;
}

void io::storeDatabase(const DatabasePtr &db, const std::string &filename)
{
    ROS_INFO("Storing database in %s with %d entries", resolvePackage(filename).c_str(), db->size());

    std::ofstream fout;

    robowflex::IO::createFile(fout, resolvePackage(filename));
    YAML::Emitter out;

    std::vector<Entry *> list;
    db->toList(list);

    out << YAML::BeginSeq;
    for (const auto &el : list)
        out << YAML::Flow << EntryToYAML(el);

    out << YAML::EndSeq;
    fout << out.c_str();
    fout.close();
}

void io::storeEntries(const std::vector<Entry *> &list, const std::string &filename)
{
    ROS_INFO("Storing Entries list in %s with %lu entries", resolvePackage(filename).c_str(), list.size());

    std::ofstream fout;
    robowflex::IO::createFile(fout, resolvePackage(filename));

    YAML::Emitter out;

    out << YAML::BeginSeq;
    for (const auto &el : list)
        out << YAML::Flow << EntryToYAML(el);

    out << YAML::EndSeq;
    fout << out.c_str();
    fout.close();
}
