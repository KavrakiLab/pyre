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
#ifndef PYRE_YAML
#define PYRE_YAML

#include <pyre/database.h>
#include <yaml-cpp/yaml.h>

namespace pyre
{
    namespace io
    {
        bool isNode(const YAML::Node &node);

        /** \brief Resolves package:// URLs to their canonical form. The path does not need to exist, but the
         * package does. Can be used to write new files in packages. */
        const std::string resolvePackage(const std::string &path);

        /** \brief Resolves package:// URLs and relative file paths to their canonical form. */
        const std::string resolvePath(const std::string &path);

        /** \brief Loads a file from a YAML node */
        const std::pair<bool, YAML::Node> loadFileToYAML(const std::string &path);

        /** \brief Converts a YAML node to an Eigen Vector */
        Eigen::VectorXd toEigen(const YAML::Node &node);

        /** \brief Converts a YAML node to an std::vector */
        std::vector<double> toVector(const YAML::Node &node);

        /** \brief Loads the db, from a file. Both relative and absolute filenames are acceptable */
        bool loadDatabase(const DatabasePtr &db, const std::string &filename);

        /** \brief Loads entries from a file. Both relative and absolute filenames are acceptable */
        bool loadEntries(std::vector<Entry *> &entries, const std::string &filename);

        /** \brief Stores the db in a file. Both relative and absolute filenames are acceptable */
        void storeDatabase(const DatabasePtr &db, const std::string &filename);

        /** \brief Stores the entries in a file. Both relative and absolute filenames are acceptable */
        void storeEntries(const std::vector<Entry *> &vec, const std::string &filename);

    }  // namespace io
}  // namespace pyre

#endif
