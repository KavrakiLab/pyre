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

/* Author: Constantinos Chamzas  */
#ifndef PYRE_SPARK
#define PYRE_SPARK

#include <pyre/class_forward.h>
#include <pyre/pyre.h>
#include <pyre/database.h>
#include <robowflex_library/adapter.h>
#include <vector>

/** \cond IGNORE */
namespace robowflex
{
    CLASS_FORWARD(Geometry);
}  // namespace robowflex
   /** \endcond */

namespace pyre
{
    /** \cond IGNORE */
    CLASS_FORWARD(Spark);
    CLASS_FORWARD(SparkPrimitive);
    /** \endcond */

    /** Brief The primitive for spark (pairs of obstacles) **/
    class SparkPrimitive : public Primitive
    {
    public:
        SparkPrimitive();
        ~SparkPrimitive();
        std::pair<robowflex::GeometryPtr, robowflex::GeometryPtr> geometries;  // the geometries in the
        std::pair<robowflex::RobotPose, robowflex::RobotPose> poses;           // the poses of the geometries
        std::pair<std::string, std::string> names;                             // the poses of the geometries
        double distance(Primitive *other);
        std::string getUID();
    };

    class Spark : public Pyre
    {
    public:
        /** Brief parametes for the SPARK framework**/
        struct Params
        {
            double dradius;
            double dclust;
            double dpairs;
            double lambda;
            double sigma;
        };

        /** \brief Constructor.
         *  Takes in the configuration file with the parameters and a database.
         *  \param[in] config The parameters used for spark.
         *  \param[in] db The database to load experiences from.
         */
        Spark(const std::string &config, const DatabasePtr &db);

        /** \brief Constructor.
         *  Takes in a configuration file with the parameters.
         *  \param[in] config The parameters used for spark.
         */
        Spark(const std::string &config);

        /** \brief Destructor.
         *  Deallocates the memory allocated for the database.
         */
        ~Spark();

        std::vector<Entry *>
        processExperience(const robowflex::TrajectoryConstPtr &traj, const robowflex::SceneConstPtr &scene,
                          const robowflex::MotionRequestBuilderConstPtr &request) override;

        /** \brief gets all the primitives from the current Scene
         */
        std::vector<SparkPrimitive *> extractPrimitives(const robowflex::SceneConstPtr &scene);

        /** \brief Creates the entries for the given primitives, */
        std::vector<Entry *> createEntries(std::vector<SparkPrimitive *> &primitives,
                                           const robowflex::TrajectoryConstPtr &traj,
                                           const robowflex::SceneConstPtr &scene);

        /** \brief Stores the current state of experiences */
        void storeExperiences(const std::string &db_file) override;

        /** \brief Loads a set of experiences */
        void loadExperiences(const std::string &db_file) override;

        /** \brief get the Database */
        const DatabasePtr getDB();

        bool setBiasedSampler(const ompl::geometric::SimpleSetupPtr &ss,
                              const robowflex::SceneConstPtr &scene,
                              const planning_interface::MotionPlanRequest &request) override;

    private:
        DatabasePtr db_;
        Params params_;
    };

}  // namespace pyre
#endif
