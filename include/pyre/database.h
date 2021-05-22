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
#ifndef PYRE_DATABASE_
#define PYRE_DATABASE_

#include <iostream>
#include <memory>
#include <set>
#include <unordered_set>
#include <Eigen/Core>
#include <ompl/datastructures/NearestNeighbors.h>
#include <pyre/class_forward.h>

namespace pyre
{
    /** \cond IGNORE */
    CLASS_FORWARD(Database);
    /** \endcond */

    /** Brief The primitive for spark (pairs of obstacles) **/
    class Primitive
    {
    public:
        virtual double distance(Primitive *other) = 0;
        virtual std::string getUID() = 0;
        virtual ~Primitive(){};
    };

    struct Entry
    {
        Primitive *key;
        std::vector<Eigen::VectorXd> value;
        Entry(Primitive *p);
        Entry(Primitive *p, std::vector<double> vec);
        ~Entry();
    };

    class Database
    {
    public:
        Database();

        std::vector<Eigen::VectorXd> vectorize(std::vector<Entry *> entries);

        /** \brief adds a single entry to the database structure, combining local samplers of same entries */
        void add(Entry *e);

        /** \brief adds many entries in the database structure, combining local samplers of same entries */
        void add(std::vector<Entry *> entries);

        /** \brief Returns the nearest entry in the database */
        Entry *nearest(Entry *e);

        /** \brief Returns the nearest entries within Radius \a r */
        void nearestR(Entry *e, double r, std::vector<Entry *> &list);

        /** \brief gets all the entries as a list. */
        void toList(std::vector<Entry *> &list);

        /** \brief Prints the metrics of the database. */
        void printMetrics(std::ostream &os);

        /**Clears the database of all the entries. */
        void clear();

        /** Returns true on the empty database. */
        bool empty();

        /** \brief Returns the number of entries in DB. */
        unsigned int size();

    private:
        std::shared_ptr<ompl::NearestNeighbors<Entry *>> nn_;
        std::unordered_set<std::string> set_;
    };
}  // namespace pyre

#endif
