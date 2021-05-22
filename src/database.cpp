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

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <pyre/database.h>

using namespace pyre;

Entry::Entry::Entry(Primitive *p) : key(p)
{
}

Entry::Entry(Primitive *p, std::vector<double> vec) : key(p)
{
    Eigen::Map<Eigen::VectorXd> x(vec.data(), vec.size());
    value.emplace_back(x);
}

Entry::~Entry()
{
    delete key;
}

Database::Database()
{
    nn_ = std::make_shared<ompl::NearestNeighborsGNAT<Entry *>>();
    nn_->setDistanceFunction([&](Entry *a, Entry *b) { return a->key->distance(b->key); });
}

std::vector<Eigen::VectorXd> Database::vectorize(std::vector<Entry *> entries)
{
    std::vector<Eigen::VectorXd> values;
    for (const auto &e : entries)
        for (const auto &val : e->value)
            values.emplace_back(val);

    return values;
}

void Database::add(Entry *e)
{
    bool unique = set_.insert(e->key->getUID()).second;

    if (not unique)
    {
        // Find the same entry in the db and add combine the local samplers
        Entry *t = nearest(e);
        for (const auto &val : e->value)
            t->value.emplace_back(val);
    }
    else
        nn_->add(e);
}

void Database::add(std::vector<Entry *> entries)
{
    for (const auto &e : entries)
        this->add(e);
}

Entry *Database::nearest(Entry *e)
{
    return nn_->nearest(e);
}

void Database::nearestR(Entry *e, double r, std::vector<Entry *> &list)
{
    std::vector<Entry *> tlist;
    nn_->nearestR(e, r, tlist);

    list.reserve(list.size() + tlist.size());
    list.insert(list.end(), tlist.begin(), tlist.end());
}

void Database::toList(std::vector<Entry *> &list)
{
    nn_->list(list);
}

void Database::printMetrics(std::ostream &os)
{
    std::vector<Entry *> list;
    this->toList(list);
    auto values = this->vectorize(list);

    os << "Number of Entries:" << nn_->size() << std::endl;
    os << "Number of Values" << values.size() << std::endl;
    os << "Ratios Values/Entry " << static_cast<float>(values.size()) / nn_->size() << std::endl;
}

/**Clears the database of all the entries **/
void Database::clear()
{
    nn_->clear();
    return;
}

/** Returns true on the empty database **/
bool Database::empty()
{
    return (nn_->size() == 0);
}
/** \brief Returns the number of entries in DB  **/
unsigned int Database::size()
{
    return nn_->size();
}

