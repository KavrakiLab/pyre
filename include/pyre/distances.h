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
#ifndef PYRE_DISTANCES
#define PYRE_DISTANCES

#include <robowflex_library/geometry.h>
namespace dist
{
    /** \brief L2 translation distance between two poses. */
    double trDistance(robowflex::RobotPose r1, robowflex::RobotPose r2);

    /** \brief L1 translation Distance between poses. */
    double trL1Distance(robowflex::RobotPose r1, robowflex::RobotPose r2);

    /** \brief rotation distance between two poses. */
    double rotDistance(robowflex::RobotPose r1, robowflex::RobotPose r2);

    /** \brief Weighted combination of rotation and translation (L2) distance between two poses. */
    double transformDistance(robowflex::RobotPose r1, robowflex::RobotPose r2, double a);

    /** \brief a size distance between two geometric primitives. */
    double geometryDistance(robowflex::GeometryPtr r1, robowflex::GeometryPtr g2);

    /** \brief Discrete distance between two occupancy grids. Returns 0 if they are exactly the same 1
     * otherwise. */
    double octoDistance(bool ***grid1, bool ***grid2, int size);

}  // namespace dist

#endif
