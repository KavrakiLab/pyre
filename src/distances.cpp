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

#include <pyre/distances.h>

double dist::trDistance(robowflex::RobotPose r1, robowflex::RobotPose r2)
{
    return (r1.translation() - r2.translation()).norm();
};

double dist::trL1Distance(robowflex::RobotPose r1, robowflex::RobotPose r2)
{
    auto diff = (r1.translation() - r2.translation());
    return std::max({std::abs(diff.x()), std::abs(diff.y()), std::abs(diff.z())});
};

double dist::rotDistance(robowflex::RobotPose r1, robowflex::RobotPose r2)
{
    return Eigen::Quaterniond(r1.linear()).angularDistance(Eigen::Quaterniond(r2.linear()));
}
double dist::transformDistance(robowflex::RobotPose r1, robowflex::RobotPose r2, double a)
{
    double tr = trDistance(r1, r2);
    double rot = rotDistance(r1, r2);
    return a * tr + (1 - a) * rot;
};
double dist::geometryDistance(robowflex::GeometryPtr g1, robowflex::GeometryPtr g2)
{
    return (g1->getDimensions() - g2->getDimensions()).norm();
}

double dist::octoDistance(bool ***grid1, bool ***grid2, int size)
{
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            for (int k = 0; k < size; k++)
                if (grid1[i][j][k] != grid2[i][j][k])
                    return 1;
    return 0;
};
