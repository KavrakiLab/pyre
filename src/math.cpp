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

/* Author: Constantinos Chamzas, Zachary Kingston*/

#include <iostream>
#include <iomanip>

#include <ros/console.h>
#include <pyre/math.h>

using namespace pyre;

Eigen::Isometry2d tf::toIsometry(double x, double y, double theta)
{
    Eigen::Isometry2d iso;
    iso.translation() = Eigen::Vector2d(x, y);
    iso.linear() = Eigen::Rotation2D<double>(theta).toRotationMatrix();

    return iso;
}

Eigen::Isometry2d tf::toIsometry(const Eigen::Vector3d &vec)
{
    return toIsometry(vec[0], vec[1], vec[2]);
}

double tf::getRotation(const Eigen::Isometry2d &frame)
{
    return Eigen::Rotation2D<double>(frame.linear()).angle();
}

Eigen::Vector3d tf::flattenIsometry(const Eigen::Isometry2d &frame)
{
    const auto &t = frame.translation();
    return {t[0], t[1], getRotation(frame)};
}
double tf::transformDistance(Eigen::Isometry2d t1, Eigen::Isometry2d t2)
{
    auto d = (t1.translation() - t2.translation()).norm();
    d += math::angleDistance(getRotation(t1), getRotation(t2));
    return d;
}

std::string tf::printFrame(const Eigen::Isometry2d &frame)
{
    const auto &v = flattenIsometry(frame);
    return printVector(v);
}

std::string tf::printVector(const Eigen::VectorXd &v, unsigned int precision)
{
    std::stringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(precision);

    ss << "[ ";
    for (unsigned int i = 0; i < v.size(); ++i)
    {
        ss << std::setw(precision + 4) << v[i];
        if (i < v.size() - 1)
            ss << ", ";
    }
    ss << " ]";

    return ss.str();
}

std::string tf::printVector(const std::vector<double> &x, unsigned int precision)
{
    auto v = math::toEigen(x);
    return printVector(v, precision);
}

std::string tf::printMatrix(const Eigen::MatrixXd &v, unsigned int precision)
{
    std::stringstream ss;
    ss.setf(std::ios::fixed, std::ios::floatfield);
    ss.precision(precision);

    for (unsigned int j = 0; j < v.rows(); ++j)
    {
        ss << ((j) ? "  " : "[ ");
        for (unsigned int i = 0; i < v.cols(); ++i)
        {
            ss << std::setw(precision + 4) << v(j, i);
            if (i < v.cols() - 1)
                ss << ", ";
        }

        if (j < v.rows() - 1)
            ss << std::endl;
        else
            ss << " ]";
    }

    return ss.str();
}

Eigen::VectorXd math::toEigen(double x)
{
    return Eigen::VectorXd::Constant(1, x);
}

Eigen::VectorXd math::toEigen(std::initializer_list<double> x)
{
    Eigen::VectorXd r(x.size());

    auto it = x.begin();
    for (unsigned int i = 0; i < x.size(); ++i)
        r[i] = *it++;

    return r;
}

Eigen::Vector3d math::toEigen(geometry_msgs::Point p)
{
    return Eigen::Vector3d{p.x, p.y, p.z};
}

Eigen::VectorXd math::toEigen(const std::vector<double> &x)
{
    Eigen::VectorXd r(x.size());

    auto it = x.begin();
    for (unsigned int i = 0; i < x.size(); ++i)
        r[i] = *it++;

    return r;
}

std::vector<double> math::toVector(const Eigen::MatrixXd &x)
{
    std::vector<double> vec(x.data(), x.data() + x.rows() * x.cols());

    return vec;
}

Eigen::MatrixXd math::toMatrix(std::vector<Eigen::VectorXd> mat)
{
    Eigen::MatrixXd M(mat.size(), mat[0].size());

    for (unsigned int i = 0; i < M.rows(); i++)
        M.row(i) = mat[i];

    return M;
}

double math::remap(double a1, double a2, double av, double b1, double b2)
{
    double dat = fabs(a2 - a1);
    double dav = fabs(av - a1);

    double dbt = fabs(b2 - b1);
    double dbv = dbt * (dav / dat);

    double bv = b1 + dbv;
    return bv;
}

double math::angleDistance(double v1, double v2)
{
    double v = fabs(v1 - v2);
    return (v > math::pi) ? math::pipi - v : v;
}

double math::uniformReal(double lower, double upper)
{
    if (lower > upper)
        ROS_ERROR("Lower bound must be lower than upper bound");

    return rng.uniformReal(lower, upper);
}

double math::gaussian(double mean, double std)
{
    return rng.gaussian(mean, std);
}
