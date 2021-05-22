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

#ifndef PYRE_MATH_
#define PYRE_MATH_

#include <type_traits>
#include <vector>
#include <map>

#include <boost/math/constants/constants.hpp>

#include <ompl/util/RandomNumbers.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometry_msgs/Point.h>

namespace pyre
{
    static ompl::RNG rng;
    /** \brief Helper functions and constants require for mathematical operations.
     */
    namespace math
    {
        static const double pi = boost::math::constants::pi<double>();
        static const double pipi = 2. * pi;
        static const double pi2 = pi / 2.;

        static const double eps = std::numeric_limits<double>::epsilon();
        static const double inf = std::numeric_limits<double>::infinity();

        /** \brief Converts a single double value into a 1 x 1 matrix.
         *  \param[in] x Value to convert.
         *  \return a 1-dimensional vector with x as the value.
         */
        Eigen::VectorXd toEigen(double x);

        /** \brief Converts a vector of initialize list into a Matrix
         *  \param[in] M vector  to convert.
         *  \return a matrix with the values of the vector list.
         */
        Eigen::MatrixXd toMatrix(std::vector<Eigen::VectorXd> M);

        /** \brief Converts an initialize list into a vector
         *  \param[in] x Values to convert.
         *  \return a vector with the values of the list.
         */
        Eigen::VectorXd toEigen(std::initializer_list<double> x);

        /** \brief Converts an initialize list into a vector
         *  \param[in] x Values to convert.
         *  \return a vector with the values of the list.
         */
        Eigen::Vector3d toEigen(geometry_msgs::Point p);

        /** \brief Converts a double vector to an Eigen Vector
         *  \param[in] x Values to convert.
         *  \return a vector with the values of the list.
         */
        Eigen::VectorXd toEigen(const std::vector<double> &x);

        /** \brief Converts an  Eigen Vector to a doulbe vector
         *  \param[in] x Values to convert.
         *  \return a vector with the values of the list.
         */
        std::vector<double> toVector(const Eigen::MatrixXd &x);

        /** \brief Remap a value \e av in the interval \e a1, \e a2 to the interval \e b1, \e b2.
         *  \param[in] a1 Lower end of first interval.
         *  \param[in] a2 Upper end of first interval.
         *  \param[in] av Value in first interval.
         *  \param[in] b1 Lower end of second interval.
         *  \param[in] b2 Upper end of second interval.
         *  \return The remapped value in the second interval.
         */
        double remap(double a1, double a2, double av, double b1, double b2);

        /** \brief Return the minimum distance between two angles (wrapping from -pi to pi).
         *  \param[in] v1 First angle.
         *  \param[in] v2 Second angle.
         *  \return The distance between angles.
         */
        double angleDistance(double v1, double v2);

        /** \brief Return a uniform random number between lower and upper.
         *  \param[in] lower Lower bound
         *  \param[in] upper Upper bound
         *  \return random number between lower and upper
         */
        double uniformReal(double lower, double upper);

        /** \brief Return a random number sampled from a gaussain with given std and mean .
         *  \param[in] mean the mean of the gaussian
         *  \param[in] std the standard deviation of the gaussian
         *  \return random number sampled from gaussian
         */
        double gaussian(double mean, double std);

    }  // namespace math

    /** \brief Helper functions for transform manipulation.
     */
    namespace tf
    {
        /** \brief A template for a vector that stores Eigen objects.
         *  \tparam V Value for the vector (contains an Eigen type).
         */
        template <typename V>
        using EigenVector = std::vector<V, Eigen::aligned_allocator<V>>;

        /** \brief A template for a map that stores Eigen objects.
         *  \tparam K Key for the map (not an Eigen type).
         *  \tparam V Value for the map (contains an Eigen type).
         */
        template <typename K, typename V>
        using EigenMap = std::map<K, V,          //
                                  std::less<K>,  //
                                  Eigen::aligned_allocator<std::pair<const K, V>>>;

        /** \name Eigen Isometry2d operations
            \{ */

        /** \brief Converts a translation (x, y) and rotation from X-axis (theta) into a transform.
         *  \param[in] x X coordinate of translation.
         *  \param[in] y Y coordinate of translation.
         *  \param[in] theta Rotation from the X-axis.
         *  \return Transform corresponding to input.
         */
        Eigen::Isometry2d toIsometry(double x, double y, double theta);

        /** \brief Gets rotation from a frame from the X-axis (t)
         *  \param[in] frame Frame to get rotation from.
         *  \return Angle of transformation.
         */
        double getRotation(const Eigen::Isometry2d &frame);

        /** \brief Converts a vector [x, y, t] composed of a translation (x, y) and rotation from X-axis (t)
         * into a transform.
         *  \param[in] vec Vector representation of transform.
         *  \return Transform corresponding to input.
         */
        Eigen::Isometry2d toIsometry(const Eigen::Vector3d &vec);

        /** \brief Return the transform distance between two transformations (wrapping from -pi to pi).
         *  \param[in] t1 First transform.
         *  \param[in] t2 Second transform.
         *  \return The distance between the transforms.
         */

        double transformDistance(Eigen::Isometry2d t1, Eigen::Isometry2d t2);

        /** \brief Converts a transformation \e frame into a vector [x, y, t] composed of a translation (x, y)
         * and rotation from X-axis (t).
         *  \param[in] frame Transform to flatten.
         *  \return Vector representation of transform.
         */

        Eigen::Vector3d flattenIsometry(const Eigen::Isometry2d &frame);

        /** \} */

        /** \name Debug & Output
            \{ */

        /** \brief Returns a string "[x, y, t]" of the translation (x, y) and rotation (t) component of a
         * transform.
         *  \param[in] frame Frame to print.
         *  \return Printable string of transform data.
         */
        std::string printFrame(const Eigen::Isometry2d &frame);

        /** \brief Returns a string "[x, y, t]" of the translation (x, y) and rotation (t) component of a
         * transform.
         *  \param[in] frame Frame to print.
         *  \return Printable string of transform data.
         */
        std::string printVector(const Eigen::VectorXd &v, unsigned int precision = 4);

        /** \brief Returns a string "[x, y, t]" of the translation (x, y) and rotation (t) component of a
         * transform.
         *  \param[in] frame Frame to print.
         *  \return Printable string of transform data.
         */

        std::string printVector(const std::vector<double> &v, unsigned int precision = 4);

        /** \brief Returns a string of a matrix's contents.
         *  \param[in] v Matrix to print.
         *  \param[in] precision Precision to print floating point values at.
         *  \return Printable string of vector.
         */
        std::string printMatrix(const Eigen::MatrixXd &v, unsigned int precision = 4);

        /** \} */
    };  // namespace tf
};      // namespace pyre

#endif
