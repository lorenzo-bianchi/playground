/**
 * VoronoiPlanner RamerDouglasPeucker functions.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * January 13, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <voronoi_planner/voronoi_planner.hpp>

namespace VoronoiPlanner
{
  /*  */
  double PerpendicularDistance(Point &pt, Point &lineStart, Point &lineEnd)
  {
    Eigen::Vector2d dx = lineEnd - lineStart;
    dx.normalize();

    Eigen::Vector2d pv = pt - lineStart;

    // Get dot product
    double pvdot = dx.dot(pv);

    // Scale line direction vector
    Eigen::Vector2d ds = pvdot * dx;

    // Subtract from pv
    Eigen::Vector2d a = pv - ds;

    return a.norm();
  }

  /*  */
  void RamerDouglasPeucker(std::vector<Point> &pointList,
                           double epsilon,
                           std::vector<Point> &out)
  {
    if (pointList.size() < 2) throw std::invalid_argument("Not enough points to simplify");

    // Find the point with maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for (size_t i = 1; i < end; i++)
    {
      double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
      if (d > dmax)
      {
        index = i;
        dmax = d;
      }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon)
    {
      // Recursive call
      std::vector<Point> recResults1;
      std::vector<Point> recResults2;
      std::vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
      std::vector<Point> lastLine(pointList.begin()+index, pointList.end());
      RamerDouglasPeucker(firstLine, epsilon, recResults1);
      RamerDouglasPeucker(lastLine, epsilon, recResults2);

      // Build the result list
      out.assign(recResults1.begin(), recResults1.end()-1);
      out.insert(out.end(), recResults2.begin(), recResults2.end());
      if (out.size() < 2) throw std::runtime_error("Problem assembling output");
    }
    else
    {
      // Return start and end points
      out.clear();
      out.push_back(pointList[0]);
      out.push_back(pointList[end]);
    }
  }
} // namespace VoronoiPlanner
