/**
 * VoronoiPlanner module auxiliary routines.
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
  double total_distance(std::vector<Point>& path)
  {
    double dist = 0.0;
    for (long unsigned int i = 0; i < path.size() - 1; i++)
    {
      dist += (path[i] - path[i + 1]).norm();
    }
    return dist;
  }

  /*  */
  double distance_between_line_point(
    std::vector<Point>& line,
    Point& point)
  {
    Eigen::Vector2d v1 = line[1] - line[0];
    Eigen::Vector2d v2 = point - line[0];
    double l1 = v1.norm();
    double l2 = v2.norm();

    double dot = v1.dot(v2);
    if (dot > l1 * l1 || dot < 0.0)
    {
      return std::min(l2, (point - line[1]).norm());
    }

    double angle = acos(dot / (l1 * l2));
    return sin(angle) * l2;
  }
  // TODO
  // def min_distance_from_obstacle(vor):
  //   min_dist = np.finfo(np.float64).max
  //   vertices = vor.vertices
  //   ridges = vor.ridge_vertices
  //   points = vor.points_polygon

  //   for ridge in ridges:
  //       _l = np.array([vertices[ridge[0]], vertices[ridge[1]]])
  //       for point in points:
  //           _p = np.array(point)
  //           _d = distance_between_line_point(_l, _p)
  //           if _d < min_dist:
  //               min_dist = _d

  //   return min_dist

  /*  */
  double radian(Eigen::Vector2d& v1, Eigen::Vector2d& v2)
  {
    v1.normalize();
    v2.normalize();
    double dot = v1.dot(v2);
    if (dot > 1.0) dot = 1.0;
    return acos(dot);
  }

  /*  */
  template<typename T>
  int find_closest(std::vector<T> vec, T elem)
  {
    for (int i = 0; i < vec.size(); i++)
    {
      if (vec[i] >= elem) return i;
    }
  }

  /*  */
  int counter_clockwise(
    Point& point1,
    Point& point2,
    Point& point3)
  {
    Eigen::Vector2d v1 = point2 - point1;
    Eigen::Vector2d v2 = point3 - point1;

    // Compute cross product between v1 and v2
    double cross = v1[0] * v2[1] - v1[1] * v2[0];
    if (cross > 0.0) return 1;
    else if (abs(cross) < 0.00001) return -1;

    return -1;
  }

  bool isIntersecting(
    std::vector<Point>& line1,
    std::vector<Point>& line2)
  {
    bool ccw1 = counter_clockwise(line1[0], line1[1], line2[0]);
    bool ccw2 = counter_clockwise(line1[0], line1[1], line2[1]);

    if (ccw1 * ccw2 < 0.0)
    {
      ccw1 = counter_clockwise(line2[0], line2[1], line1[0]);
      ccw2 = counter_clockwise(line2[0], line2[1], line2[1]);

      if (ccw1 * ccw2 < 0.0)
      {
        return true;
      }
    }
    return false;
  }

  ///////////////////////////////
  // Line class implementation //
  ///////////////////////////////
  /*  */
  Line::Line(std::vector<Point> inputPoints) : point_distance(0.0), points(inputPoints)
  {
    if (points.size() != 2) throw std::invalid_argument("Line must have 2 points.");
  }

  /*  */
  std::vector<Point> Line::generateLine()
  {
    return generateLineBase(points[0], points[1]);
  }

  /*  */
  std::vector<Point> Line::generateLineBase(
    Point& point1,
    Point& point2)
  {
    std::vector<Point> result;

    if (point1[0] != point2[0])
    {
      result = generateLineBase(point1, point2, 0);
    }
    else if (point1[1] != point2[1])
    {
      result = generateLineBase(point1, point2, 1);
    }
    else
    {
      std::cout << "Identical point has been detected: [" << point1[0] << ", " << point1[1] << "]" << std::endl;
    }

    return result;
  }

  /*  */
  std::vector<Point> Line::generateLineBase(
    Point& point1,
    Point& point2,
    int axis)
  {
    std::vector<Point> result;

    if (point1[axis] > point2[axis])
    {
      Point temp = point1;
      point1 = point2;
      point2 = temp;
    }

    double dis = distance(point1, point2);
    Eigen::Vector2d stepVec = (point2 - point1) / dis;
    for (double current = point1[axis]; current <= point2[axis]; current += point_distance)
    {
      result.push_back(point1 + current * stepVec);
    }

    return result;
  }

  /*  */
  bool Line::isIntersectingClass(Line& otherLine)
  {
    return isIntersecting(points, otherLine.points);
  }

  double Line::distance(Point& point1, Point& point2)
  {
    return (point1 - point2).norm();
  }
  //////////////////////////////////

  ///////////////////////////////////
  // Triangle class implementation //
  ///////////////////////////////////
  Triangle::Triangle(std::vector<Point> inputPoints) : Line(inputPoints), distance_tresh(0.0)
  {
    if (points.size() != 3) throw std::invalid_argument("Triangle must have 3 points.");
  }

  /*  */
  std::vector<Point> Triangle::generateLine()
  {
    std::vector<Point> result;

    // loop for every line in Triangle
    for (long unsigned int i = 0; i < points.size(); i++)
    {
      Point p1 = points[i];
      Point p2 = points[i + 1 < points.size() ? i + 1 : 0];
      auto v = generateLineBase(p1, p2);
      result.insert(result.end(), v.begin(), v.end());
    }

    return result;
  }

  /*  */
  bool Triangle::is_in_polygon(Point& point)
  {
    // if point is close enough, filter out
    if (test_distance_tresh(points, point, distance_tresh))
    {
      return true;
    }

    return test_point_convex(points, point);
  }

  /*  */
  bool Triangle::test_distance_tresh(
    std::vector<Point>& points,
    Point& test_point,
    double distance_trash)
  {
    for (long unsigned int i = 0; i < points.size(); i++)
    {
      double dist = (test_point - points[i]).norm();
      if (dist < distance_trash)
      {
        return true;
      }
    }
    return false;
  }

  /*  */
  bool Triangle::test_point_convex(std::vector<Point>& points, Point& test_point)
  {
    for (long unsigned int i = 0; i < 3; i++)
    {
      bool ccw1 = counter_clockwise(points[i], points[(i + 1) % 3], test_point);
      bool ccw2 = counter_clockwise(points[i], test_point, points[(i + 2) % 3]);
      if (ccw1 * ccw2 <= 0.0)
      {
        return false;
      }
    }
    return true;
  }

} // namespace VoronoiPlanner
