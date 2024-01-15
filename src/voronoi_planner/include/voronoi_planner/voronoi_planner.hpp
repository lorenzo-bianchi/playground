/**
 * VoronoiPlanner headers.
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

#ifndef VORONOI_PLANNER_HPP
#define VORONOI_PLANNER_HPP

#include <atomic>
#include <cfloat>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>

#include <dua_node/dua_node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <vector>

#include <Eigen/Dense>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/Qhull.h"

#include "libqhull_r/libqhull_r.h"

using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetList;
using orgQhull::QhullQh;
using orgQhull::RboxPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;

using namespace std::chrono_literals;
using namespace rcl_interfaces::msg;

typedef Eigen::Vector2d Point;
typedef std::vector<Point> Polygon;
typedef std::vector<Polygon> Polygons;

#define UNUSED(arg) (void)(arg)

#define EPSILON 1e-6

#define LINE std::cout<<__LINE__<<std::endl;

namespace VoronoiPlanner
{

class Line
{
public:
  Line();
  Line(std::vector<Point> inputPoints);
  std::vector<Point> generateLine();
  bool isIntersectingClass(Line& otherLine);

protected:
  double point_distance;
  std::vector<Point> points;

  std::vector<Point> generateLineBase(Point& point1, Point& point2);
  std::vector<Point> generateLineBase(Point& point1, Point& point2, int axis);
  double distance(Point& point1, Point& point2);
};

class Triangle : public Line
{
public:
  Triangle(std::vector<Point> inputPoints);
  std::vector<Point> generateLine();
  std::vector<Point> get_points() { return points; }

private:
  double distance_tresh;

  bool is_in_polygon(Point& point);
  bool test_distance_tresh(std::vector<Point>& points, Point& test_point, double distance_trash);
  bool test_point_convex(std::vector<Point>& points, Point& test_point);
};

// Geometry
bool isIntersecting(std::vector<Point>& line1, std::vector<Point>& line2);
int counter_clockwise(Point& point1, Point& point2, Point& point3);
template<typename T>
int find_closest(std::vector<T> vec, T elem);
double radian(Eigen::Vector2d& v1, Eigen::Vector2d& v2);
double distance_between_line_point(std::vector<Point>& line, Point& point);
double total_distance(std::vector<Point>& path);

//RDP
double PerpendicularDistance(Point &pt, Point &lineStart, Point &lineEnd);
void RamerDouglasPeucker(std::vector<Point> &pointList, double epsilon, std::vector<Point> &out);

// Tricpp
bool isClockwise(Polygon& polygon);
double triangleSum(double x1, double y1, double x2, double y2, double x3, double y3);
bool isConvex(Point& prev, Point& point, Point& next);
double triangleArea(Point& p1, Point& p2, Point& p3);
bool isPointInside(Point& p, Point& a, Point& b, Point& c);
bool containsNoPoints(Point& p1, Point& p2, Point& p3, Polygon& polygon);
bool isEar(Point& p1, Point& p2, Point& p3, Polygon& polygon);
void earclip(Polygon& polygon, std::vector<Triangle>& triangles);
double calculateTotalArea(std::vector<Triangle>& triangles);

/**
 * Convert messages and transform data
 */
class VoronoiPlannerNode : public DUANode::NodeBase
{
public:
  VoronoiPlannerNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~VoronoiPlannerNode();

private:
  /* Publishers */
  // rclcpp::Publisher<voronoi_planner_msgs::msg::VoronoiPlanner>::SharedPtr joy_pub_;

  /* Utility routines */

  /* Node parameters */
  double axis_max_val;
  int64_t axis_deadzone_val;
  std::string joy_topic_name;
  std::string joy_path;

  /* Synchronization primitives for internal update operations */
  std::atomic<bool> stop_thread;

  /* Node init functions */
  void init_atomics();
  void init_parameters();
  void init_publishers();

  /* Internal state variables */
  std::vector<std::vector<std::vector<double>>> polygons;

  int dim;                  /* dimension of points */
  int numpoints;            /* number of points */
  coordT *points;           /* array of coordinates for each point */
  coordT *feaspoint;
  boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */
  std::string flags;        /* option flags for qhull, see html/qh-quick.htm */
  FILE *outfile = stdout;   /* output from qh_produce_output
                              use NULL to skip qh_produce_output */
  FILE *errfile = stderr;   /* error messages from qhull code */
  int exitcode;             /* 0 if no error from qhull */
  facetT *facet;            /* set by FORALLfacets */
  int curlong, totlong;     /* memory remaining after qh_memfreeshort */

  qhT qh_qh;                /* Qhull's data structure.  First argument of most calls */
  qhT *qh= &qh_qh;          /* Alternatively -- qhT *qh= (qhT *)malloc(sizeof(qhT)) */

};

} // namespace VoronoiPlanner

#endif // VORONOI_PLANNER_HPP
