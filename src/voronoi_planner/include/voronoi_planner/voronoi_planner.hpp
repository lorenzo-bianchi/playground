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

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <Eigen/Dense>
#include <fcntl.h>
#include <iostream>
#include <ostream>
#include <unistd.h>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <fstream>

#include <dua_node/dua_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/Qhull.h"

#include "libqhull_r/libqhull_r.h"

#include "matplotlibcpp.h"

#define UNUSED(arg) (void)(arg)
#define EPSILON 1e-6
#define LINE std::cout << __FUNCTION__ << ", LINE: " << __LINE__ << std::endl;

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

namespace plt = matplotlibcpp;

typedef Eigen::Vector2d Point;
typedef std::vector<Point> Polygon;
typedef std::vector<Polygon> Polygons;
typedef int NodeT;
typedef std::vector<NodeT> Chain;
typedef std::vector<Chain> Chains;
typedef Eigen::Matrix<NodeT, 2, 1> ChainIdx;
typedef std::deque<ChainIdx> ChainStart;
typedef Eigen::Matrix<NodeT, 2, 1> RidgeVertex;
typedef std::vector<RidgeVertex> RidgeVertices;
typedef std::unordered_map<NodeT, Chain> DictT;
typedef std::vector<Point> VertexChain;
typedef std::vector<VertexChain> VertexChains;

namespace VoronoiPlanner
{

enum run_type {
  // non_lined = 0,
  // non_deleted = 1,
  non_optimized = 2,
  optimized = 3
};

class Line
{
public:
  Line();
  Line(std::vector<Point> inputPoints);
  std::vector<Point> generate_line();
  bool is_intersecting_class(Line& otherLine);
  std::vector<Point> get_points() { return points; }
  void set_point_distance(double distance) { point_distance = distance; }

protected:
  double point_distance;
  std::vector<Point> points;

  std::vector<Point> generate_line_base(Point& point1, Point& point2);
  std::vector<Point> generate_line_base(Point& point1, Point& point2, int axis);
  double distance(Point& point1, Point& point2);
};

class Triangle : public Line
{
public:
  Triangle(std::vector<Point> inputPoints);
  std::vector<Point> generate_line();
  bool is_in_polygon(Point& point);
  void set_distance_tresh(double distance) { distance_tresh = distance; }

private:
  double distance_tresh;
  bool test_distance_tresh(std::vector<Point>& points, Point& test_point, double distance_trash);
  bool test_point_convex(std::vector<Point>& points, Point& test_point);
};

struct Result
{
  std::vector<Triangle> triangles;
  std::vector<Line> boundaries;
  std::vector<Point> points;
  std::vector<Point> points_polygon;
  VertexChain vertices;
  RidgeVertices ridge_vertices;
  Chains chains;
};

class IndexDict
{
public:
  IndexDict(RidgeVertices& vec);
  void insert(NodeT key, Chain& value);
  bool contains(NodeT key);
  Chain find(NodeT key);
  std::vector<std::pair<NodeT, Chain>> items();

private:
  DictT dict;

  bool contains(DictT& dict, NodeT key);
  DictT generate(RidgeVertices& vec, bool reverse);
};

class Qhull
{
public:
  Qhull(std::string flags, std::vector<Point> points);
  ~Qhull();
  RidgeVertices ridge_vertices;
  std::vector<Point> get_points();
  void get_voronoi_diagram(VertexChain& vor_vertices,
                           std::vector<Eigen::Matrix<NodeT, 2, 1>>& vor_ridge_points,
                           RidgeVertices& vor_ridge_vertices,
                           Chains& vor_regions,
                           std::vector<NodeT>& vor_point_region);
  int get_nridges() { return nridges; }
  void set_nridges(int n) { nridges = n; }
  std::vector<Eigen::Matrix<NodeT, 2, 1>> get_ridge_points() { return ridge_points; }
  std::vector<Eigen::Matrix<NodeT, 2, 1>> ridge_points;

private:
  qhT* qh;
  int ndim;
  int numpoints;
  std::vector<std::vector<Point>> point_arrays;
  int nridges;

  void check_active();
  void close();
};

class Voronoi
{
public:
  Voronoi() {}
  Voronoi(std::vector<Point> points);

  qhT* qh;

  std::vector<Triangle> triangles;
  std::vector<Line> boundaries;
  std::vector<Point> points_polygon;
  Chains chains;

  VertexChain vertices;
  std::vector<Eigen::Matrix<NodeT, 2, 1>> ridge_points;
  RidgeVertices ridge_vertices;
  Chains regions;
  std::vector<NodeT> point_region;
  std::vector<Point> points;
  int ndim;
  int npoints;
  Eigen::Vector2d min_bound;
  Eigen::Vector2d max_bound;

private:
};

class GeneralizedVoronoi
{
public:
  GeneralizedVoronoi();
  void add_point(Point& point);
  void add_points(std::vector<Point>& new_points);
  void add_line(Line& line);
  void add_lines(std::vector<Line>& new_lines);
  void add_triangle(Triangle& triangle);
  void add_triangles(std::vector<Triangle>& new_triangles);
  void add_boundary(Line& boundary);
  void add_boundaries(std::vector<Line>& new_boundaries);
  void add_polygon(Polygon& polygon);
  void add_polygons(Polygons& polygons);
  bool optimize_line();
  void run(run_type type, bool plot, Result& result);
  void run_non_optimized(const bool generate_result, Result& result);
  void run_optimized(Result& result);
    void generate_plot();
  // get functions
  std::vector<Point> get_points() { return points; }
  std::vector<Line> get_lines() { return lines; }
  std::vector<Triangle> get_triangles() { return triangles; }
  std::vector<Line> get_boundaries() { return boundaries; }
  std::vector<Point> get_triangle_points() { return triangle_points; }
  std::vector<Point> get_boundary_points() { return boundary_points; }
  std::vector<Point> get_line_points() { return line_points; }
  std::vector<Point> get_triangle_lined_points() { return triangle_lined_points; }
  std::vector<Point> get_boundary_lined_points() { return boundary_lined_points; }
  std::vector<Point> get_line_lined_points() { return line_lined_points; }
  Chains get_chains() { return chains; }
  Voronoi get_vor() { return vor; }

  // rdp_epsilon set function
  void set_rdp_epsilon(float epsilon) { rdp_epsilon = epsilon; }

private:
  float rdp_epsilon;

  std::vector<Point> points;
  std::vector<Line> lines;
  std::vector<Triangle> triangles;
  std::vector<Line> boundaries;

  std::vector<Point> triangle_points;
  std::vector<Point> boundary_points;
  std::vector<Point> line_points;

  std::vector<Point> triangle_lined_points;
  std::vector<Point> boundary_lined_points;
  std::vector<Point> line_lined_points;

  Chains chains;

  Voronoi vor;

  void run_voronoi(std::vector<Point>& points);
  void generate_result(Result& result);
  Chains generate_chains();
  Chain generate_chain(IndexDict& dict, ChainStart& start, Chain& feature);
  ChainStart chain(IndexDict& dict, ChainIdx& idx, Chain& chain, Chain& feature);
  VertexChains generate_vertex_chains(Chains& chains);
  VertexChains optimize_line_base(VertexChains& chains);
  void regenerate_voronoi(VertexChains& chains);

  void delete_unfinished();
  Chain unfinished_vertices();
  Chain ridges_to_delete(Chain& vertex_vec);
  void delete_vertex(Chain& to_delete);
  void delete_ridge(Chain& to_delete);
  void reorganize_ridge(Chain& deleted_vertices);
  Chain vertices_in_polygon();
};

///////////////////////////////////

// Geometry
int counter_clockwise(Point& point1, Point& point2, Point& point3);
double distance_between_line_point(std::vector<Point>& line, Point& point);
int find_closest(Chain vec, NodeT elem);
bool is_intersecting(std::vector<Point>& line1, std::vector<Point>& line2);
double radian(Eigen::Vector2d& v1, Eigen::Vector2d& v2);
double total_distance(std::vector<Point>& path);
std::vector<Triangle> triangulation(Polygon& polygon);

// RDP
double perpendicular_distance(Point& pt, Point& lineStart, Point& lineEnd);
void ramer_douglas_peucker(VertexChain& pointList, double epsilon, VertexChain& out);

// Tricpp
double calculate_total_area(std::vector<Triangle>& triangles);
bool contains_no_points(Point& p1, Point& p2, Point& p3, Polygon& polygon);
void earclip(Polygon& polygon, std::vector<Triangle>& triangles);
bool is_clockwise(Polygon& polygon);
bool is_convex(Point& prev, Point& point, Point& next);
bool is_ear(Point& p1, Point& p2, Point& p3, Polygon& polygon);
bool is_point_inside(Point& p, Point& a, Point& b, Point& c);
double triangle_area(Point& p1, Point& p2, Point& p3);
double triangle_sum(double x1, double y1, double x2, double y2, double x3, double y3);

// Qhull
void visit_voronoi(qhT* _qh, FILE* ptr, vertexT* vertex, vertexT* vertexA, setT* centers, boolT unbounded);
void qh_order_vertexneighbors_nd(qhT* qh, int nd, vertexT* vertex);
int  qh_new_qhull_scipy(qhT* qh, int dim, int numpoints, coordT* points, boolT ismalloc,
                        char* qhull_cmd, FILE* outfile, FILE* errfile, coordT* feaspoint);

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
  double distance_tresh_;
  double point_distance_;
  double rdp_epsilon_;

  /* Synchronization primitives for internal update operations */
  std::atomic<bool> stop_thread;

  /* Node init functions */
  void init_atomics();
  void init_parameters();
  void init_publishers();

  /* Internal state variables */
  std::vector<std::vector<std::vector<double>>> polygons;

};

} // namespace VoronoiPlanner

#endif // VORONOI_PLANNER_HPP