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
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <queue>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <dua_node/dua_node.hpp>
#include <opencv2/opencv.hpp>
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

#include <toppra/algorithm.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#include <toppra/solver/seidel.hpp>
#include <toppra/toppra.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>

#include "matplotlibcpp.h"

#include <random>

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

typedef std::vector<int> Chain;
typedef std::vector<Chain> Chains;
typedef Eigen::Vector2i ChainIdx;
typedef std::deque<ChainIdx> ChainStart;
typedef Eigen::Vector2d Point;
typedef std::vector<Point> Polygon;
typedef std::vector<Polygon> Polygons;
typedef Eigen::Vector2i RidgeVertex;
typedef std::vector<RidgeVertex> RidgeVertices;
typedef std::vector<Point> VertexChain;
typedef std::vector<VertexChain> VertexChains;
typedef std::map<int, Chain> Dict;
typedef Eigen::Matrix<bool, -1, -1> OccupancyGrid;

namespace VoronoiPlanner
{

enum run_type
{
  non_optimized = 0,
  optimized = 1
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

  static double point_distance;

protected:
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

  static double distance_tresh;

private:
  bool test_distance_tresh(std::vector<Point>& points, Point& test_point, double distance_trash);
  bool test_point_convex(std::vector<Point>& points, Point& test_point);
};

struct Result
{
  std::vector<Triangle> triangles;
  std::vector<Line> boundaries;
  std::vector<Point> points;
  // std::vector<Point> points_polygon;
  VertexChain vertices;
  RidgeVertices ridge_vertices;
  // Chains chains;
};

class IndexDict
{
public:
  IndexDict() {}
  void insert(int key, Chain& value);
  IndexDict(RidgeVertices& vec);
  bool contains(int key);
  Chain find(int key);
  std::vector<std::pair<int, Chain>> items();

private:
  Dict dict;

  bool contains(Dict& dict, int key);
  Dict generate(RidgeVertices& vec, bool reverse);
};

class Qhull
{
public:
  Qhull(std::string flags, std::vector<Point> points);
  ~Qhull();
  std::vector<Point> get_points();
  void get_voronoi_diagram(VertexChain& vor_vertices,
                           std::vector<Eigen::Vector2i>& vor_ridge_points,
                           RidgeVertices& vor_ridge_vertices,
                           Chains& vor_regions,
                           std::vector<int>& vor_point_region);
  int get_nridges() { return nridges; }
  void set_nridges(int n) { nridges = n; }

  RidgeVertices ridge_vertices;
  std::vector<Eigen::Vector2i> ridge_points;

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
  std::vector<Eigen::Vector2i> ridge_points;
  RidgeVertices ridge_vertices;
  Chains regions;
  std::vector<int> point_region;
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
  void run_non_optimized(bool generate_result, Result& result);
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

  static float rdp_epsilon;

private:
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

  std::vector<std::pair<Point, Polygon>> polygons;

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
  void delete_vertex(Chain to_delete);
  void delete_ridge(Chain to_delete);
  void reorganize_ridge(Chain& deleted_vertices);
  Chain vertices_in_polygon();
};

class Astar
{
public:
  Astar() {}
  Astar(Result vor, Point start, Point end);
  void set_result(std::vector<Point> result) { this->result = result; }
  std::vector<Point> get_result() { return result; }

  class Node
  {
  public:
    Node() {}
    Node(int idx, Node* parent, double g, double h);
    int get_idx() { return idx; }
    Node* get_parent() { return parent; }
    double get_g() { return g; }
    double get_h() { return h; }
    double get_f() { return f; }
    void set_idx(int idx) { this->idx = idx; }
    void set_parent(Node* parent) { this->parent = parent; }
    void set_g(double g) { this->g = g; }
    void set_h(double h) { this->h = h; }
    void set_f(double f) { this->f = f; }

    bool operator<(const Node& other) const
    {
      return f < other.f;
    }

  private:
    int idx;
    Node* parent;
    double g;
    double h;
    double f;
  };

  std::vector<Point> run();
  void generate_plot();

private:
  std::vector<Point> result;
  Result vor;
  IndexDict dict;
  int start;
  int end;

  Node* astar();
  double heuristic(int idx);
  Node* generate_node(int idx, Node* current);
  bool is_goal(int idx);
  int add_ridge(Point point);
  std::vector<int> find_adjacent(Point point);
};

///////////////////////////////////

// Geometry
int counter_clockwise(Point& point1, Point& point2, Point& point3);
double distance_between_line_point(std::vector<Point>& line, Point& point);
int find_closest(Chain vec, int elem);
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
 * @brief VoronoiPlanner node class.
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
  void plot_voronoi();
  void save_log();
  void polys_from_grid(OccupancyGrid grid, Polygons &polygons);
  double spline_length(toppra::Vectors s, int64_t sample_points);
  void spline_curvature(toppra::Vectors ds,
                        toppra::Vectors dds,
                        int64_t sample_points,
                        std::vector<double> &curvature);
  void simple_cycles(Result vor_result);

  /* Node parameters */
  double distance_tresh_;
  std::vector<double> field_size_;
  double grid_resolution_;
  double line_increase_;
  double move_coefficient_;
  std::vector<int64_t> plot_size_;
  bool plot_voronoi_;
  double point_distance_;
  double points_tresh_;
  double rdp_epsilon_;
  std::vector<double> robot_goal_;
  std::vector<double> robot_start_;
  int64_t sample_points_;
  bool save_log_;
  int64_t spline_bc_order_;
  std::vector<double> spline_bc_values_;

  /* Synchronization primitives for internal update operations */
  std::atomic<bool> stop_thread;

  /* Node init functions */
  void init_atomics();
  void init_parameters();
  void init_publishers();

  /* Internal state variables */
  std::vector<std::vector<std::vector<double>>> polygons;
  GeneralizedVoronoi gen_vor;
  Result vor_result;
  std::vector<Point> path;
  std::vector<Eigen::Vector3d> path3d;
  std::vector<Eigen::Vector3d> path3d_orig;
  toppra::Vectors pv;
  toppra::Vectors dpv;
  toppra::Vectors ddpv;
  toppra::Vector times;
  std::vector<double> curvature;
  double length;
  Point start;
  Point goal;
  Astar astar;
};

} // namespace VoronoiPlanner

#endif // VORONOI_PLANNER_HPP