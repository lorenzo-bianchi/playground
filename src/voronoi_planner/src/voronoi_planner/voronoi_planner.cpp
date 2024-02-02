/**
 * VoronoiPlanner node initialization and parameters routines.
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

double Line::point_distance = 0.0;
double Triangle::distance_tresh = 0.0;
float GeneralizedVoronoi::rdp_epsilon = 0.0;

/**
 * @brief VoronoiPlanner node constructor.
 *
 * @param node_opts Options for the base node.
 */
VoronoiPlannerNode::VoronoiPlannerNode(const rclcpp::NodeOptions & node_options)
: NodeBase("voronoi_planner", node_options, true)
{
  // Initialize parameters
  init_parameters();

  // Initialize atomic members
  init_atomics();

  // Initialize topic publishers
  init_publishers();

  // Initialize static variables
  Line::point_distance = point_distance_;
  Triangle::distance_tresh = distance_tresh_;
  GeneralizedVoronoi::rdp_epsilon = rdp_epsilon_;

  // Test
  // Polygons polys = {
  //                   {{0.08, 0.08}, {0.08, 0.3}, {0.3, 0.3}, {0.3, 0.08}},
  //                   {{0.3, 0.5}, {0.35, 0.55}, {0.3, 0.6}, {0.45, 0.65}, {0.5, 0.5}, {0.4, 0.45}, {0.4, 0.4}},
  //                  };

  Polygons polys = {
                    {{0.23943661971830987, 0.7295774647887324},
                      {0.33943661971830985, 0.7309859154929578},
                      {0.3380281690140845, 0.8352112676056338}},
                    {{0.23943661971830987, 0.7295774647887324},
                      {0.3380281690140845, 0.8352112676056338},
                      {0.23661971830985917, 0.8323943661971831}},
                    {{0.43239436619718313, 0.7211267605633803},
                      {0.5309859154929578, 0.7211267605633803},
                      {0.5309859154929578, 0.8267605633802817}},
                    {{0.43239436619718313, 0.7211267605633803},
                      {0.5309859154929578, 0.8267605633802817},
                      {0.4309859154929578, 0.8267605633802817}},
                    {{0.7056338028169015, 0.7183098591549296},
                      {0.7070422535211268, 0.8225352112676056},
                      {0.6028169014084508, 0.823943661971831}},
                    {{0.7056338028169015, 0.7183098591549296},
                      {0.6028169014084508, 0.823943661971831},
                      {0.6028169014084508, 0.719718309859155}},
                    {{0.3676056338028169, 0.3788732394366197},
                      {0.36619718309859156, 0.47605633802816905},
                      {0.26760563380281693, 0.4746478873239437}},
                    {{0.3676056338028169, 0.3788732394366197},
                      {0.26760563380281693, 0.4746478873239437},
                      {0.26760563380281693, 0.39014084507042257}},
                    {{0.3676056338028169, 0.3788732394366197},
                      {0.26760563380281693, 0.39014084507042257},
                      {0.27605633802816903, 0.3887323943661972}},
                    {{0.27605633802816903, 0.3887323943661972},
                      {0.27042253521126763, 0.38169014084507047},
                      {0.27605633802816903, 0.376056338028169}},
                    {{0.27605633802816903, 0.3887323943661972},
                      {0.27605633802816903, 0.376056338028169},
                      {0.2774647887323944, 0.38450704225352117}},
                    {{0.2774647887323944, 0.38450704225352117},
                      {0.3042253521126761, 0.376056338028169},
                      {0.3676056338028169, 0.3788732394366197}},
                    {{0.2774647887323944, 0.38450704225352117},
                      {0.3676056338028169, 0.3788732394366197},
                      {0.27605633802816903, 0.3887323943661972}},
                    {{0.5887323943661972, 0.24225352112676057},
                      {0.6901408450704226, 0.24366197183098592},
                      {0.6887323943661973, 0.34366197183098596}},
                    {{0.5887323943661972, 0.24225352112676057},
                      {0.6887323943661973, 0.34366197183098596},
                      {0.5859154929577465, 0.34366197183098596}},
                    {{0.4197183098591549, 0.23380281690140847},
                      {0.5197183098591549, 0.23380281690140847},
                      {0.5183098591549296, 0.3352112676056338}},
                    {{0.5183098591549296, 0.3352112676056338},
                      {0.4197183098591549, 0.33380281690140845},
                      {0.4253521126760564, 0.3183098591549296}},
                    {{0.4253521126760564, 0.3183098591549296},
                      {0.4197183098591549, 0.3183098591549296},
                      {0.4197183098591549, 0.23380281690140847}},
                    {{0.4197183098591549, 0.23380281690140847},
                      {0.5183098591549296, 0.3352112676056338},
                      {0.4253521126760564, 0.3183098591549296}},
                    {{0.23661971830985917, 0.223943661971831},
                      {0.34647887323943666, 0.2267605633802817},
                      {0.3450704225352113, 0.3323943661971831}},
                    {{0.23661971830985917, 0.223943661971831},
                      {0.3450704225352113, 0.3323943661971831},
                      {0.23661971830985917, 0.32816901408450705}}
                  };

  // Polygons polys = {
  //                   {{ 0.5,  0.0},
  //                    { 0.0,  0.5},
  //                    {-0.5,  0.0},
  //                    { 0.0, -0.5}},
  //                  };

  // Line b1 = Line({{ 1.0, -1.0}, { 1.0,  1.0}});
  // Line b2 = Line({{ 1.0,  1.0}, {-1.0,  1.0}});
  // Line b3 = Line({{-1.0,  1.0}, {-1.0, -1.0}});
  // Line b4 = Line({{-1.0, -1.0}, { 1.0, -1.0}});

  Line b1 = Line({{0.0, 0.0}, {1.0, 0.0}});
  Line b2 = Line({{1.0, 0.0}, {1.0, 1.0}});
  Line b3 = Line({{1.0, 1.0}, {0.0, 1.0}});
  Line b4 = Line({{0.0, 0.0}, {0.0, 1.0}});

  std::vector<Line> boundaries = {b1, b2, b3, b4};

  auto startTime = std::chrono::high_resolution_clock::now();
  // Code to be timed
  gen_vor = GeneralizedVoronoi();
  gen_vor.add_polygons(polys);
  gen_vor.add_boundaries(boundaries);
  Result vor_result;

  // Compute Voronoi graph
  gen_vor.run(run_type::optimized, plot_voronoi_, vor_result);

  // Run A* algorithm
  Point start = {0.05, 0.05};
  Point end = {0.9, 0.9};

  Astar astar = Astar(vor_result, start, end);
  std::vector<Point> path = astar.run();
  if (plot_voronoi_) astar.generate_plot();

  // Add z component to path points
  std::vector<Eigen::Vector3d> path3d;
  path3d.resize(path.size());
  for (size_t i = 0; i < path.size(); i++)
  {
    path3d[i] = {path[i][0], path[i][1], 0.0};
  }

  // Filter out points too close
  size_t i = 0;
  while (i < path3d.size() - 1)
  {
    if ((path3d[i] - path3d[i+1]).norm() < points_tresh_)
      path3d.erase(path3d.begin() + i + 1);
    else
      i++;
  }

  // Move points
  auto path3d_orig = path3d;
  for (size_t i = 1; i < path3d.size()-1; i++)
  {
    Eigen::Vector3d a = path3d[i-1];
    Eigen::Vector3d b = path3d[i];
    Eigen::Vector3d c = path3d[i+1];

    Eigen::Vector3d ba = (a - b);
    Eigen::Vector3d bc = (c - b);

    Eigen::Vector3d bis = ba + bc;
    double norm_angle = bis.norm();
    bis = bis / norm_angle;

    path3d[i] = b + move_coefficient_ * norm_angle * bis;
  }

  // Run topp-ra
  toppra::Vectors positions;
  for (auto vi : path3d)
  {
    toppra::Vector vi_eigen(vi.size());
    for (long int i = 0; i < vi.size(); i++) vi_eigen(i) = vi[i];
    positions.push_back(vi_eigen);
  }

  int N = positions.size();
  toppra::Vector times;
  times.resize(N);
  for (int i = 0; i < N; i++) times[i] = i / double(N-1);

  toppra::BoundaryCond bc_start = toppra::BoundaryCond(spline_bc_order_, spline_bc_values_);
  toppra::BoundaryCond bc_end = toppra::BoundaryCond(spline_bc_order_, spline_bc_values_);
  //toppra::BoundaryCond bc = toppra::BoundaryCond("natural");  // "notaknot", "clamped", "natural", "manual
  toppra::BoundaryCondFull bc_type{bc_start, bc_end};

  toppra::PiecewisePolyPath spline = toppra::PiecewisePolyPath::CubicSpline(positions, times, bc_type);

  times.resize(sample_points_);
  for (int i = 0; i < sample_points_; i++)
    times[i] = i / double(sample_points_-1);

  // Compute useful quantities
  toppra::Vectors pv = spline.eval(times, 0);
  toppra::Vectors dpv = spline.eval(times, 1);
  toppra::Vectors ddpv = spline.eval(times, 2);

  // Spline length
  double length = 0.0;
  for (int i = 0; i < sample_points_; ++i)
  {
    Eigen::Vector3d p1 = pv[i];
    Eigen::Vector3d p2 = pv[i+1];

    length += (p2 - p1).norm();
  }

  // Spline curvature
  std::vector<double> curvature;
  curvature.resize(sample_points_);
  double num, den;
  for (int i = 0; i < sample_points_; ++i)
  {
    Eigen::Vector3d dp = dpv[i];
    Eigen::Vector3d ddp = ddpv[i];

    num = dp.cross(ddp).norm();
    den = std::pow(dp.norm(), 3);
    if (den < 1e-6) den = 1e-6;
    curvature[i] = num / den;
  }

  /////////////////////////////////////////

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
  RCLCPP_INFO(this->get_logger(), "Total time: %ld ms", duration);

  // Plot
  plt::figure_size(720, 720);

  std::vector<double> X, Y;
  for (auto vector : vor_result.points)
  {
    X.push_back(vector[0]);
    Y.push_back(vector[1]);
  }
  plt::plot(X, Y, "b.");

  // X.clear(); Y.clear();
  // for (auto vector : vor_result.vertices)
  // {
  //   X.push_back(vector[0]);
  //   Y.push_back(vector[1]);
  // }
  // plt::plot(X, Y, "yo");

  X.clear(); Y.clear();
  for (size_t i = 0; i <  vor_result.ridge_vertices.size(); i++)
  {
    RidgeVertex simplex = vor_result.ridge_vertices[i];

    Point p1 = vor_result.vertices[simplex[0]];
    Point p2 = vor_result.vertices[simplex[1]];

    X = {p1[0], p2[0]};
    Y = {p1[1], p2[1]};
    plt::plot(X, Y, "m--");
  }

  X.clear(); Y.clear();
  for (size_t i = 0; i < path3d_orig.size()-1; i++)
  {
    X = {path3d_orig[i][0], path3d_orig[i+1][0]};
    Y = {path3d_orig[i][1], path3d_orig[i+1][1]};
    plt::plot(X, Y, "yo");
    plt::plot(X, Y, "r");
  }

  // X.clear(); Y.clear();
  // plt::plot({start[0]}, {start[1]}, "k*");
  // plt::plot({end[0]}, {end[1]}, "k*");
  // for (size_t i = 0; i < path3d.size()-1; i++)
  // {
  //   X = {path3d[i][0], path3d[i+1][0]};
  //   Y = {path3d[i][1], path3d[i+1][1]};
  //   plt::plot(X, Y, "r");
  // }

  X.clear(); Y.clear();
  for (size_t i = 0; i < pv.size()-1; i++)
  {
    X = {pv[i][0], pv[i+1][0]};
    Y = {pv[i][1], pv[i+1][1]};
    plt::plot(X, Y, "g");
  }

  plt::legend();
  plt::set_aspect_equal();
  plt::show();

  throw std::invalid_argument("Error in generate_plot()");

  // Save data on file
  if (save_log_) this->save_log();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief VoronoiPlanner node destructor.
 */
VoronoiPlannerNode::~VoronoiPlannerNode()
{
  RCLCPP_INFO(this->get_logger(), "Node destroyed");
}

/**
 * @brief Routine to initialize atomic members.
 */
void VoronoiPlannerNode::init_atomics()
{
}

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_publishers()
{
  // VoronoiPlanner
  // joy_pub_ = this->create_publisher<voronoi_planner_msgs::msg::VoronoiPlanner>(
  //   joy_topic_name,
  //   rclcpp::QoS(1));
}
} // namespace VoronoiPlanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoronoiPlanner::VoronoiPlannerNode)
