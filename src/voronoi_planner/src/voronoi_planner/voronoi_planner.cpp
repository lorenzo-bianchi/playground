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

  // Initialize TOPP-RA variables
  init_toppra();

  // Initialize static variables
  Line::point_distance = point_distance_;
  Triangle::distance_tresh = distance_tresh_;
  GeneralizedVoronoi::rdp_epsilon = rdp_epsilon_;

  // Test
  // Polygons polys = {
  //                   {{0.08, 0.08}, {0.08, 0.3}, {0.3, 0.3}, {0.3, 0.08}},
  //                   {{0.3, 0.5}, {0.35, 0.55}, {0.3, 0.6}, {0.45, 0.65}, {0.5, 0.5}, {0.4, 0.45}, {0.4, 0.4}},
  //                  };

  // Polygons polys = {
  //                   {{0.23943661971830987, 0.7295774647887324},
  //                     {0.33943661971830985, 0.7309859154929578},
  //                     {0.3380281690140845, 0.8352112676056338}},
  //                   {{0.23943661971830987, 0.7295774647887324},
  //                     {0.3380281690140845, 0.8352112676056338},
  //                     {0.23661971830985917, 0.8323943661971831}},
  //                   {{0.43239436619718313, 0.7211267605633803},
  //                     {0.5309859154929578, 0.7211267605633803},
  //                     {0.5309859154929578, 0.8267605633802817}},
  //                   {{0.43239436619718313, 0.7211267605633803},
  //                     {0.5309859154929578, 0.8267605633802817},
  //                     {0.4309859154929578, 0.8267605633802817}},
  //                   {{0.7056338028169015, 0.7183098591549296},
  //                     {0.7070422535211268, 0.8225352112676056},
  //                     {0.6028169014084508, 0.823943661971831}},
  //                   {{0.7056338028169015, 0.7183098591549296},
  //                     {0.6028169014084508, 0.823943661971831},
  //                     {0.6028169014084508, 0.719718309859155}},
  //                   {{0.3676056338028169, 0.3788732394366197},
  //                     {0.36619718309859156, 0.47605633802816905},
  //                     {0.26760563380281693, 0.4746478873239437}},
  //                   {{0.3676056338028169, 0.3788732394366197},
  //                     {0.26760563380281693, 0.4746478873239437},
  //                     {0.26760563380281693, 0.39014084507042257}},
  //                   {{0.3676056338028169, 0.3788732394366197},
  //                     {0.26760563380281693, 0.39014084507042257},
  //                     {0.27605633802816903, 0.3887323943661972}},
  //                   {{0.27605633802816903, 0.3887323943661972},
  //                     {0.27042253521126763, 0.38169014084507047},
  //                     {0.27605633802816903, 0.376056338028169}},
  //                   {{0.27605633802816903, 0.3887323943661972},
  //                     {0.27605633802816903, 0.376056338028169},
  //                     {0.2774647887323944, 0.38450704225352117}},
  //                   {{0.2774647887323944, 0.38450704225352117},
  //                     {0.3042253521126761, 0.376056338028169},
  //                     {0.3676056338028169, 0.3788732394366197}},
  //                   {{0.2774647887323944, 0.38450704225352117},
  //                     {0.3676056338028169, 0.3788732394366197},
  //                     {0.27605633802816903, 0.3887323943661972}},
  //                   {{0.5887323943661972, 0.24225352112676057},
  //                     {0.6901408450704226, 0.24366197183098592},
  //                     {0.6887323943661973, 0.34366197183098596}},
  //                   {{0.5887323943661972, 0.24225352112676057},
  //                     {0.6887323943661973, 0.34366197183098596},
  //                     {0.5859154929577465, 0.34366197183098596}},
  //                   {{0.4197183098591549, 0.23380281690140847},
  //                     {0.5197183098591549, 0.23380281690140847},
  //                     {0.5183098591549296, 0.3352112676056338}},
  //                   {{0.5183098591549296, 0.3352112676056338},
  //                     {0.4197183098591549, 0.33380281690140845},
  //                     {0.4253521126760564, 0.3183098591549296}},
  //                   {{0.4253521126760564, 0.3183098591549296},
  //                     {0.4197183098591549, 0.3183098591549296},
  //                     {0.4197183098591549, 0.23380281690140847}},
  //                   {{0.4197183098591549, 0.23380281690140847},
  //                     {0.5183098591549296, 0.3352112676056338},
  //                     {0.4253521126760564, 0.3183098591549296}},
  //                   {{0.23661971830985917, 0.223943661971831},
  //                     {0.34647887323943666, 0.2267605633802817},
  //                     {0.3450704225352113, 0.3323943661971831}},
  //                   {{0.23661971830985917, 0.223943661971831},
  //                     {0.3450704225352113, 0.3323943661971831},
  //                     {0.23661971830985917, 0.32816901408450705}}
  //                 };

  // Polygons polys = {
  //                   {{ 0.5,  0.0},
  //                    { 0.0,  0.5},
  //                    {-0.5,  0.0},
  //                    { 0.0, -0.5}},
  //                  };

  // Polygons polys = {
  //                   {{12.5, 0.0}, {15.0, 0.0}, {15.0, 1.5}, {12.5, 1.5}},
  //                   {{19.0, 1.0}, {17.0, 1.0}, {17.0, 3.0}, {19.0, 3.0}},
  //                   {{14.5, 8.0}, {19.0, 8.0}, {19.0, 9.0}, {14.5, 9.0}},
  //                   {{12.0, 3.0}, {12.0, 4.5}, {13.0, 4.5}, {13.0, 4.0}, {14.0, 4.0}, {14.0, 4.5}, {15.0, 4.5}, {15.0, 3.0}},
  //                   {{12.0, 5.5}, {13.0, 5.5}, {13.0, 6.0}, {14.0, 6.0}, {14.0, 5.5}, {15.0, 5.5}, {15.0, 6.5}, {12.0, 6.5}},
  //                   {{3.0, 3.0}, {4.0, 3.0}, {4.0, 4.0}, {3.0, 4.0}},
  //                   {{3.0, 5.0}, {4.0, 5.0}, {4.0, 6.0}, {3.0, 6.0}},
  //                   {{3.0, 7.0}, {4.0, 7.0}, {4.0, 8.0}, {3.0, 8.0}},
  //                   {{5.0, 3.0}, {6.0, 3.0}, {6.0, 4.0}, {5.0, 4.0}},
  //                   {{5.0, 5.0}, {6.0, 5.0}, {6.0, 6.0}, {5.0, 6.0}},
  //                   {{5.0, 7.0}, {6.0, 7.0}, {6.0, 8.0}, {5.0, 8.0}},
  //                   {{7.0, 3.0}, {8.0, 3.0}, {8.0, 4.0}, {7.0, 4.0}},
  //                   {{7.0, 5.0}, {8.0, 5.0}, {8.0, 6.0}, {7.0, 6.0}},
  //                   {{7.0, 7.0}, {8.0, 7.0}, {8.0, 8.0}, {7.0, 8.0}},
  //                  };

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distribution(5, 95);

  // Polygons polys;
  // int K = 10;
  // for (int i = 1; i < K; i++)
  // {
  //   // get random number between 1 and 100
  //   double x = distribution(gen) * grid_resolution_;
  //   double y = int(distribution(gen) / 2) * grid_resolution_;
  //   double size;
  //   if (i < K/4)
  //     size = 0.4;
  //   else if (i < K/2)
  //     size = 0.8;
  //   else if (i < 3*K/4)
  //     size = 1.2;
  //   else
  //     size = 2.0;
  //   // size = 0.2;
  //   // append to polys the square of size 0.2 with the bottom-left corner in (x, y)
  //   double x1 = x;
  //   double x2 = std::min(x + size, 20.0);
  //   double y1 = y;
  //   double y2 = std::min(y + size, 10.0);
  //   polys.push_back({{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}});
  // }
  // Line b1 = Line({{ 1.0, -1.0}, { 1.0,  1.0}});
  // Line b2 = Line({{ 1.0,  1.0}, {-1.0,  1.0}});
  // Line b3 = Line({{-1.0,  1.0}, {-1.0, -1.0}});
  // Line b4 = Line({{-1.0, -1.0}, { 1.0, -1.0}});
  // Line b1 = Line({{0.0, 0.0}, {1.0, 0.0}});
  // Line b2 = Line({{1.0, 0.0}, {1.0, 1.0}});
  // Line b3 = Line({{1.0, 1.0}, {0.0, 1.0}});
  // Line b4 = Line({{0.0, 0.0}, {0.0, 1.0}});

  // Create and populate occupancy grid
  std::vector<int> grid_size = {int(field_size_[1] / grid_resolution_ + 1),
                                int(field_size_[0] / grid_resolution_ + 1),
                                int(field_size_[2] / grid_resolution_)};

  OccupancyGrid3D grid3D;
  for (int i = 0; i < grid_size[2]; i++)
  {
    OccupancyGrid2D grid2D = OccupancyGrid2D(grid_size[0], grid_size[1]);
    grid2D.setZero();
    grid3D.push_back(grid2D);
  }

  int num_parallelepipeds = 10;

  std::vector<std::vector<int>> parallelepipeds;
  std::uniform_int_distribution<int> rand_x(10, grid_size[0] - 10);
  std::uniform_int_distribution<int> rand_y(0, grid_size[1] - 1);
  std::uniform_int_distribution<int> rand_width(2, grid_size[0] / 8);
  std::uniform_int_distribution<int> rand_height(2, grid_size[1] / 4);
  std::uniform_int_distribution<int> rand_depth(2, grid_size[2]);
  for (int i = 0; i < num_parallelepipeds; i++)
  {
    int x = rand_x(gen);
    int y = rand_y(gen);
    int z = 0;
    int width = rand_width(gen);
    int height = rand_height(gen);
    int depth = rand_depth(gen);

    // insert parallelepiped in grid3D
    for (int k = z; k < z + depth; k++)
    {
      for (int j = y; j < y + height; j++)
      {
        for (int i = x; i < x + width; i++)
          grid3D[k](i, j) = true;
      }
    }

  }
  ///////////////////////////////////////////////////////////

  //
  Line b1 = Line({{           0.0,            0.0}, {field_size_[0],            0.0}});
  Line b2 = Line({{field_size_[0],            0.0}, {field_size_[0], field_size_[1]}});
  Line b3 = Line({{field_size_[0], field_size_[1]}, {           0.0, field_size_[1]}});
  Line b4 = Line({{           0.0,            0.0}, {           0.0, field_size_[1]}});
  std::vector<Line> boundaries = {b1, b2, b3, b4};

  auto start_time = std::chrono::high_resolution_clock::now();

  // Code to be timed
  int lower_layer = 0;
  int n_layers = 4;

  results.r_lengths.push_back(0);
  results.v_lengths.push_back(0);

  // TODO: threads?
  for (int idx = lower_layer; idx < lower_layer + n_layers; idx++)
  {
    OccupancyGrid2D layer = grid3D[idx];

    Polygons polys;
    polys_from_grid(layer, polys);
    if (polys.size() < 2) continue;

    gen_vor = GeneralizedVoronoi();
    gen_vor.add_polygons(polys);
    gen_vor.add_boundaries(boundaries);

    // Compute Voronoi graph
    gen_vor.run(run_type::optimized, plot_voronoi_, idx * grid_resolution_, results);
  }
  results.r_lengths.push_back(results.ridges.size());

  //
  double thr = 0.2;
  int above_layers = 1;

  // print size of vertices
  std::cout << "results.vertices.size() pre: " << results.vertices.size() << std::endl;
  std::cout << "results.ridges.size() pre: " << results.ridges.size() << std::endl;
  if (results.altitudes.size() > 1)
  {
    for (size_t i = 0; i < results.altitudes.size()-1; i++)
    {
      for (size_t lower_idx = results.v_lengths[i]; lower_idx < results.v_lengths[i+1]; lower_idx++)
      {
        Point3D lower_vertex = results.vertices[lower_idx];
        int delta = std::min(1+above_layers, (int) (results.altitudes.size()-i));
        for (size_t upper_idx = results.v_lengths[i+1]; upper_idx < results.v_lengths[i+delta]; upper_idx++)
        {
          Point3D upper_vertex = results.vertices[upper_idx];

          Eigen::Vector3d hdist = (upper_vertex - lower_vertex).cwiseAbs();
          if (hdist[0] < thr && hdist[1] < thr)
            results.ridges.push_back({lower_idx, upper_idx});
        }
      }
    }
  }
  std::cout << "results.vertices.size() post: " << results.vertices.size() << std::endl;
  std::cout << "results.ridges.size() post: " << results.ridges.size() << std::endl;

  //////////////////////////////////////////

  auto contours_end_time = std::chrono::high_resolution_clock::now();



  auto voronoi_end_time = std::chrono::high_resolution_clock::now();

  // Run A* algorithm
  start = {robot_start_[0], robot_start_[1], robot_start_[2]};
  goal = {robot_goal_[0], robot_goal_[1], robot_goal_[2]};

  astar = Astar(results, start, goal);
  path = astar.run();

  // Filter out points too close
  size_t i = 0;
  while (i < path.size() - 1)
  {
    if ((path[i] - path[i+1]).norm() < points_tresh_)
      path.erase(path.begin() + i + 1);
    else
      i++;
  }

  auto astar_end_time = std::chrono::high_resolution_clock::now();

  // Move points
  path_orig = path;
  for (size_t i = 1; i < path.size()-1; i++)
  {
    Eigen::Vector3d a = path[i-1];
    Eigen::Vector3d b = path[i];
    Eigen::Vector3d c = path[i+1];

    Eigen::Vector3d ba = (a - b);
    Eigen::Vector3d bc = (c - b);

    Eigen::Vector3d bis = ba + bc;
    double norm_angle = bis.norm();
    bis = bis / norm_angle;

    path[i] = b + move_coefficient_ * norm_angle * bis;
  }

  // Run topp-ra
  toppra::Vectors positions;
  for (auto& vi : path)
  {
    toppra::Vector vi_eigen(vi.size());
    for (long int i = 0; i < vi.size(); i++) vi_eigen(i) = vi[i];
    positions.push_back(vi_eigen);
  }

  int N = positions.size();
  times.resize(N);
  for (int i = 0; i < N; i++) times[i] = i / double(N-1);

  toppra::BoundaryCond bc_start = toppra::BoundaryCond(spline_bc_order_, spline_bc_values_);
  toppra::BoundaryCond bc_end = toppra::BoundaryCond(spline_bc_order_, spline_bc_values_);
  //toppra::BoundaryCond bc = toppra::BoundaryCond("natural");  // "notaknot", "clamped", "natural", "manual
  toppra::BoundaryCondFull bc_type{bc_start, bc_end};

  toppra::PiecewisePolyPath spline = toppra::PiecewisePolyPath::CubicSpline(positions, times, bc_type);
  std::shared_ptr<toppra::PiecewisePolyPath> spline_ptr = std::make_shared<toppra::PiecewisePolyPath>(spline);

  times.resize(sample_points_);
  for (int i = 0; i < sample_points_; i++)
    times[i] = i / double(sample_points_-1);

  // Compute velocity profile
  toppra::algorithm::TOPPRA problem{constrains, spline_ptr};
  problem.computePathParametrization(0, 0);

  pd = problem.getParameterizationData();

  spline_path = std::make_shared<toppra::parametrizer::ConstAccel>(spline_ptr, pd.gridpoints, pd.parametrization);
  // spline_path = std::make_shared<toppra::parametrizer::Spline>(spline_ptr, pd.gridpoints, pd.parametrization);

  const toppra::Bound optimized_time_interval = spline_path->pathInterval();
  std::cout << "Optimized time: " << optimized_time_interval[1] << "s" << std::endl;
  time_breaks_optimized = toppra::Vector::LinSpaced(5 * sample_points_, optimized_time_interval(0), optimized_time_interval(1));

  q_nominal_optimized = spline_path->eval(time_breaks_optimized, 0);
  q_dot_nominal_optimized = spline_path->eval(time_breaks_optimized, 1);
  q_ddot_nominal_optimized = spline_path->eval(time_breaks_optimized, 2);

  std::vector<Eigen::VectorXd> q_nominal_optimized_mat;
  std::vector<Eigen::VectorXd> q_dot_nominal_optimized_mat;
  std::vector<double> time_breaks_optimized_vec;
  for (size_t i = 0; i < (size_t) 5 * sample_points_; i++)
  {
    const double t = time_breaks_optimized[i];
    const toppra::Vector& q_t = q_nominal_optimized[i];
    const toppra::Vector& q_dot_t = q_dot_nominal_optimized[i];
    q_nominal_optimized_mat.push_back(q_t);
    q_dot_nominal_optimized_mat.push_back(q_dot_t);
    time_breaks_optimized_vec.push_back(t);
  }

  // Check if computed velocity is correct
  Point3D pos = start;
  for (size_t i = 1; i < q_dot_nominal_optimized_mat.size(); i++)
  {
    pos += q_dot_nominal_optimized_mat[i] * (time_breaks_optimized[i] - time_breaks_optimized[i-1]);
  }
  std::cout << "Final position: " << pos.transpose() << std::endl;
  std::cout << "Error: " << goal.transpose() - pos.transpose() << std::endl;

  // // print q_nominal_optimized_mat
  // for (size_t i = 0; i < q_dot_nominal_optimized_mat.size(); i++)
  // {
  //   std::cout << "q_dot_nominal_optimized_mat[" << i << "] = " << q_dot_nominal_optimized_mat[i].transpose() << std::endl;
  // }






  // Sample spline
  pv = spline.eval(times, 0);
  dpv = spline.eval(times, 1);
  ddpv = spline.eval(times, 2);

  // Compute spline length
  length = spline_length(pv, sample_points_);

  // Compute spline curvature
  std::vector<double> curvature;
  spline_curvature(dpv, ddpv, sample_points_, curvature);

  // Find Voronoi regions
  // simple_cycles(vor_result);

  auto end_time = std::chrono::high_resolution_clock::now();

  /////////////////////////////////////////

  auto duration_contours = std::chrono::duration_cast<std::chrono::milliseconds>(contours_end_time - start_time).count();
  auto duration_voronoi = std::chrono::duration_cast<std::chrono::milliseconds>(voronoi_end_time - contours_end_time).count();
  auto duration_astar = std::chrono::duration_cast<std::chrono::milliseconds>(astar_end_time - voronoi_end_time).count();
  auto duration_spline = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - astar_end_time).count();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  RCLCPP_WARN(this->get_logger(), "Contours time: %ld ms", duration_contours);
  RCLCPP_WARN(this->get_logger(), "Voronoi time: %ld ms", duration_voronoi);
  RCLCPP_WARN(this->get_logger(), "A* time: %ld ms", duration_astar);
  RCLCPP_WARN(this->get_logger(), "Spline time: %ld ms", duration_spline);
  RCLCPP_WARN(this->get_logger(), "Total time: %ld ms", duration);

  // Plot
  if (plot_voronoi_)
  {
    if (results.altitudes.size() > 1) this->plot_voronoi_3d();
    else plot_voronoi_2d(0);
  }

  throw std::invalid_argument("Manually stopped after generate_plot()");

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

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_toppra()
{
  velLimitLower = -max_vel_ * toppra::Vector::Ones(3);
  velLimitUpper =  max_vel_ * toppra::Vector::Ones(3);
  accLimitLower = -max_acc_ * toppra::Vector::Ones(3);
  accLimitUpper =  max_acc_ * toppra::Vector::Ones(3);

  toppra::LinearConstraintPtr ljv, lja;
  ljv = std::make_shared<toppra::constraint::LinearJointVelocity>(velLimitLower, velLimitUpper);
  lja = std::make_shared<toppra::constraint::LinearJointAcceleration>(accLimitLower, accLimitUpper);

  constrains = toppra::LinearConstraintPtrs{ljv, lja};
}

} // namespace VoronoiPlanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoronoiPlanner::VoronoiPlannerNode)
