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

  // Initialize callback groups
  init_cgroups();

  // Initialize topic publishers
  init_publishers();

  // Initialize timers
  //init_timers();

  // Initialize actions
  init_actions();

  // Initialize TOPP-RA variables
  init_toppra();

  // Initialize static variables
  init_static_vars();

  //////////////////////////////////////////

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
  // std::random_device rd;
  // std::mt19937 gen(rd());
  //std::uniform_int_distribution<> distribution(5, 95);
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

  for (int i = 0; i < grid_size[2]; i++)
  {
    OccupancyGrid2D grid2D = OccupancyGrid2D(grid_size[0], grid_size[1]);
    grid2D.setZero();
    grid3D.push_back(grid2D);
  }

  std::mt19937 gen(seed_);
  std::uniform_int_distribution<int> rand_x(10, grid_size[1] - 10);
  std::uniform_int_distribution<int> rand_y(0, grid_size[0] - 1);
  std::uniform_int_distribution<int> rand_width(2, grid_size[1] / 8);
  std::uniform_int_distribution<int> rand_height(2, grid_size[0] / 4);
  std::uniform_int_distribution<int> rand_depth(2, grid_size[2]);

  int num_parallelepipeds = 30;
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
      for (int i = y; i < std::min(y + height, grid_size[0]-1); i++)
      {
        for (int j = x; j < std::min(x + width, grid_size[1]-10); j++)
          grid3D[k](i, j) = true;
      }
    }
  }

  // // delete all previous markers
  //visualization_msgs::msg::MarkerArray marker_array;
  // visualization_msgs::msg::Marker marker;
  // marker.action = visualization_msgs::msg::Marker::DELETEALL;
  // marker_array.markers.push_back(marker);
  // marker_pub_->publish(marker_array);

  // publish marker array with obstacles
  // clear marker_array
  // marker_array.markers.clear();
  // for (size_t i = 0; i < grid3D.size(); i++)
  // {
  //   for (size_t j = 0; j < (size_t) grid3D[i].rows(); j++)
  //   {
  //     for (size_t k = 0; k < (size_t) grid3D[i].cols(); k++)
  //     {
  //       if (grid3D[i](j, k))
  //       {
  //         visualization_msgs::msg::Marker marker;
  //         marker.header.frame_id = "map";
  //         marker.header.stamp = this->now();
  //         marker.ns = "obstacles";
  //         marker.id = i * grid3D[i].rows() * grid3D[i].cols() + j * grid3D[i].cols() + k;
  //         marker.type = visualization_msgs::msg::Marker::CUBE;
  //         marker.action = visualization_msgs::msg::Marker::ADD;
  //         marker.pose.position.x = k * grid_resolution_;
  //         marker.pose.position.y = j * grid_resolution_;
  //         marker.pose.position.z = i * grid_resolution_;
  //         marker.pose.orientation.x = 0.0;
  //         marker.pose.orientation.y = 0.0;
  //         marker.pose.orientation.z = 0.0;
  //         marker.pose.orientation.w = 1.0;
  //         marker.scale.x = grid_resolution_;
  //         marker.scale.y = grid_resolution_;
  //         marker.scale.z = grid_resolution_;
  //         marker.color.a = 1.0;
  //         marker.color.r = 0.0;
  //         marker.color.g = 0.0;
  //         marker.color.b = 1.0;
  //         marker_array.markers.push_back(marker);
  //       }
  //     }
  //   }
  // }
  // marker_pub_->publish(marker_array);

  ///////////////////////////////////////////////////////////

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
 * @brief Routine to initialize callback groups.
 */
void VoronoiPlannerNode::init_cgroups()
{
  // Actions
  actions_cgroup_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_publishers()
{
  // Marker array
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/test/obstacles",
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize static variables.
 */
void VoronoiPlannerNode::init_static_vars()
{
  Line::point_distance = point_distance_;
  Triangle::distance_tresh = distance_tresh_;
  GeneralizedVoronoi::rdp_epsilon = rdp_epsilon_voronoi_;
}

/**
 * @brief Routine to initialize timers.
 */
void VoronoiPlannerNode::init_timers()
{
  visualization_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(5.0),
    std::bind(
        &VoronoiPlannerNode::visualization_timer_clbk,
        this));
}

/**
 * @brief Routine to initialize actions.
 */
void VoronoiPlannerNode::init_actions()
{
  // Initialize action servers
  // Voronoi Planner
  find_path_server_ = rclcpp_action::create_server<FindPath>(
    this,
    "~/voro_planner",
    std::bind(
      &VoronoiPlannerNode::find_path_handle_goal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &VoronoiPlannerNode::find_path_handle_cancel,
      this,
      std::placeholders::_1),
    std::bind(
      &VoronoiPlannerNode::find_path_handle_accepted,
      this,
      std::placeholders::_1),
    DUAQoS::get_action_server_options(),
    actions_cgroup_);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_toppra()
{
  toppra::Vector vel_limit_lower = -max_vel_ * toppra::Vector::Ones(3);
  toppra::Vector vel_limit_upper =  max_vel_ * toppra::Vector::Ones(3);
  toppra::Vector acc_limit_lower = -max_acc_ * toppra::Vector::Ones(3);
  toppra::Vector acc_limit_upper =  max_acc_ * toppra::Vector::Ones(3);

  toppra::LinearConstraintPtr ljv, lja;
  ljv = std::make_shared<toppra::constraint::LinearJointVelocity>(vel_limit_lower, vel_limit_upper);
  lja = std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_limit_lower, acc_limit_upper);

  constraints = toppra::LinearConstraintPtrs{ljv, lja};
}

} // namespace VoronoiPlanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoronoiPlanner::VoronoiPlannerNode)
