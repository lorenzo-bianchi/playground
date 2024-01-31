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

  Line b1 = Line({{ 1.0, -1.0}, { 1.0,  1.0}});
  Line b2 = Line({{ 1.0,  1.0}, {-1.0,  1.0}});
  Line b3 = Line({{-1.0,  1.0}, {-1.0, -1.0}});
  Line b4 = Line({{-1.0, -1.0}, { 1.0, -1.0}});
  std::vector<Line> boundaries = {b1, b2, b3, b4};

  auto startTime = std::chrono::high_resolution_clock::now();
  // Code to be timed
  gen_vor = GeneralizedVoronoi();
  gen_vor.add_polygons(polys);
  gen_vor.add_boundaries(boundaries);
  Result vor_result;

  gen_vor.run(run_type::optimized, plot_voronoi_, vor_result);

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
  RCLCPP_INFO(this->get_logger(), "Total time: %ld ms", duration);

  // Save data on file
  if (save_log_)
  {
    this->save_log();
  }

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
