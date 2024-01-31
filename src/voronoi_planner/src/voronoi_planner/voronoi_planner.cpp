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

  Polygons polys = {
                    {{ 0.5,  0.0},
                     { 0.0,  0.5},
                     {-0.5,  0.0},
                     { 0.0, -0.5}},
                   };

  Line b1 = Line({{ 1.0, -1.0}, { 1.0,  1.0}});
  Line b2 = Line({{ 1.0,  1.0}, {-1.0,  1.0}});
  Line b3 = Line({{-1.0,  1.0}, {-1.0, -1.0}});
  Line b4 = Line({{-1.0, -1.0}, { 1.0, -1.0}});
  std::vector<Line> boundaries = {b1, b2, b3, b4};

  auto startTime = std::chrono::high_resolution_clock::now();
  // Code to be timed
  GeneralizedVoronoi gen_vor = GeneralizedVoronoi();
  gen_vor.add_polygons(polys);
  gen_vor.add_boundaries(boundaries);
  Result vor_result;

  gen_vor.run(run_type::optimized, plot_voronoi_, vor_result);

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
  std::cout << "Total time: " << duration << " ms" << std::endl;

  // Save data on file
  auto timestamp = std::chrono::system_clock::now();
  auto timestampInSeconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count();
  std::string filename = "/home/neo/workspace/logs/output_" + std::to_string(timestampInSeconds) + ".txt";
  std::ofstream outputFile(filename);

  // get points
  auto points = gen_vor.get_points();
  outputFile << "Points:" << std::endl;
  outputFile << points.size() << std::endl;
  for (auto & p : points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Points" << std::endl << std::endl;

  // get lines
  auto pts = gen_vor.get_lines();
  outputFile << "Lines:" << std::endl;
  outputFile << pts.size() << std::endl;
  for (auto & p : pts)
  {
    auto t = p.get_points();
    outputFile << t[0][0] << " " << t[0][1] << "\t"
               << t[1][0] << " " << t[1][1] << std::endl;
  }
  outputFile << "End Lines" << std::endl << std::endl;

  // get triangles
  auto tris = gen_vor.get_triangles();
  outputFile << "Triangles:" << std::endl;
  outputFile << tris.size() << std::endl;
  for (auto & t : tris)
  {
    auto points = t.get_points();
    outputFile << points[0][0] << " " << points[0][1] << "\t"
               << points[1][0] << " " << points[1][1] << "\t"
               << points[2][0] << " " << points[2][1] << std::endl;
  }
  outputFile << "End Triangles" << std::endl << std::endl;

  // get boundaries
  auto bnds = gen_vor.get_boundaries();
  outputFile << "Boundaries:" << std::endl;
  outputFile << bnds.size() << std::endl;
  for (auto & b : bnds)
  {
    auto points = b.get_points();
    outputFile << points[0][0] << " " << points[0][1] << "\t"
               << points[1][0] << " " << points[1][1] << std::endl;
  }
  outputFile << "End Boundaries" << std::endl << std::endl;

  // get triangle points
  auto tri_pts = gen_vor.get_triangle_points();
  outputFile << "Triangle points" << std::endl;
  outputFile << tri_pts.size() << std::endl;
  for (auto & p : tri_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Triangle points" << std::endl << std::endl;

  // get boundary points
  auto bnd_pts = gen_vor.get_boundary_points();
  outputFile << "Boundary points" << std::endl;
  outputFile << bnd_pts.size() << std::endl;
  for (auto & p : bnd_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Boundary points" << std::endl << std::endl;

  // get line points
  auto line_pts = gen_vor.get_line_points();
  outputFile << "Line points" << std::endl;
  outputFile << line_pts.size() << std::endl;
  for (auto & p : line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Line points" << std::endl << std::endl;

  // get triangle lined points
  auto tri_line_pts = gen_vor.get_triangle_lined_points();
  outputFile << "Triangle lined points" << std::endl;
  outputFile << tri_line_pts.size() << std::endl;
  for (auto & p : tri_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Triangle lined points" << std::endl << std::endl;

  //get boundary lined points
  auto bnd_line_pts = gen_vor.get_boundary_lined_points();
  outputFile << "Boundary lined points" << std::endl;
  outputFile << bnd_line_pts.size() << std::endl;
  for (auto & p : bnd_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Boundary line points" << std::endl << std::endl;

  // get line lined points
  auto line_line_pts = gen_vor.get_line_lined_points();
  outputFile << "Line lined points" << std::endl;
  outputFile << line_line_pts.size() << std::endl;
  for (auto & p : line_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Line lined points" << std::endl << std::endl;

  // get chains
  auto chains = gen_vor.get_chains();
  outputFile << "Chains" << std::endl;
  outputFile << chains.size() << std::endl;
  for (auto & c : chains)
  {
    for (auto & p : c)
    {
      outputFile << p << ", ";
    }
    outputFile << std::endl;
  }
  outputFile << "End Chains" << std::endl << std::endl;

  // get vor
  auto vor = gen_vor.get_vor();

  // get vor.vertices
  outputFile << "Vor.vertices" << std::endl;
  outputFile << vor.vertices.size() << std::endl;
  for (auto & p : vor.vertices)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.vertices" << std::endl << std::endl;

  // get vor.ridge_points
  outputFile << "Vor.ridge_points" << std::endl;
  outputFile << vor.ridge_points.size() << std::endl;
  for (auto & p : vor.ridge_points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.ridge_points" << std::endl << std::endl;

  // get vor.ridge_vertices
  outputFile << "Vor.ridge_vertices" << std::endl;
  outputFile << vor.ridge_vertices.size() << std::endl;
  for (auto & p : vor.ridge_vertices)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.ridge_vertices" << std::endl << std::endl;

  // get vor.regions
  outputFile << "Vor.regions" << std::endl;
  outputFile << vor.regions.size() << std::endl;
  for (Chain p : vor.regions)
  {
    for (NodeT q : p)
    {
      outputFile << q << " ";
    }
    outputFile << std::endl;
  }
  outputFile << "End Vor.regions" << std::endl << std::endl;

  // get vor.point_region
  outputFile << "Vor.point_region" << std::endl;
  outputFile << vor.point_region.size() << std::endl;
  for (auto & p : vor.point_region)
  {
    outputFile << p << std::endl;
  }
  outputFile << "End Vor.point_region" << std::endl << std::endl;

  // get vor.points
  outputFile << "Vor.points" << std::endl;
  outputFile << vor.points.size() << std::endl;
  for (auto & p : vor.points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.points" << std::endl << std::endl;

  outputFile.close();

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
