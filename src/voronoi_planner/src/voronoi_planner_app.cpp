/**
 * VoronoiPlanner standalone application.
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

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <voronoi_planner/voronoi_planner.hpp>

using namespace DUAAppManagement;

int main(int argc, char ** argv)
{
  ROS2AppManager<rclcpp::executors::MultiThreadedExecutor,
    VoronoiPlanner::VoronoiPlannerNode> app_manager(
    argc,
    argv,
    "voronoi_planner_app");

  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "voronoi_planner_app_signal_handler",
    app_manager.get_executor());
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.ignore(SIGHUP);
  sig_handler.ignore(SIGUSR1);
  sig_handler.ignore(SIGUSR2);

  app_manager.run();

  app_manager.shutdown();
  sig_handler.fini();

  exit(EXIT_SUCCESS);
}
