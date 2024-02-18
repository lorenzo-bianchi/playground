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
double VoronoiPlannerNode::spline_length(toppra::Vectors s, int64_t sample_points)
{
  double length = 0.0;
  for (int i = 0; i < sample_points-1; ++i)
  {
    Eigen::Vector3d p1 = s[i];
    Eigen::Vector3d p2 = s[i+1];

    length += (p2 - p1).norm();
  }
  return length;
}

/*  */
void VoronoiPlannerNode::spline_curvature(toppra::Vectors ds,
                                          toppra::Vectors dds,
                                          int64_t sample_points,
                                          std::vector<double> &curvature)
{
  curvature.resize(sample_points);
  double num, den;
  for (int i = 0; i < sample_points; ++i)
  {
    Eigen::Vector3d dp = ds[i];
    Eigen::Vector3d ddp = dds[i];

    num = dp.cross(ddp).norm();
    den = std::pow(dp.norm(), 3);
    if (den < 1e-6) den = 1e-6;
    curvature[i] = num / den;
  }
}

/*  */
void VoronoiPlannerNode::simple_cycles(Result vor_result)
{
  // size_t n_nodes = 21;
  // vor_result.ridge_vertices = {{0, 1}, {1, 2}, {2, 3}, {3, 4}, {8, 9}, {9, 10}, {10, 11}, {11, 12}, {16, 17}, {17, 18}, {18, 19}, {19, 20}, {0, 5}, {5, 8}, {8, 13}, {13, 16}, {2, 6}, {6, 10}, {10, 14}, {14, 18}, {4, 7}, {7, 12}, {12, 15}, {15, 20}};

  size_t n_nodes = vor_result.vertices.size();

  std::vector<std::vector<int>> adj(n_nodes);
  for (const Eigen::Vector2i& edge : vor_result.ridge_vertices)
  {
    adj[edge[0]].push_back(edge[1]);
    adj[edge[1]].push_back(edge[0]);
  }

  std::vector<int> nodes;
  for (size_t i = 0; i < n_nodes; i++)
  {
    nodes.push_back(i);
  }

  while (!nodes.empty())
  {
    int starting_node = nodes[0];

    if (adj[starting_node].size() < 2)
    {
      for (const int& child : adj[starting_node])
      {
        adj[starting_node].erase(std::remove(adj[starting_node].begin(), adj[starting_node].end(), child), adj[starting_node].end());
        adj[child].erase(std::remove(adj[child].begin(), adj[child].end(), starting_node), adj[child].end());
      }
      nodes.erase(std::remove(nodes.begin(), nodes.end(), starting_node), nodes.end());
      continue;
    }

    int next_node = adj[starting_node][0];

    std::vector<bool> visited(n_nodes, false);

    // find_simple_cycle
    std::vector<int> simple_cycle;
    std::vector<int> parents = std::vector<int>(n_nodes, 0);

    std::vector<int> q;
    q.push_back(starting_node);
    bool ok = true;

    while (!q.empty())
    {
      int node = q[0];
      q.erase(q.begin());
      visited[node] = true;

      for (const int& child : adj[node])
      {
        if (node == starting_node and child == next_node) continue;

        if (visited[child] == false)
        {
          parents[child] = node;

          if (child == next_node)
          {
            ok = false;
            break;
          }

          q.push_back(child);
          visited[child] = true;
        }
      }

      if (!ok) break;
    }

    simple_cycle.push_back(starting_node);
    int x = next_node;

    while (x != starting_node)
    {
      simple_cycle.push_back(x);
      x = parents[x];
    }

    //print simple_cycles
    std::cout << "Simple cycle: ";
    for (const int& node : simple_cycle)
    {
      std::cout << node << " ";
    }
    std::cout << std::endl;

    //

    std::vector<int> neighbours = adj[starting_node];
    for (const int& child : neighbours)
    {
      adj[starting_node].erase(std::remove(adj[starting_node].begin(), adj[starting_node].end(), child), adj[starting_node].end());
      adj[child].erase(std::remove(adj[child].begin(), adj[child].end(), starting_node), adj[child].end());
    }

    nodes.erase(std::remove(nodes.begin(), nodes.end(), starting_node), nodes.end());
  }
}

/*  */
void VoronoiPlannerNode::polys_from_grid(OccupancyGrid grid, Polygons &polygons)
{
  // convert grid to cv::Mat
  cv::Mat grid_cv(grid.rows(), grid.cols(), CV_8UC1);
  for (int i = 0; i < grid.rows(); i++)
  {
    for (int j = 0; j < grid.cols(); j++)
    {
      grid_cv.at<uchar>(i, j) = grid(i, j);
    }
  }

  // print grid_cv with double for
  // for (int i = 0; i < grid.rows(); i++)
  // {
  //   for (int j = 0; j < grid.cols(); j++)
  //   {
  //     std::cout << (int)grid_cv.at<uchar>(i, j) << " ";
  //   }
  //   std::cout << std::endl;
  // }


  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grid_cv, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours)
  {
    Polygon polygon;
    for (const auto& point : contour)
    {
      polygon.push_back(Point(point.x * grid_resolution_, point.y * grid_resolution_));
    }
    polygons.push_back(polygon);
  }

}

/*  */
void VoronoiPlannerNode::plot_voronoi()
{
  plt::figure_size(plot_size_[0], plot_size_[1]);

  std::vector<double> X, Y;
  for (auto& vector : vor_result.points)
  {
    X.push_back(vector[0]);
    Y.push_back(vector[1]);
  }
  plt::plot(X, Y, "b.");

  // X.clear(); Y.clear();
  // for (auto& vector : vor_result.vertices)
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
  // plt::plot({goal[0]}, {goal[1]}, "k*");
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
}

/*  */
void VoronoiPlannerNode::save_log()
{
  auto timestamp = std::chrono::system_clock::now();
  auto timestampInSeconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp.time_since_epoch()).count();
  std::string filename = "/home/neo/workspace/logs/output_" + std::to_string(timestampInSeconds) + ".txt";
  std::ofstream outputFile(filename);

  // get points
  auto points = gen_vor.get_points();
  outputFile << "Points:" << std::endl;
  outputFile << points.size() << std::endl;
  for (auto& p : points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Points" << std::endl << std::endl;

  // get lines
  auto pts = gen_vor.get_lines();
  outputFile << "Lines:" << std::endl;
  outputFile << pts.size() << std::endl;
  for (auto& p : pts)
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
  for (auto& t : tris)
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
  for (auto& b : bnds)
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
  for (auto& p : tri_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Triangle points" << std::endl << std::endl;

  // get boundary points
  auto bnd_pts = gen_vor.get_boundary_points();
  outputFile << "Boundary points" << std::endl;
  outputFile << bnd_pts.size() << std::endl;
  for (auto& p : bnd_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Boundary points" << std::endl << std::endl;

  // get line points
  auto line_pts = gen_vor.get_line_points();
  outputFile << "Line points" << std::endl;
  outputFile << line_pts.size() << std::endl;
  for (auto& p : line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Line points" << std::endl << std::endl;

  // get triangle lined points
  auto tri_line_pts = gen_vor.get_triangle_lined_points();
  outputFile << "Triangle lined points" << std::endl;
  outputFile << tri_line_pts.size() << std::endl;
  for (auto& p : tri_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Triangle lined points" << std::endl << std::endl;

  //get boundary lined points
  auto bnd_line_pts = gen_vor.get_boundary_lined_points();
  outputFile << "Boundary lined points" << std::endl;
  outputFile << bnd_line_pts.size() << std::endl;
  for (auto& p : bnd_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Boundary line points" << std::endl << std::endl;

  // get line lined points
  auto line_line_pts = gen_vor.get_line_lined_points();
  outputFile << "Line lined points" << std::endl;
  outputFile << line_line_pts.size() << std::endl;
  for (auto& p : line_line_pts)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Line lined points" << std::endl << std::endl;

  // get chains
  auto chains = gen_vor.get_chains();
  outputFile << "Chains" << std::endl;
  outputFile << chains.size() << std::endl;
  for (auto& c : chains)
  {
    for (auto& p : c)
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
  for (auto& p : vor.vertices)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.vertices" << std::endl << std::endl;

  // get vor.ridge_points
  outputFile << "Vor.ridge_points" << std::endl;
  outputFile << vor.ridge_points.size() << std::endl;
  for (auto& p : vor.ridge_points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.ridge_points" << std::endl << std::endl;

  // get vor.ridge_vertices
  outputFile << "Vor.ridge_vertices" << std::endl;
  outputFile << vor.ridge_vertices.size() << std::endl;
  for (auto& p : vor.ridge_vertices)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.ridge_vertices" << std::endl << std::endl;

  // get vor.regions
  outputFile << "Vor.regions" << std::endl;
  outputFile << vor.regions.size() << std::endl;
  for (Chain& p : vor.regions)
  {
    for (int& q : p)
    {
      outputFile << q << " ";
    }
    outputFile << std::endl;
  }
  outputFile << "End Vor.regions" << std::endl << std::endl;

  // get vor.point_region
  outputFile << "Vor.point_region" << std::endl;
  outputFile << vor.point_region.size() << std::endl;
  for (auto& p : vor.point_region)
  {
    outputFile << p << std::endl;
  }
  outputFile << "End Vor.point_region" << std::endl << std::endl;

  // get vor.points
  outputFile << "Vor.points" << std::endl;
  outputFile << vor.points.size() << std::endl;
  for (auto& p : vor.points)
  {
    outputFile << p[0] << " " << p[1] << std::endl;
  }
  outputFile << "End Vor.points" << std::endl << std::endl;

  outputFile.close();
}
} // namespace VoronoiPlanner
