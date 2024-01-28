/**
 * VoronoiPlanner - voronoi.py implementation.
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
GeneralizedVoronoi::GeneralizedVoronoi() : rdp_epsilon(0.0064/*rdp_epsilon_*/) {}

/*  */
void GeneralizedVoronoi::add_point(Point& point)
{
  this->points.push_back(point);
}

/*  */
void GeneralizedVoronoi::add_points(std::vector<Point>& new_points)
{
  this->points.insert(this->points.end(), new_points.begin(), new_points.end());
}

/*  */
void GeneralizedVoronoi::add_line(Line& line)
{
  this->lines.push_back(line);

  auto temp = line.generate_line();
  this->line_lined_points.insert(this->line_lined_points.end(), temp.begin(), temp.end());

  auto pts = line.get_points();
  this->line_points.insert(this->line_points.end(), pts.begin(), pts.end());
}

/*  */
void GeneralizedVoronoi::add_lines(std::vector<Line>& new_lines)
{
  for (auto& line : new_lines) {
    add_line(line);
  }
}

/*  */
void GeneralizedVoronoi::add_triangle(Triangle& triangle)
{
  this->triangles.push_back(triangle);

  auto temp = triangle.generate_line();
  this->triangle_lined_points.insert(this->triangle_lined_points.end(), temp.begin(), temp.end());

  auto tri = triangle.get_points();
  this->triangle_points.insert(this->triangle_points.end(), tri.begin(), tri.end());
}

/*  */
void GeneralizedVoronoi::add_triangles(std::vector<Triangle>& new_triangles)
{
  for (auto& triangle : new_triangles)
  {
    add_triangle(triangle);
  }
}

/*  */
void GeneralizedVoronoi::add_boundary(Line& boundary)
{
  this->boundaries.push_back(boundary);

  auto temp = boundary.generate_line();
  this->boundary_lined_points.insert(this->boundary_lined_points.end(), temp.begin(), temp.end());

  auto bound = boundary.get_points();
  this->boundary_points.insert(this->boundary_points.end(), bound.begin(), bound.end());
}

/*  */
void GeneralizedVoronoi::add_boundaries(std::vector<Line>& new_boundaries)
{
  for (auto& boundary : new_boundaries)
  {
    add_boundary(boundary);
  }
}

/*  */
void GeneralizedVoronoi::add_polygon(Polygon& polygon)
{
  auto triangles = triangulation(polygon);
  for (auto& vertices : triangles)
  {
    Triangle tri = Triangle(vertices);
    add_triangle(tri);
  }
}

/*  */
void GeneralizedVoronoi::add_polygons(Polygons& polygons)
{
  for (auto& polygon : polygons)
  {
    add_polygon(polygon);
  }
}

/*  */
Chain GeneralizedVoronoi::vertices_in_polygon()
{
  Chain in_polygon = {};

  for (size_t i = 0; i < this->vor.vertices.size(); i++)
  {
    for (auto tri : this->triangles)
    {
      if (tri.is_in_polygon(this->vor.vertices[i]))
      {
        in_polygon.push_back(i);
        break;
      }
    }
  }

  return in_polygon;
}

/*  */
void GeneralizedVoronoi::run_non_optimized(bool generate_result, Result& result)
{
  std::vector<Point> all_points = {};
  all_points.insert(all_points.end(), boundary_lined_points.begin(), boundary_lined_points.end());
  all_points.insert(all_points.end(), triangle_lined_points.begin(), triangle_lined_points.end());
  all_points.insert(all_points.end(), line_lined_points.begin(), line_lined_points.end());
  all_points.insert(all_points.end(), points.begin(), points.end());

  run_voronoi(all_points);

  // calculate unreachable vertices and ridges
  Chain unreachable_vertices = vertices_in_polygon();
  Chain ridge_to_delete = ridges_to_delete(unreachable_vertices);

  // delete unreachable vertices and ridges
  this->delete_vertex(unreachable_vertices);
  delete_ridge(ridge_to_delete);

  // reorganize ridge vertices
  this->reorganize_ridge(unreachable_vertices);

  if (generate_result) this->generate_result(result);
}

/*  */
void GeneralizedVoronoi::run_optimized(Result& result)
{
  run_non_optimized(false, result);

  while (true)
  {
    LINE
    if (!this->optimize_line())
    {
      LINE
      break;
    }
    LINE
    this->delete_unfinished();
    LINE
  }
  LINE
}

/*  */
void GeneralizedVoronoi::delete_unfinished()
{
  // regenerate chain by optimized value
  this->chains = generate_chains();
  //print chains
  // std::cout << "chains:" << std::endl;
  // for (auto& chain : chains)
  // {
  //   for (auto& ele : chain)
  //   {
  //     std::cout << ele << " ";
  //   }
  //   std::cout << std::endl;
  // }

  // calculate unfinished vertices and ridges
  Chain unfinished_vertices = this->unfinished_vertices();
  Chain ridge_to_delete = ridges_to_delete(unfinished_vertices);

  // delete unfinished vertices and ridges
  delete_vertex(unfinished_vertices);
  delete_ridge(ridge_to_delete);
  reorganize_ridge(unfinished_vertices);
}

/*  */
Chain GeneralizedVoronoi::unfinished_vertices()
{
  IndexDict dict = IndexDict(vor.ridge_vertices);
  Chain unfinished = {};

  for (auto& entry : dict.items())
  {
    if (entry.second.size() == 1)
    {
      unfinished.push_back(entry.first);
    }
  }

  Chain chain_vertices = {};
  for (auto& ele : unfinished)
  {
    for (auto& chain : chains)
    {
      if (ele == chain[0])
      {
        chain_vertices.push_back(chain[0]);
        chain_vertices.push_back(chain[1]);
        break;
      }
      else if (ele == chain.back())
      {
        chain_vertices.push_back(chain.back());
        chain_vertices.push_back(chain[chain.size()-2]);
        break;
      }
    }
  }

  std::sort(chain_vertices.begin(), chain_vertices.end());
  return chain_vertices;
}

/*  */
Chain GeneralizedVoronoi::ridges_to_delete(Chain& vertex_vec)
{
  std::vector<NodeT> to_delete = {};
  VertexChain vertices = this->vor.vertices;

  for (size_t i = 0; i < this->vor.ridge_vertices.size(); i++)
  {
    RidgeVertex rv = this->vor.ridge_vertices[i];

    // if ridge heads outside, delete ridge
    if (rv[0] == -1 || rv[1] == -1)
    {
      to_delete.push_back(i);
      continue;
    }

    // if ridge contains deleted vertex, delete ridge
    bool deleted = false;
    for (auto ver : vertex_vec)
    {
      if (ver == rv[0] || ver == rv[1])
      {
        to_delete.push_back(i);
        deleted = true;
        break;
      }
    }
    if (deleted) continue;

    // if ridge intersects with line, delete ridge
    for (auto& line : lines)
    {
      Line l2 = Line({vertices[rv[0]], vertices[rv[1]]});
      if (line.is_intersecting_class(l2))
      {
        to_delete.push_back(i);
        break;
      }
    }
  }

  return to_delete;
}

/*  */
void GeneralizedVoronoi::delete_vertex(Chain& to_delete)
{
  Chain new_vertices = {};
  for (int i = 0; i < (int) vor.vertices.size(); i++)
  {
    bool deleted = false;
    for (auto& ele : to_delete)
    {
      if (i == ele)
      {
        deleted = true;
        break;
      }
    }
    if (!deleted) new_vertices.push_back(i);
  }

  VertexChain vertices = {};
  for (auto& ele : new_vertices)
  {
    vertices.push_back(vor.vertices[ele]);
  }

  vor.vertices = vertices;
}

/*  */
void GeneralizedVoronoi::delete_ridge(Chain& to_delete)
{
  RidgeVertices new_ridge_vertices = {};
  for (int i = 0; i < (int) this->vor.ridge_vertices.size(); i++)
  {
    bool deleted = false;
    for (auto& ele : to_delete)
    {
      if (i == ele)
      {
        deleted = true;
        break;
      }
    }
    if (!deleted) new_ridge_vertices.push_back(this->vor.ridge_vertices[i]);
  }

  this->vor.ridge_vertices = new_ridge_vertices;
}

/*  */
void GeneralizedVoronoi::reorganize_ridge(Chain& deleted_vertices)
{
  for (size_t i = 0; i < vor.ridge_vertices.size(); i++)
  {
    RidgeVertex rv = vor.ridge_vertices[i];
    NodeT rv0 = rv[0];
    NodeT rv1 = rv[1];
    NodeT rv0i = find_closest(deleted_vertices, rv0);
    NodeT rv1i = find_closest(deleted_vertices, rv1);
    vor.ridge_vertices[i] = RidgeVertex({rv0 - rv0i, rv1 - rv1i});
  }
}

/*  */
void GeneralizedVoronoi::run(run_type type, bool plot, Result& result)
{
  switch (type)
  {
    case run_type::non_optimized:
      run_non_optimized(true, result);
      break;
    case run_type::optimized:
      run_optimized(result);
      break;
  }

  if (plot) this->generate_plot();
}

/*  */
void GeneralizedVoronoi::generate_plot()
{
  std::vector<double> X, Y, VX, VY;
  for (auto vector : this->vor.points)
  {
    X.push_back(vector[0]);
    Y.push_back(vector[1]);
  }
  for (auto vector : this->vor.vertices)
  {
    VX.push_back(vector[0]);
    VY.push_back(vector[1]);
  }

  std::vector<std::vector<Point>> finite_segments, infinite_segments;
  int rp_size = this->vor.ridge_points.size();
  int rv_size = this->vor.ridge_vertices.size();
  for (int i = 0; i < std::min(rp_size, rv_size); i++)
  {
    //Eigen::Matrix<NodeT, 2, 1> point_idx = this->vor.ridge_points[i];
    RidgeVertex simplex = this->vor.ridge_vertices[i];

    // check if all simplex values are >= 0
    bool check = true;
    for (auto ele : simplex)
    {
      if (ele < 0)
      {
        check = false;
        break;
      }
    }
    if (check)
    {
      Point p1 = this->vor.vertices[simplex[0]];
      Point p2 = this->vor.vertices[simplex[1]];
      finite_segments.push_back({p1, p2});
    }
    else
    {
      // infinite_segments
    }
  }

  plt::figure_size(640, 480);
  plt::plot(X, Y, "b.");
  plt::plot(VX, VY, "yo");
  // plot finite_segments
  std::vector<double> x, y;
  for (auto& segment : finite_segments)
  {
    x = {segment[0][0], segment[1][0]};
    y = {segment[0][1], segment[1][1]};
    plt::plot(x, y, "k");
  }
  plt::legend();

  plt::show();
}

/*  */
bool GeneralizedVoronoi::optimize_line()
{
LINE
  this->chains = this->generate_chains();
  if (this->chains.empty()) return false;
LINE
  VertexChains vertex_chains = this->generate_vertex_chains(this->chains);
LINE
  if (this->chains.empty()) return false;
LINE
  VertexChains optimized_chains = this->optimize_line_base(vertex_chains);
  this->regenerate_voronoi(optimized_chains);
LINE
  return true;
}

/*  */
void GeneralizedVoronoi::regenerate_voronoi(VertexChains& chains)
{
  VertexChain vertices = {};
  RidgeVertices ridge_vertices = {};

  for (auto& chain : chains)
  {
    if (std::find(vertices.begin(), vertices.end(), chain[0]) == vertices.end())
    {
      vertices.push_back(chain[0]);
    }

    for (size_t i = 0; i < chain.size()-1; i++)
    {
      int index = std::find(vertices.begin(), vertices.end(), chain[i]) - vertices.begin();
      RidgeVertex idx = {index, -1};

      if (std::find(vertices.begin(), vertices.end(), chain[i+1]) == vertices.end())
      {
        vertices.push_back(chain[i+1]);
        idx[1] = vertices.size()-1;
      }
      else
      {
        index = std::find(vertices.begin(), vertices.end(), chain[i]) - vertices.begin();
        idx[1] = index;
      }

      ridge_vertices.push_back(idx);
    }
  }

  vor.vertices = vertices;
  vor.ridge_vertices = ridge_vertices;
}

/*  */
VertexChains GeneralizedVoronoi::optimize_line_base(VertexChains& chains)
{
  VertexChains optimized_chains = {};
  for (auto& chain : chains)
  {
    VertexChain out = {};
    // // show chain
    // std::cout << "chain: ";
    // for (auto& ele : chain)
    // {
    //   std::cout << ele[0] << " " << ele[1] << " ||| ";
    // }
    // std::cout << std::endl;
    ramer_douglas_peucker(chain, rdp_epsilon, out);
    // // show out
    // std::cout << "out: ";
    // for (auto& ele : out)
    // {
    //   std::cout << ele[0] << " " << ele[1] << " ||| ";
    // }
    // std::cout << std::endl;
    optimized_chains.push_back(out);
  }
  return optimized_chains;
}

/*  */
void GeneralizedVoronoi::run_voronoi(std::vector<Point>& points)
{
  this->vor = Voronoi(points);
}

/*  */
void GeneralizedVoronoi::generate_result(Result& result)
{
  result.triangles = this->triangles;
  result.boundaries = this->boundaries;
  result.points = this->vor.points;
  result.points_polygon = this->triangle_lined_points;
  result.vertices = this->vor.vertices;
  result.ridge_vertices = this->vor.ridge_vertices;
  if (!this->chains.empty())
    result.chains = this->chains;
}

/*  */
Chains GeneralizedVoronoi::generate_chains()
{
LINE
//print vor.ridge_vertices
std::cout << "vor.ridge_vertices:" << std::endl;
for (auto& ele : this->vor.ridge_vertices)
{
  std::cout << ele[0] << " " << ele[1] << std::endl;
}
  IndexDict dict = IndexDict(this->vor.ridge_vertices);
LINE


  // ignition value must be dead end point (which has only 1 neighbor)
  int ignition_idx = -1;
  for (std::pair<NodeT, Chain> entry : dict.items())
  {
    NodeT key = entry.first;
    Chain value = entry.second;
LINE
// std::cout << "key: " << key << " value: ";
// for (auto& ele : value)
// {
//   std::cout << ele << " ";
// }
// std::cout << std::endl;

    if (value.size() == 1)
    {
      ignition_idx = key;
      break;
    }
  }
  // std::cout << "ignition_idx: " << ignition_idx << std::endl;
LINE
  // voronoi diagram with no dead end point cannot be optimized
  if (ignition_idx == -1) return {};

  // generate chains
  Chain feature_point = {};
  Chains chains = {};
  ChainStart start_point = {};
  start_point.push_back({-1, ignition_idx});
LINE
  while (!start_point.empty())
  {
LINE
    Chain temp = this->generate_chain(dict, start_point, feature_point);
LINE
    chains.push_back(temp);
  }
  // std::cout << std::endl << std::endl;
  //throw std::invalid_argument("Error in chain generation");

  return chains;
}

/*  */
Chain GeneralizedVoronoi::generate_chain(IndexDict& dict, ChainStart& start, Chain& feature)
{
  Chain chain = {};
  // print start
  // std::cout << "--------------------------------------------------" << std::endl;
  // std::cout << "start pre pop: " << std::endl;
  // std::cout << "\t\t";
  // for (auto& ele : start)
  // {
  //   std::cout << ele[0] << " " << ele[1] << " ";
  // }
  // std::cout << std::endl;

  // get new starting point
  ChainIdx idx = start.back();
  start.pop_back();

  // std::cout << "start post pop: " << std::endl;
  // std::cout << "\t\t";
  // for (auto& ele : start)
  // {
  //   std::cout << ele[0] << " " << ele[1] << " ";
  // }
  // std::cout << std::endl;

  // case of dead end point
  if (idx[0] != -1) chain.push_back(idx[0]);

  // ignite chain
  ChainStart new_start = this->chain(dict, idx, chain, feature);
  // print new_start
  // std::cout << "new_start: " << std::endl;
  // std::cout << "\t\t";
  // for (auto& ele : new_start)
  // {
  //   std::cout << ele[0] << " " << ele[1] << " ";
  // }
  // std::cout << std::endl;

  // add chain start and end to feature points
  feature.push_back(chain.front());
  feature.push_back(chain.back());

  // add new starting points to queue
  for (auto& ele : new_start)
  {
    start.push_back(ele);
  }

  return chain;
}

/*  */
ChainStart GeneralizedVoronoi::chain(IndexDict& dict, ChainIdx& idx, Chain& chain, Chain& feature)
{
  // append current point to chain
  chain.push_back(idx[1]);

  // search neighbor on index dictionary
  Chain neighbor = dict.find(idx[1]);
  int neighbor_count = neighbor.size();

  // visited is selected base on feature point(diverging point) and previous point
  // append idx[0] to feature and assign to visited
  Chain visited = feature;
  visited.push_back(idx[0]);

  // case 1, dead end point
  if (neighbor_count == 1)
  {
    // std::cout << "\t\t1" << std::endl;
    if (neighbor[0] == idx[0]) return {};
    ChainIdx new_idx = {idx[1], neighbor[0]};
    return this->chain(dict, new_idx, chain, feature);
  }

  // case 2, middle of line
  else if (neighbor_count == 2)
  {
    // std::cout << "\t\t2" << std::endl;
    std::vector<bool> has_visited = {false, false};
    // check if neighbor[0] is in visited
    bool check1 = std::find(visited.begin(), visited.end(), neighbor[0]) != visited.end();
    bool check2 = std::find(visited.begin(), visited.end(), neighbor[1]) != visited.end();

    if (check1) has_visited[0] = true;
    if (check2) has_visited[1] = true;

    NodeT next_idx;
    // if both neighbor is visited, it's end of line
    if (has_visited[0] && has_visited[1])
    {
      if (idx[0] == neighbor[0]) chain.push_back(neighbor[1]);
      else if (idx[0] == neighbor[1]) chain.push_back(neighbor[0]);
      return {};
    }

    // prevent going back
    else if (has_visited[0]) next_idx = 1;
    else if (has_visited[1]) next_idx = 0;
    else throw std::invalid_argument("Error in chain generation");

    // find next vertex
    ChainIdx new_idx = {idx[1], neighbor[next_idx]};
    return this->chain(dict, new_idx, chain, feature);
  }
  // print visited
  // std::cout << "visited: " << std::endl;
  // std::cout << "\t\t";
  // for (auto& ele : visited)
  // {
  //   std::cout << ele << " ";
  // }
  // std::cout << std::endl;
  // print neighbor
  // std::cout << "neighbor: " << std::endl;
  // std::cout << "\t\t";
  // for (auto& ele : neighbor)
  // {
  //   std::cout << ele << " ";
  // }
  // std::cout << std::endl;





  // std::cout << "\t\t3" << std::endl;
  // case more than 2, diverging point
  // line must end on diverging point
  // new starting points must be added for full construction
  ChainStart new_start_points = {};
  for (int i = 0; i < neighbor_count; i++)
  {
    // std::cout << "\t\t4" << std::endl;
    bool check = std::find(visited.begin(), visited.end(), neighbor[i]) != visited.end();
    if (check) continue;
    // std::cout << "\t\t5" << std::endl;
    new_start_points.push_back({idx[1], neighbor[i]});
  }
  // std::cout << "\t\t6" << std::endl;
  return new_start_points;
}

/*  */
// chains sono
// [[120, 121, 269], [269, 268, 267, 341], [341, 340, 339, 222, 286, 287], [290, 292]]
// ritorna [[[0.341, 0.37], [0.341, 0.372], [0.342, 0.355]]]
std::vector<std::vector<Point>> GeneralizedVoronoi::generate_vertex_chains(Chains& chains)
{
  std::vector<std::vector<Point>> vertex_chains = {};
  for (auto& chain : chains)
  {
    std::vector<Point> vertex_chain = {};
    for (auto& ele : chain)
    {
      vertex_chain.push_back(this->vor.vertices[ele]);
    }
    vertex_chains.push_back(vertex_chain);
  }
  return vertex_chains;
}

} // namespace VoronoiPlanner