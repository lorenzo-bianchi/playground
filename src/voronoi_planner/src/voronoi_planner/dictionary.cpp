/**
 * VoronoiPlanner - dictionary.py implementation.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * ChainTelligent Systems Lab <isl.torvergata@gmail.com>
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

/*
ridge_vecritces in ingresso al costruttore è un array di coppie di punti
    ridge_vertices = []
    [ 44  45]
    [ 46  47]
    [ 29  30]
    ...
  ]

nel dizionario può accadere idx : [a], idx : [a, b] o idx : [a, b], forse di più

*/

/*  */
IndexDict::IndexDict(RidgeVertices& vec)
{
  DictT dict = generate(vec, false);
  DictT rdict = generate(vec, true);

  for (auto entry : rdict)
  {
    NodeT key = entry.first;
    Chain& value = entry.second;

    if (contains(dict, key))
    {
      for (NodeT ele : value)
      {
        if (std::find(dict[key].begin(), dict[key].end(), ele) == dict[key].end())
        {
          dict[key].push_back(ele);
        }
      }
    }
    else dict[key] = value;
  }

  this->dict = dict;
}

/*  */
DictT IndexDict::generate(RidgeVertices& vec, bool reverse)
{
  RidgeVertices sorted = vec;
  NodeT idx = reverse ? 1 : 0;

  std::sort(
    sorted.begin(), sorted.end(), [idx](RidgeVertex& a, RidgeVertex& b)
    {
      return a[idx] < b[idx];
    });


  Chain organized;
  Chains value;
  std::vector<NodeT> key;

  NodeT current = sorted[0][idx];

  for (auto ele : sorted)
  {
    if (ele[idx] != current)
    {
      value.push_back(organized);
      key.push_back(current);
      organized.clear();
      current = ele[idx];
    }
    organized.push_back(reverse ? ele[0] : ele[1]);
  }

  value.push_back(organized);
  key.push_back(current);

  std::unordered_map<NodeT, Chain> result;
  for (size_t i = 0; i < key.size(); i++)
  {
    result[key[i]] = value[i];
  }

  return result;
}

/*  */
bool IndexDict::contains(DictT& dict, NodeT key)
{
  return dict.find(key) != dict.end();
}

/*  */
bool IndexDict::contains(NodeT key)
{
  return contains(this->dict, key);
}

/*  */
void IndexDict::insert(NodeT key, Chain& value)
{
  if (contains(this->dict, key))
  {
    throw std::invalid_argument("Key already exists");
  }

  this->dict[key] = value;

  for (NodeT ele : value)
  {
    if (contains(this->dict, ele))
    {
      this->dict[ele].push_back(key);
    }
    else
    {
      this->dict[ele] = {key};
    }
  }
}

/*  */
Chain IndexDict::find(NodeT key)
{
  if (contains(key)) return this->dict.at(key);
  return {};
}

/*  */
std::vector<std::pair<NodeT, Chain>> IndexDict::items()
{
  std::vector<std::pair<NodeT, Chain>> result;
  for (auto entry : this->dict)
  {
    result.push_back(entry);
  }
  return result;
}

} // namespace VoronoiPlanner