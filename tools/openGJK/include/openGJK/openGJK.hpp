//                           _____      _ _  __                                   //
//                          / ____|    | | |/ /                                   //
//    ___  _ __   ___ _ __ | |  __     | | ' /                                    //
//   / _ \| '_ \ / _ \ '_ \| | |_ |_   | |  <                                     //
//  | (_) | |_) |  __/ | | | |__| | |__| | . \                                    //
//   \___/| .__/ \___|_| |_|\_____|\____/|_|\_\                                   //
//        | |                                                                     //
//        |_|                                                                     //
//                                                                                //
// Copyright 2022 Mattia Montanari, University of Oxford                          //
//                                                                                //
// This program is free software: you can redistribute it and/or modify it under  //
// the terms of the GNU General Public License as published by the Free Software  //
// Foundation, either version 3 of the License. You should have received a copy   //
// of the GNU General Public License along with this program. If not, visit       //
//                                                                                //
//     https://www.gnu.org/licenses/                                              //
//                                                                                //
// This program is distributed in the hope that it will be useful, but WITHOUT    //
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  //
// FOR A PARTICULAR PURPOSE. See GNU General Public License for details.          //

#ifndef OPENGJK_H__
#define OPENGJK_H__

#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "math.h"

/// @brief Structure of a body
struct gkPolytope
{
  int numpoints;        // Number of points defining the body
  Eigen::Vector3d s;         // Support mapping computed last
  std::vector<Eigen::Vector3d> coord;      // Points' coordinates

  gkPolytope() : coord(4) {}
};

/// @brief Structure of the simplex
struct gkSimplex
{
  int nvrtx;            // Number of simplex's vertices
  Eigen::Vector4i wids;          // Label of the simplex's vertices
  Eigen::Vector4d lambdas;   // Barycentric coordiantes for each vertex
  std::vector<Eigen::Vector3d> vrtx;   // Coordinates of simplex's vertices

  gkSimplex() : vrtx(4) {} 
};

/// @brief Uses the GJK algorithm to compute the minimum distance between two bodies
double compute_minimum_distance(const gkPolytope p_, const gkPolytope q_, gkSimplex& s_);

#endif  // OPENGJK_H__
