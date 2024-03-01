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

/// @author Mattia Montanari
/// @date July 2022

#include "openGJK/openGJK.hpp"

/**
 * @brief Main program of example1_c (described in Section 3.1 of the paper).
 *
 */
int main()
{
  /* Squared distance computed by openGJK.                                 */
  double dd = 0;
  /* Structure of simplex used by openGJK.                                 */
  gkSimplex s;
  /* Structures of body 1 and body 2, respectively.                        */
  gkPolytope bd1;
  gkPolytope bd2;

  /* For importing openGJK this is Step 2: adapt the data structure for the
   * two bodies that will be passed to the GJK procedure. */

  /* Coordinates of object 1. */
  std::vector<Eigen::Vector3d> vrtx1 =
  {
    Eigen::Vector3d(0.0, 5.5, 0.0),
    Eigen::Vector3d(2.3, 1.0, -2.0),
    Eigen::Vector3d(8.1, 4.0, 2.4),
    Eigen::Vector3d(4.3, 5.0, 2.2),
    Eigen::Vector3d(2.5, 1.0, 2.3),
    Eigen::Vector3d(7.1, 1.0, 2.4),
    Eigen::Vector3d(1.0, 1.5, 0.3),
    Eigen::Vector3d(3.3, 0.5, 0.3),
    Eigen::Vector3d(6.0, 1.4, 0.2)
  }; 
  bd1.coord = vrtx1;
  bd1.numpoints = vrtx1.size();

  /* Coordinates of object 1. */
  std::vector<Eigen::Vector3d> vrtx2 =
  {
    Eigen::Vector3d(0.0, -5.5, 0.0),
    Eigen::Vector3d(-2.3, -1.0, 2.0),
    Eigen::Vector3d(-8.1, -4.0, -2.4),
    Eigen::Vector3d(-4.3, -5.0, -2.2),
    Eigen::Vector3d(-2.5, -1.0, -2.3),
    Eigen::Vector3d(-7.1, -1.0, -2.4),
    Eigen::Vector3d(-1.0, -1.5, -0.3),
    Eigen::Vector3d(-3.3, -0.5, -0.3),
    Eigen::Vector3d(-6.0, -1.4, -0.2)
  };
  bd2.coord = vrtx2;
  bd2.numpoints = vrtx2.size();

  /* Initialise simplex as empty */
  s.nvrtx = 0;

  /* For importing openGJK this is Step 3: invoke the GJK procedure. */
  /* Compute squared distance using GJK algorithm. */
  dd = compute_minimum_distance(bd1, bd2, s);

  /* Print distance between objects. */
  printf("Distance between bodies %f\n", dd);

  return 0;
}
