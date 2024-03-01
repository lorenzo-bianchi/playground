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

#include "openGJK/openGJK.hpp"

#define eps_rel22 1e-10
#define eps_tot22 1e-12

#define norm2(a) (a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
#define dotProduct(a, b) (a[0] * b[0] + a[1] * b[1] + a[2] * b[2])

#define S3Dregion1234()                                  \
  v[0] = 0;                                              \
  v[1] = 0;                                              \
  v[2] = 0;                                              \
  s.nvrtx = 4;

#define select_1ik()                                     \
  s.nvrtx = 3;                                          \
  s.vrtx[2] = s.vrtx[3]; \
  s.vrtx[1] = si;         \
  s.vrtx[0] = sk;

#define select_1ij()                                     \
  s.nvrtx = 3;                                          \
  s.vrtx[2] = s.vrtx[3]; \
  s.vrtx[1] = si;         \
  s.vrtx[0] = sj;

#define select_1jk()                                     \
  s.nvrtx = 3;                                          \
  s.vrtx[2] = s.vrtx[3]; \
  s.vrtx[1] = sj;         \
  s.vrtx[0] = sk;

#define select_1i()                                      \
  s.nvrtx = 2;                                          \
  s.vrtx[1] = s.vrtx[3]; \
  s.vrtx[0] = si;

#define select_1j()                                      \
  s.nvrtx = 2;                                          \
  s.vrtx[1] = s.vrtx[3]; \
  s.vrtx[0] = sj;

#define select_1k()                                      \
  s.nvrtx = 2;                                          \
  s.vrtx[1] = s.vrtx[3]; \
  s.vrtx[0] = sk;

#define getvrtx(point, location)                         \
  point = s.vrtx[location];                       \

#define calculateEdgeVector(p1p2, p2)                    \
  p1p2 = p2 - s.vrtx[3];                       \

#define S1Dregion1()                                     \
  v = s.vrtx[1];                                  \
  s.nvrtx = 1;                                          \
  s.vrtx[0] = s.vrtx[1];                         \

#define S2Dregion1()                                     \
  v = s.vrtx[2];                                  \
  s.nvrtx = 1;                                          \
  s.vrtx[0] = s.vrtx[2];                         \

#define S2Dregion12()                                    \
  s.nvrtx = 2;                                          \
  s.vrtx[0] = s.vrtx[2];                         \

#define S2Dregion13()                                    \
  s.nvrtx = 2;                                          \
  s.vrtx[1] = s.vrtx[2];                         \

#define S3Dregion1()                                     \
  v = s1;                                          \
  s.nvrtx = 1;                                          \
  s.vrtx[0] = s1;                                 \

inline static double determinant(const Eigen::Vector3d p,
                                 const Eigen::Vector3d q,
                                 const Eigen::Vector3d r)
{
  return p[0] * ((q[1] * r[2]) - (r[1] * q[2])) - p[1] * (q[0] * r[2] - r[0] * q[2]) +
         p[2] * (q[0] * r[1] - r[0] * q[1]);
}

inline static void crossProduct(const Eigen::Vector3d a,
                                const Eigen::Vector3d b,
                                Eigen::Vector3d& c)
{
  c = a.cross(b);
}

inline static void projectOnLine(const Eigen::Vector3d p, 
                                 const Eigen::Vector3d q,
                                 Eigen::Vector3d& v)
{
  Eigen::Vector3d pq;
  double tmp;
  pq = p - q;

  tmp = dotProduct(p, pq) / dotProduct(pq, pq);

  v = p - pq * tmp;
}

inline static void projectOnPlane(const Eigen::Vector3d p, 
                                  const Eigen::Vector3d q, 
                                  const Eigen::Vector3d r, 
                                  Eigen::Vector3d& v)
{
  Eigen::Vector3d n, pq, pr;
  double tmp;

  pq = p - q;
  pr = p - r;

  crossProduct(pq, pr, n);
  tmp = dotProduct(n, p) / dotProduct(n, n);

  v = n * tmp;
}

inline static int hff1(const Eigen::Vector3d p, const Eigen::Vector3d q)
{
  double tmp = 0;

  for (int i = 0; i < 3; i++) tmp += (p[i] * p[i] - p[i] * q[i]);

  if (tmp > 0) return 1;  // keep q
  return 0;
}

inline static int hff2(const Eigen::Vector3d p, 
                       const Eigen::Vector3d q, 
                       const Eigen::Vector3d r)
{
  Eigen::Vector3d ntmp;
  Eigen::Vector3d n, pq, pr;
  double tmp = 0;

  pq = q - p;
  pr = r - p;

  crossProduct(pq, pr, ntmp);
  crossProduct(pq, ntmp, n);

  for (int i = 0; i < 3; i++) tmp += p[i] * n[i];

  if (tmp < 0) return 1;  // Discard r

  return 0;
}

inline static int hff3(const Eigen::Vector3d p,
                       const Eigen::Vector3d q,
                       const Eigen::Vector3d r)
{
  Eigen::Vector3d n, pq, pr;
  double tmp = 0;

  pq = q - p;
  pr = r - p;

  crossProduct(pq, pr, n);

  for (int i = 0; i < 3; i++) tmp += (p[i] * n[i]);

  if (tmp > 0) return 0;  // discard s

  return 1;
}

inline static void S1D(gkSimplex& s, Eigen::Vector3d& v)
{
  Eigen::Vector3d s1p = s.vrtx[1];
  Eigen::Vector3d s2p = s.vrtx[0];

  if (hff1(s1p, s2p))
  {
    projectOnLine(s1p, s2p, v);  // Update v, no need to update s
    return;                      // Return V{1,2}
  }
  else
  {
    S1Dregion1();  // Update v and s
    return;        // Return V{1}
  }
}

inline static void S2D(gkSimplex& s, Eigen::Vector3d& v)
{
  Eigen::Vector3d s1p = s.vrtx[2];
  Eigen::Vector3d s2p = s.vrtx[1];
  Eigen::Vector3d s3p = s.vrtx[0];
  int hff1f_s12 = hff1(s1p, s2p);
  int hff1f_s13 = hff1(s1p, s3p);
  int hff2f_23 = !hff2(s1p, s2p, s3p);
  int hff2f_32 = !hff2(s1p, s3p, s2p);

  if (hff1f_s12)
  {
    if (hff2f_23)
    {
      if (hff1f_s13)
      {
        if (hff2f_32)
        {
          projectOnPlane(s1p, s2p, s3p, v);  // Update s, no need to update c
          return;                            // Return V{1,2,3}
        } 
        else
        {
          projectOnLine(s1p, s3p, v);  // Update v
          S2Dregion13();               // Update s
          return;                      // Return V{1,3}
        }
      }
      else
      {
        projectOnPlane(s1p, s2p, s3p, v);  // Update s, no need to update c
        return;                            // Return V{1,2,3}
      }
    }
    else
    {
      projectOnLine(s1p, s2p, v);  // Update v
      S2Dregion12();               // Update s
      return;                      // Return V{1,2}
    }
  }
  else if (hff1f_s13)
  {
    if (hff2f_32)
    {
      projectOnPlane(s1p, s2p, s3p, v);  // Update s, no need to update v
      return;                            // Return V{1,2,3}
    }
    else
    {
      projectOnLine(s1p, s3p, v);  // Update v
      S2Dregion13();               // Update s
      return;                      // Return V{1,3}
    }
  }
  else
  {
    S2Dregion1();  // Update s and v
    return;        // Return V{1}
  }
}

inline static void S3D(gkSimplex& s, Eigen::Vector3d& v)
{
  Eigen::Vector3d s1, s2, s3, s4, s1s2, s1s3, s1s4;
  Eigen::Vector3d si, sj, sk;
  int testLineThree, testLineFour, testPlaneTwo, testPlaneThree, testPlaneFour, dotTotal;
  int i, j, k;

  getvrtx(s1, 3);
  getvrtx(s2, 2);
  getvrtx(s3, 1);
  getvrtx(s4, 0);
  calculateEdgeVector(s1s2, s2);
  calculateEdgeVector(s1s3, s3);
  calculateEdgeVector(s1s4, s4);

  Eigen::Vector3i hff1_tests;
  hff1_tests[2] = hff1(s1, s2);
  hff1_tests[1] = hff1(s1, s3);
  hff1_tests[0] = hff1(s1, s4);
  testLineThree = hff1(s1, s3);
  testLineFour = hff1(s1, s4);

  dotTotal = hff1(s1, s2) + testLineThree + testLineFour;
  if (dotTotal == 0) /* case 0.0 -------------------------------------- */
  {
    S3Dregion1();
    return;
  }

  double det134 = determinant(s1s3, s1s4, s1s2);
  int sss;
  if (det134 > 0)
    sss = 0;
  else
    sss = 1;

  testPlaneTwo = hff3(s1, s3, s4) - sss;
  testPlaneTwo = testPlaneTwo * testPlaneTwo;
  testPlaneThree = hff3(s1, s4, s2) - sss;
  testPlaneThree = testPlaneThree * testPlaneThree;
  testPlaneFour = hff3(s1, s2, s3) - sss;
  testPlaneFour = testPlaneFour * testPlaneFour;

  switch (testPlaneTwo + testPlaneThree + testPlaneFour)
  {
    case 3:
      S3Dregion1234();
      break;

    case 2:
      // Only one facing the oring
      // 1,i,j, are the indices of the points on the triangle and remove k from
      // simplex
      s.nvrtx = 3;
      if (!testPlaneTwo)  // k = 2;   removes s2
        s.vrtx[2] = s.vrtx[3];
      else if (!testPlaneThree)  // k = 1; // removes s3
      {
        s.vrtx[1] = s2;
        s.vrtx[2] = s.vrtx[3];
      } 
      else if (!testPlaneFour)  // k = 0; // removes s4  and no need to reorder
      {
        s.vrtx[0] = s3;
        s.vrtx[1] = s2;
        s.vrtx[2] = s.vrtx[3];
      }
      // Call S2D
      S2D(s, v);
      break;
    case 1:
      // Two triangles face the origins:
      //    The only positive hff3 is for triangle 1,i,j, therefore k must be in
      //    the solution as it supports the the point of minimum norm.

      // 1,i,j, are the indices of the points on the triangle and remove k from
      // simplex
      s.nvrtx = 3;
      if (testPlaneTwo)
      {
        k = 2;  // s2
        i = 1;
        j = 0;
      } 
      else if (testPlaneThree)
      {
        k = 1;  // s3
        i = 0;
        j = 2;
      }
      else
      {
        k = 0;  // s4
        i = 2;
        j = 1;
      }

      getvrtx(si, i);
      getvrtx(sj, j);
      getvrtx(sk, k);

      if (dotTotal == 1)
      {
        if (hff1_tests[k])
        {
          if (!hff2(s1, sk, si))
          {
            select_1ik();
            projectOnPlane(s1, si, sk, v);
          }
          else if (!hff2(s1, sk, sj))
          {
            select_1jk();
            projectOnPlane(s1, sj, sk, v);
          }
          else
          {
            select_1k();  // select region 1i
            projectOnLine(s1, sk, v);
          }
        }
        else if (hff1_tests[i])
        {
          if (!hff2(s1, si, sk))
          {
            select_1ik();
            projectOnPlane(s1, si, sk, v);
          }
          else
          {
            select_1i();  // select region 1i
            projectOnLine(s1, si, v);
          }
        }
        else
        {
          if (!hff2(s1, sj, sk))
          {
            select_1jk();
            projectOnPlane(s1, sj, sk, v);
          }
          else
          {
            select_1j();  // select region 1i
            projectOnLine(s1, sj, v);
          }
        }
      }
      else if (dotTotal == 2)
      {
        // Two edges have positive hff1, meaning that for two edges the origin's
        // project fall on the segement.
        //  Certainly the edge 1,k supports the the point of minimum norm, and so
        //  hff1_1k is positive

        if (hff1_tests[i])
        {
          if (!hff2(s1, sk, si))
            if (!hff2(s1, si, sk))
            {
              select_1ik();  // select region 1ik
              projectOnPlane(s1, si, sk, v);
            }
            else
            {
              select_1k();  // select region 1k
              projectOnLine(s1, sk, v);
            }
          else
          {
            if (!hff2(s1, sk, sj))
            {
              select_1jk();  // select region 1jk
              projectOnPlane(s1, sj, sk, v);
            }
            else
            {
              select_1k();  // select region 1k
              projectOnLine(s1, sk, v);
            }
          }
        }
        else if (hff1_tests[j])
        {  //  there is no other choice
          if (!hff2(s1, sk, sj))
            if (!hff2(s1, sj, sk))
            {
              select_1jk();  // select region 1jk
              projectOnPlane(s1, sj, sk, v);
            }
            else
            {
              select_1j();  // select region 1j
              projectOnLine(s1, sj, v);
            }
          else
          {
            if (!hff2(s1, sk, si))
            {
              select_1ik();  // select region 1ik
              projectOnPlane(s1, si, sk, v);
            }
            else
            {
              select_1k();  // select region 1k
              projectOnLine(s1, sk, v);
            }
          }
        }
        else
        {
          // ERROR;
        }

      }
      else if (dotTotal == 3)
      {
        // MM : ALL THIS HYPHOTESIS IS FALSE
        // sk is s.t. hff3 for sk < 0. So, sk must support the origin because
        // there are 2 triangles facing the origin.

        int hff2_ik = hff2(s1, si, sk);
        int hff2_jk = hff2(s1, sj, sk);
        int hff2_ki = hff2(s1, sk, si);
        int hff2_kj = hff2(s1, sk, sj);

        if (hff2_ki == 0 && hff2_kj == 0)
          printf("\n\n UNEXPECTED VALUES!!! \n\n");

        if (hff2_ki == 1 && hff2_kj == 1)
        {
          select_1k();
          projectOnLine(s1, sk, v);
        }
        else if (hff2_ki)
        {
          // discard i
          if (hff2_jk)
          {
            // discard k
            select_1j();
            projectOnLine(s1, sj, v);
          }
          else
          {
            select_1jk();
            projectOnPlane(s1, sk, sj, v);
          }
        }
        else
        {
          // discard j
          if (hff2_ik)
          {
            // discard k
            select_1i();
            projectOnLine(s1, si, v);
          }
          else
          {
            select_1ik();
            projectOnPlane(s1, sk, si, v);
          }
        }
      }
      break;

    case 0:
      // The origin is outside all 3 triangles
      if (dotTotal == 1)
      {
        // Here si is set such that hff(s1,si) > 0
        if (testLineThree)
        {
          k = 2;
          i = 1;  // s3
          j = 0;
        }
        else if (testLineFour)
        {
          k = 1;  // s3
          i = 0;
          j = 2;
        }
        else
        {
          k = 0;
          i = 2;  // s2
          j = 1;
        }
        getvrtx(si, i);
        getvrtx(sj, j);
        getvrtx(sk, k);

        if (!hff2(s1, si, sj))
        {
          select_1ij();
          projectOnPlane(s1, si, sj, v);
        }
        else if (!hff2(s1, si, sk))
        {
          select_1ik();
          projectOnPlane(s1, si, sk, v);
        }
        else
        {
          select_1i();
          projectOnLine(s1, si, v);
        }
      }
      else if (dotTotal == 2)
      {
        // Here si is set such that hff(s1,si) < 0
        s.nvrtx = 3;
        if (!testLineThree)
        {
          k = 2;
          i = 1;  // s3
          j = 0;
        }
        else if (!testLineFour)
        {
          k = 1;
          i = 0;  // s4
          j = 2;
        }
        else
        {
          k = 0;
          i = 2;  // s2
          j = 1;
        }
        getvrtx(si, i);
        getvrtx(sj, j);
        getvrtx(sk, k);

        if (!hff2(s1, sj, sk))
        {
          if (!hff2(s1, sk, sj))
          {
            select_1jk();  // select region 1jk
            projectOnPlane(s1, sj, sk, v);
          }
          else if (!hff2(s1, sk, si))
          {
            select_1ik();
            projectOnPlane(s1, sk, si, v);
          }
          else
          {
            select_1k();
            projectOnLine(s1, sk, v);
          }
        }
        else if (!hff2(s1, sj, si))
        {
          select_1ij();
          projectOnPlane(s1, si, sj, v);
        }
        else
        {
          select_1j();
          projectOnLine(s1, sj, v);
        }
      }
      break;
    default:
      printf("\nERROR:\tunhandled");
  }
}

inline static void support(gkPolytope& body, const Eigen::Vector3d v)
{
  double s, maxs;
  Eigen::Vector3d vrt;
  int better = -1;

  maxs = dotProduct(body.s, v);

  for (int i = 0; i < body.numpoints; ++i)
  {
    vrt = body.coord[i];
    s = dotProduct(vrt, v);
    if (s > maxs)
    {
      maxs = s;
      better = i;
    }
  }

  if (better != -1)
  {
    body.s[0] = body.coord[better][0];
    body.s[1] = body.coord[better][1];
    body.s[2] = body.coord[better][2];
  }
}

inline static void subalgorithm(gkSimplex& s, Eigen::Vector3d& v) {
  switch (s.nvrtx)
  {
    case 4:
      S3D(s, v);
      break;
    case 3:
      S2D(s, v);
      break;
    case 2:
      S1D(s, v);
      break;
    default:
      printf("\nERROR:\t invalid simplex\n");
  }
}

double compute_minimum_distance(gkPolytope bd1, gkPolytope bd2, gkSimplex& s)
{
  int k = 0;                        /**< Iteration counter            */
  int i;                            /**< General purpose counter      */
  int mk = 25;                      /**< Maximum number of iterations of the GJK algorithm */
  int absTestin;
  double norm2Wmax = 0;
  double tesnorm;
  Eigen::Vector3d v;                /**< Search direction             */
  Eigen::Vector3d vminus;           /**< Search direction * -1        */
  Eigen::Vector3d w;                /**< Vertex on CSO boundary given by the difference of support functions on both bodies */
  double eps_rel = eps_rel22;       /**< Tolerance on relative        */
  double eps_rel2 = eps_rel * eps_rel;
  double eps_tot = eps_tot22;
  double exeedtol_rel;              /**< Test for 1st exit condition  */
  int nullV = 0;

  /* Initialise search direction */
  v = bd1.coord[0] - bd2.coord[0];

  /* Inialise simplex */
  s.nvrtx = 1;
  s.vrtx[0] = v;

  bd1.s = bd1.coord[0];
  bd2.s = bd2.coord[0];

  /* Begin GJK iteration */
  do
  {
    k++;

    /* Update negative search direction */
    vminus = -v;

    /* Support function */
    support(bd1, vminus);
    support(bd2, v);
    w = bd1.s - bd2.s;

    /* Test first exit condition (new point already in simplex/can't move further) */
    exeedtol_rel = (norm2(v) - dotProduct(v, w));
    if (exeedtol_rel <= (eps_rel * norm2(v)) || exeedtol_rel < eps_tot22)
      break;

    nullV = norm2(v) < eps_rel2;
    if (nullV)
      break;

    /* Add new vertex to simplex */
    i = s.nvrtx;
    s.vrtx[i] = w;
    s.nvrtx++;

    /* Invoke distance sub-algorithm */
    subalgorithm(s, v);

    /* Test */
    for (int jj = 0; jj < s.nvrtx; jj++)
    {
      tesnorm = norm2(s.vrtx[jj]);
      if (tesnorm > norm2Wmax)
        norm2Wmax = tesnorm;
    }

    absTestin = (norm2(v) <= (eps_tot * eps_tot * norm2Wmax));
    if (absTestin)
      break;

  } while ((s.nvrtx != 4) && (k != mk));

  if (k == mk)
    printf("\n * * * * * * * * * MAXIMUM ITERATION NUMBER REACHED!!!  * * * * * * * * * * * \n");

  return sqrt(norm2(v));
}
