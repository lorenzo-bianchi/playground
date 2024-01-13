#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <ostream>
#include <vector>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/Qhull.h"

#include "libqhull_r/libqhull_r.h"

using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetList;
using orgQhull::QhullQh;
using orgQhull::RboxPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;

// void qh_zero(qhT *qh, FILE *errfile)
// {
//     memset((char *)qh, 0, sizeof(qhT));   /* every field is 0, FALSE, NULL */
//     qh->NOerrexit= True;
//     qh_meminit(qh, errfile);
// }

class Line {
public:
    Line(const std::vector<std::vector<double>> inputPoints);
    std::vector<std::vector<double>> generateLine();
    bool isIntersecting(const Line& otherLine);
private:
    std::vector<std::vector<double>> points;
    double point_distance;

    std::vector<std::vector<double>> generateLineBase(const std::vector<double>& vec1, const std::vector<double>& vec2);
    std::vector<std::vector<double>> generateLineX(const std::vector<double>& vec1, const std::vector<double>& vec2);
    std::vector<std::vector<double>> generateLineY(const std::vector<double>& vec1, const std::vector<double>& vec2);
    std::vector<std::vector<double>> generateLineBase(const std::vector<double>& vec1, const std::vector<double>& vec2, int axis);
    double distance(const std::vector<double>& vec1, const std::vector<double>& vec2);
    bool isIntersectingBase(const std::vector<std::vector<double>>& line1, const std::vector<std::vector<double>>& line2);
};

class GVoronoi
{
public:
    GVoronoi();
    void add_polygons(std::vector<std::vector<std::vector<double>>> polygons);

private:

};

std::vector<std::vector<std::vector<double>>> polygons;

int dim;                  /* dimension of points */
int numpoints;            /* number of points */
coordT *points;           /* array of coordinates for each point */
coordT *feaspoint;
boolT ismalloc;           /* True if qhull should free points in qh_freeqhull() or reallocation */
char flags[] = "qhull Tv"; /* option flags for qhull, see html/qh-quick.htm */
FILE *outfile= stdout;    /* output from qh_produce_output
                            use NULL to skip qh_produce_output */
FILE *errfile= stderr;    /* error messages from qhull code */
int exitcode;             /* 0 if no error from qhull */
facetT *facet;            /* set by FORALLfacets */
int curlong, totlong;     /* memory remaining after qh_memfreeshort */

qhT qh_qh;                /* Qhull's data structure.  First argument of most calls */
qhT *qh= &qh_qh;          /* Alternatively -- qhT *qh= (qhT *)malloc(sizeof(qhT)) */