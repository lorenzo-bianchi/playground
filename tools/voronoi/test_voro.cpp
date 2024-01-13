#include "test_voro.hpp"

Line::Line(const std::vector<std::vector<double>> inputPoints) : point_distance(0.0), points(inputPoints)
{
    if (points.size() != 2) throw std::invalid_argument("Line must have 2 points.");
}

std::vector<std::vector<double>> Line::generateLine()
{
    return generateLineBase(points[0], points[1]);
}

bool Line::isIntersecting(const Line& otherLine)
{
    return isIntersectingBase(points, otherLine.points);
}


std::vector<std::vector<double>> Line::generateLineBase(
    const std::vector<double>& vec1,
    const std::vector<double>& vec2)
{
    std::vector<std::vector<double>> result;

    if (vec1[0] != vec2[0]) {
        result = generateLineX(vec1, vec2);
    } else if (vec1[1] != vec2[1]) {
        result = generateLineY(vec1, vec2);
    } else {
        std::cout << "Identical point has been detected." << std::endl;
        std::cout << "Points: [" << vec1[0] << ", " << vec1[1] << "]" << std::endl;
    }

    return result;
}

std::vector<std::vector<double>> Line::generateLineX(
    const std::vector<double>& vec1,
    const std::vector<double>& vec2)
{
    return generateLineBase(vec1, vec2, 0);
}

std::vector<std::vector<double>> Line::generateLineY(
    const std::vector<double>& vec1,
    const std::vector<double>& vec2)
{
    return generateLineBase(vec1, vec2, 1);
}

std::vector<std::vector<double>> Line::generateLineBase(
    const std::vector<double>& vec1,
    const std::vector<double>& vec2,
    int axis)
{
    std::vector<std::vector<double>> result;

    if (vec1[axis] > vec2[axis])
    {
        std::swap(vec1, vec2);
    }

    double dis = distance(vec1, vec2);
    std::vector<double> stepVec = {(vec2[0] - vec1[0]) / dis, (vec2[1] - vec1[1]) / dis};
    for (double current = vec1[axis]; current <= vec2[axis]; current += point_distance)
    {
        result.push_back({vec1[0] + current * stepVec[0], vec1[1] + current * stepVec[1]});
    }

    return result;
}

double Line::distance(const std::vector<double>& vec1, const std::vector<double>& vec2)
{
    return std::sqrt(std::pow(vec2[0] - vec1[0], 2) + std::pow(vec2[1] - vec1[1], 2));
}

bool Line::isIntersectingBase(
    const std::vector<std::vector<double>>& line1,
    const std::vector<std::vector<double>>& line2)
{
    // Implementation of intersection check between two lines
    // You may need to replace this with the actual logic for line intersection
    return true;
}

class GVoronoi
{
public:
    GVoronoi();
    void add_polygons(std::vector<std::vector<std::vector<double>>> polygons)
    {
        for (auto polygon : polygons)
        {
            std::vector<std::vector<double>> points;
            for (auto point : polygon)
            {
                std::vector<double> p = {point[0], point[1], 0.0};
                points.push_back(p);
            }
            this->polygons.push_back(points);
        }
    }

private:

};

int main()
{
    polygons = {
                {{0.08, 0.08}, {0.3, 0.08}, {0.3, 0.3}},
                {{0.08, 0.08}, {0.3, 0.3}, {0.08, 0.3}},
                {{0.3, 0.5}, {0.4, 0.4}, {0.5, 0.5}},
                {{0.3, 0.5}, {0.5, 0.5}, {0.3, 0.6}}
               };

    GVoronoi vor = GVoronoi();
    vor.add_polygons(polygons);
    vor.add_boundaries();

    qh_zero(qh, errfile);

    /* initialize dim, numpoints, points[], ismalloc here */
    exitcode = qh_new_qhull_scipy(qh, dim, numpoints, points, ismalloc, flags, outfile, errfile, feaspoint);
    if (!exitcode) {                  /* if no error */
        /* 'qh->facet_list' contains the convex hull */
        for (facet = qh->facet_list; facet && facet->next; facet = facet->next) {
            /* ... your code ... */
        }
    }

    // Release memory
    qh_freeqhull(qh, !qh_ALL);
    qh_memfreeshort(qh, &curlong, &totlong);
    if (curlong || totlong)
        qh_fprintf(qh, errfile, 7079, "qhull internal warning (main): did not free %d bytes of long memory(%d pieces)\n", totlong, curlong);
}