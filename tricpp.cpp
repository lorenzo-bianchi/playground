#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

double EPSILON = sqrt(std::numeric_limits<double>::epsilon());

template <class T>
void print_vector(const std::vector<T>& v)
{
    for (const auto& e : v)
    {
        std::cout << e << std::endl;
    }
    std::cout << std::endl;
}

class  Point
{
    public:
        double x;
        double y;

        Point(double x, double y) : x(x), y(y) {}

        bool operator==(const Point& other) const
        {
            return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
        }

        bool operator!=(const Point& other) const
        {
            return !(*this == other);
        }

        friend std::ostream& operator<<(std::ostream& os, const Point& point)
        {
            os << '\t' << point.x << ' ' << point.y;
            return os;
        }
};
typedef std::vector<Point> Polygon;
typedef std::vector<Polygon> Polygons;

class Triangle
{
    public:
        Point p1;
        Point p2;
        Point p3;

    friend std::ostream& operator<<(std::ostream& os, const Triangle& triangle)
    {
        os << '\t' << triangle.p1 << ' ' << triangle.p2 << ' ' << triangle.p3;
        return os;
    }
};

bool isClockwise(const Polygon& polygon)
{
    double s = 0;
    int polygonCount = polygon.size();
    for (int i = 0; i < polygonCount; ++i)
    {
        const Point& point = polygon[i];
        const Point& point2 = polygon[(i + 1) % polygonCount];
        s += (point2.x - point.x) * (point2.y + point.y);
    }
    return s > 0;
}

double triangleSum(double x1, double y1, double x2, double y2, double x3, double y3)
{
    return x1 * (y3 - y2) + x2 * (y1 - y3) + x3 * (y2 - y1);
}

bool isConvex(const Point& prev, const Point& point, const Point& next)
{
    return triangleSum(prev.x, prev.y, point.x, point.y, next.x, next.y) < 0;
}

double triangleArea(const Point& p1, const Point& p2, const Point& p3)
{
    return std::abs((p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y)) / 2.0);
}

bool isPointInside(const Point& p, const Point& a, const Point& b, const Point& c)
{
    double area = triangleArea(a, b, c);
    double area1 = triangleArea(p, b, c);
    double area2 = triangleArea(p, a, c);
    double area3 = triangleArea(p, a, b);
    double areaDiff = std::abs(area - (area1 + area2 + area3));
    return areaDiff < EPSILON;
}

bool containsNoPoints(const Point& p1, const Point& p2, const Point& p3, const Polygon& polygon)
{
    for (const Point& pn : polygon)
    {
        if (pn == p1 || pn == p2 || pn == p3)
        {
            continue;
        }
        else if (isPointInside(pn, p1, p2, p3))
        {
            return false;
        }
    }
    return true;
}

bool isEar(const Point& p1, const Point& p2, const Point& p3, const Polygon& polygon)
{
    return containsNoPoints(p1, p2, p3, polygon) &&
           isConvex(p1, p2, p3) &&
           triangleArea(p1, p2, p3) > 0;
}

void earclip(const Polygon& polygon, std::vector<Triangle>* triangles)
{
    Polygon earVertex = {};

    Polygon polygonCopy = polygon;
    if (isClockwise(polygon))
    {
        std::reverse(polygonCopy.begin(), polygonCopy.end());
    }

    int pointCount = polygonCopy.size();
    for (int i = 0; i < pointCount; i++)
    {
        int prevIndex = (i - 1 + pointCount) % pointCount;
        const Point& prevPoint = polygonCopy[prevIndex];
        const Point& point = polygonCopy[i];
        int nextIndex = (i + 1) % pointCount;
        const Point& nextPoint = polygonCopy[nextIndex];

        if (isEar(prevPoint, point, nextPoint, polygonCopy))
        {
            earVertex.push_back(point);
        }
    }

    while (!earVertex.empty() && pointCount >= 3)
    {
        Point ear = earVertex.front();
        earVertex.erase(earVertex.begin());

        auto it = std::find(polygonCopy.begin(), polygonCopy.end(), ear);
        int i = std::distance(polygonCopy.begin(), it);

        int prevIndex = (i - 1 + pointCount) % pointCount;
        Point prevPoint = polygonCopy[prevIndex];

        int nextIndex = (i + 1) % pointCount;
        Point nextPoint = polygonCopy[nextIndex];

        polygonCopy.erase(it);

        pointCount--;
        triangles->push_back({prevPoint, ear, nextPoint});

        if (pointCount > 3)
        {
            int prevPrevIndex = (prevIndex - 1 + pointCount) % pointCount;
            Point prevPrevPoint = polygonCopy[prevPrevIndex];
            int nextNextIndex = (i + 1) % pointCount;
            Point nextNextPoint = polygonCopy[nextNextIndex];

            std::vector<std::tuple<Point, Point, Point, Polygon>> groups = {
                {prevPrevPoint, prevPoint, nextPoint, polygonCopy},
                {prevPoint, nextPoint, nextNextPoint, polygonCopy}
            };

            for (const auto& group : groups)
            {
                Point p = std::get<1>(group);
                if (isEar(std::get<0>(group), p, std::get<2>(group), std::get<3>(group)))
                {
                    auto earIt = std::find(earVertex.begin(), earVertex.end(), p);
                    if (earIt == earVertex.end())
                    {
                        earVertex.push_back(p);
                    }
                }
                else
                {
                    auto earIt = std::find(earVertex.begin(), earVertex.end(), p);
                    if (earIt != earVertex.end())
                    {
                        earVertex.erase(earIt);
                    }
                }
            }
        }
    }
}

double calculateTotalArea(const std::vector<Triangle>& triangles)
{
    double result = 0.0;
    for (const auto& triangle : triangles)
    {
        double sides[3];
        for (int i = 0; i < 3; i++)
        {
            int nextIndex = (i + 1) % 3;
            const Point& pt = (i == 0) ? triangle.p1 : ((i == 1) ? triangle.p2 : triangle.p3);
            const Point& pt2 = (nextIndex == 0) ? triangle.p1 : ((nextIndex == 1) ? triangle.p2 : triangle.p3);
            double side = std::sqrt(std::pow(pt2.x - pt.x, 2) + std::pow(pt2.y - pt.y, 2));
            sides[i] = side;
        }

        std::sort(std::begin(sides), std::end(sides));
        double a = sides[0];
        double b = sides[1];
        double c = sides[2];

        double area = 0.25 * std::sqrt(std::abs((a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c))));
        result += area;
    }

    return result;
}

int main()
{
    Polygons polygons = {
                        //  {{0.08, 0.08}, {0.08, 0.3}, {0.3, 0.3}, {0.3, 0.08}},
                        //  {{0.3, 0.5}, {0.3, 0.6}, {0.5, 0.5}, {0.4, 0.4}},
                         {
                            {1.00, 0.00},
                            {0.81, 0.59},
                            {0.31, 0.95},
                            {-0.31, 0.95},
                            {-0.81, 0.59},
                            {-1.00, 0.00},
                            {-0.81, -0.59},
                            {-0.31, -0.95},
                            {0.31, -0.95},
                            {0.81, -0.59}
                         }
                        };

    std::vector<Triangle> triangles = {};
    for (auto polygon : polygons)
    {
        std::vector<Triangle> triangles_ = {};
        earclip(polygon, &triangles_);
        triangles.insert(triangles.end(), triangles_.begin(), triangles_.end());
    }
    print_vector(triangles);

    return 0;
}