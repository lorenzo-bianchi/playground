#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

double PerpendicularDistance(const Eigen::Vector2d &pt,
                             const Eigen::Vector2d &lineStart,
                             const Eigen::Vector2d &lineEnd)
{
	Eigen::Vector2d dx = lineEnd - lineStart;
	dx.normalize();

	Eigen::Vector2d pv = pt - lineStart;

	// Get dot product
	double pvdot = dx.dot(pv);

	// Scale line direction vector
	Eigen::Vector2d ds = pvdot * dx;

	// Subtract from pv
	Eigen::Vector2d a = pv - ds;

	return a.norm();
}

void RamerDouglasPeucker(const std::vector<Eigen::Vector2d> &pointList, double epsilon, std::vector<Eigen::Vector2d> &out)
{
	if (pointList.size()<2) throw std::invalid_argument("Not enough points to simplify");

	// Find the point with maximum distance from line between start and end
	double dmax = 0.0;
	size_t index = 0;
	size_t end = pointList.size()-1;
	for (size_t i = 1; i < end; i++)
	{
		double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
		if (d > dmax)
		{
			index = i;
			dmax = d;
		}
	}

	// If max distance is greater than epsilon, recursively simplify
	if (dmax > epsilon)
	{
		// Recursive call
		std::vector<Eigen::Vector2d> recResults1;
		std::vector<Eigen::Vector2d> recResults2;
		std::vector<Eigen::Vector2d> firstLine(pointList.begin(), pointList.begin()+index+1);
		std::vector<Eigen::Vector2d> lastLine(pointList.begin()+index, pointList.end());
		RamerDouglasPeucker(firstLine, epsilon, recResults1);
		RamerDouglasPeucker(lastLine, epsilon, recResults2);

		// Build the result list
		out.assign(recResults1.begin(), recResults1.end()-1);
		out.insert(out.end(), recResults2.begin(), recResults2.end());
		if (out.size() < 2) throw std::runtime_error("Problem assembling output");
	}
	else
	{
		// Return start and end points
		out.clear();
		out.push_back(pointList[0]);
		out.push_back(pointList[end]);
	}
}

int main()
{
	std::vector<Eigen::Vector2d> pointsIn = {
                                                {0.0, 0.0}, {3.0, 8.0}, {5.0, 2.0},
                                                {5.0, 4.0}, {6.0, 20.0}, {6.4, 15.5},
                                                {7.0, 25.0}, {9.1, 16.9}, {10.0, 10.0},
                                                {11.0, 5.5}, {17.3, 3.2}, {27.8, 0.1}
                                            };
	std::vector<Eigen::Vector2d> pointsOut;

	RamerDouglasPeucker(pointsIn, 1.0, pointsOut);

	for (size_t i = 0; i < pointsOut.size(); i++)
		std::cout << i+1 << ") \t" << pointsOut[i][0] << " \t" << pointsOut[i][1] << std::endl;

	return 0;
}