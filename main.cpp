#include <Eigen/Dense>
#include <iostream>
#include <tuple>
#include <math.h> 
#include <vector>
#include <map>
#include "Vertex.h"
using namespace Eigen;

// Method to create a temporary matrix with the given coordinates and x, y dimensions
Eigen::MatrixXd createTemporaryMatrix(std::vector<std::tuple<int, int, double>> objectWeights, int x, int y) {
	Eigen::MatrixXd tempCostmap;
	tempCostmap = MatrixXd::Constant(x, y, 0.0);
	for (int i = 0; i < objectWeights.size(); i++) {
		std::tuple<int, int, double> object = objectWeights.at(i);
		tempCostmap(std::get<0>(object), std::get<1>(object)) = std::get<2>(object);
	}
	return tempCostmap;
}

//Adds objects onto the existing costmap
void drawObjects(Eigen::MatrixXd& costmap, std::vector<std::tuple<int, int, double>> objectWeights) {
	Eigen::MatrixXd objectCostMap = createTemporaryMatrix(objectWeights, costmap.rows(), costmap.cols());
	costmap += objectCostMap;
	return;
}

//Adds a Bounding box to the cosmap
// Takes a (x,y) vector of vertices vertices (The four corners of the box)
// Takes a orientation tuple (x, y) coordinate
// Takes in maximumBoundings of existing costmap
// Takes in object weight of that bounding box
std::vector<std::tuple<int, int, double>> addBoundingBox(std::vector<std::tuple<float, float>> vertices, std::tuple<float, float> orientation, int maxXMatrix, int maxYMatrix, int objectWeight) {
	
	//Calculates the Angle from the orientation tuple
	double theta = atan(std::get<1>(orientation) / std::get<0>(orientation));
	
	//Generates a clockwise or counter-clockwise rotation matrix, to rotate these points so that the bounding box is horizontal 
	Matrix2f rotationMatrix;
	rotationMatrix << cos(theta), sin(theta),
						  -sin(theta), cos(theta);

	//Creates a matrix of the vertices
	MatrixXf vertexMatrix(2, 4);
	for (int i = 0; i < vertices.size(); i++) {
		std::tuple<float, float> vertex = vertices.at(i);
		vertexMatrix(0, i) = std::get<0>(vertex);
		vertexMatrix(1, i) = std::get<1>(vertex);
	}

	//Rotates the vertices
	vertexMatrix = rotationMatrix * vertexMatrix;
	//Calculating the min and max bounds of x and y of the horizontal bounding box
	double xMin = DBL_MAX;
	double xMax = DBL_MIN;
	double yMin = DBL_MAX;
	double yMax = DBL_MIN;

	for (int i = 0; i < vertices.size(); i++) {
		double x = vertexMatrix(0, i);
		double y = vertexMatrix(1, i);
		xMin = fmin(xMin, x);
		yMin = fmin(yMin, y);
		xMax = fmax(xMax, x);
		yMax = fmax(yMax, y);
	}

	double columns =((floor(xMax) - floor(xMin) + 1) * 2) +  ((floor(yMax) - floor(yMin) + 1) * 2);
	MatrixXf points(2,(int)ceil(columns));
	//Generating all the points of the entire horizontal portion of the bounding box boundary
	int k = 0;
	for (double i = 1; i < (floor(xMax)-floor(xMin)+1); i++) {
			points(0, k) = xMin+i;
			points(1, k) = yMin;
			k++;
			points(0, k) = xMin + i;
			points(1, k) = yMax;
			k++;
	}

	//Generating all the points of the entire vertical portion of the bounding box boundary
	for (double j = 1; j < (floor(yMax) - floor(yMin) + 1); j++) {
		points(0, k) = xMin;
		points(1, k) = yMin+j;
		k++;
		points(0, k) = xMax;
		points(1, k) = yMin + j;
		k++;
	}

	points.conservativeResize(2, k);
	//Rotating this boundary of points using the angle given to give the original bounding box points
	points = rotationMatrix.inverse() * points;
	//std::cout << points << std::endl;

	//Calculating the min and max for each x coordinate
	std::map<int, std::tuple<int, int>> rangePoints;
	for (int i = 0; i < points.cols(); i++) {
		int x = round(points(0, i));
		int y = round(points(1, i));

		int yMin = 0;
		int yMax = maxXMatrix;
			
		int xMin = 0;
		int xMax = maxYMatrix;

		if (x < xMin || x > xMax) {
			continue;
		}

		yMin = fmax(yMin, y);
		yMax = fmin(yMax, y);

		if (rangePoints.count(x) > 0) {
			std::tuple<int, int> minMax = rangePoints[x];
			yMin = fmin(std::get<0>(minMax), y);
			yMax = fmax(std::get<1>(minMax), y);
		}
		rangePoints[x] = std::make_tuple(yMin, yMax);
	}
	std::vector<std::tuple<int, int, double>> objectWeights;
	//Generating a Vector of x,y from the min and max of each coordinate
	for (auto const& point : rangePoints)
	{
		//std::cout << point.first << "," << std::get<0>(point.second) << std::endl;
		//std::cout << point.first << "," << std::get<1>(point.second) << std::endl;
		int xCoord = point.first;
		for (int i = std::get<0>(point.second); i <= std::get<1>(point.second); i++) {
			objectWeights.push_back(std::make_tuple(maxXMatrix - (i), xCoord - 1, objectWeight));
		}
	}
	return objectWeights;
}

int main() {
	int x, y;
	std::cout << "Enter Dimensions of Matrix" << std::endl;
	std::cin >> x >> y;

	std::cout << "Dimensions are: " << x << "x" << y << std::endl;

	Eigen::MatrixXd costmap;
	costmap = MatrixXd::Constant(x, y, 0.0);

	std::cout << costmap << std::endl;
	std::cout << "========================================" << std::endl;
	std::vector<std::tuple<int, int, double>> objectWeights;

	//while (true) {
	//	std::cout << "Enter Object Weights (x, y, weight): " << std::endl;
	//	int xCord, yCord;
	//	double weight;
	//	std::cin >> xCord >> yCord >> weight;
	//	if (xCord < 0) {
	//		break;
	//	}
	//	if (xCord < x && yCord < y) {
	//		std::tuple<int, int, double> objectTuple = std::make_tuple(xCord, yCord, weight);
	//		objectWeights.push_back(objectTuple);
	//	}
	//	else {
	//		std::cout << "X, Y Coordinates out of range. Please try again." << std::endl;
	//	}
	//	std::cout << "========================================" << std::endl;
	//}

	//Sample Bounding Box for testing
	std::tuple<float, float> v1 = std::make_tuple(2.464, 3.732);
	std::tuple<float, float> v2 = std::make_tuple(0.464, 7.196);
	std::tuple<float, float> v3 = std::make_tuple(5.062, 5.232);
	std::tuple<float, float> v4 = std::make_tuple(3.062, 8.696);
	std::tuple<float, float> orientation = std::make_tuple(sqrt(3), 1);

	//std::tuple<float, float> v1 = std::make_tuple(7.997, 4.851);
	//std::tuple<float, float> v2 = std::make_tuple(9.902, 3.751);
	//std::tuple<float, float> v3 = std::make_tuple(6.497, 2.253);
	//std::tuple<float, float> v4 = std::make_tuple(8.402, 1.153);
	//std::tuple<float, float> orientation = std::make_tuple(-sqrt(3), 1);

	std::vector<std::tuple<float, float>> vertices;
	vertices.emplace_back(v1);
	vertices.emplace_back(v2);
	vertices.emplace_back(v3);
	vertices.emplace_back(v4);
	objectWeights = addBoundingBox(vertices, orientation, x, y, 1);
	drawObjects(costmap, objectWeights);

	std::cout << costmap << std::endl;
	std::cin.get();
}
