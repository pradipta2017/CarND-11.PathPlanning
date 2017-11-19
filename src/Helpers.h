#pragma once
#pragma once
#include <vector>
#include "spline.h"

using namespace std;

class Helpers
{
public:

	tk::spline s;

	Helpers();
	~Helpers();
	vector<double> interpolate_points(vector<double> x_points, vector<double> y_points, double interval, int output_size);
	vector<double> interpolate_points(vector<double> x_points, vector<double> y_points, vector<double> eval_at_x);
};

Helpers::Helpers() {}
Helpers::~Helpers() {}

vector<double> Helpers::interpolate_points(vector<double> x_points, vector<double> y_points,double interval, int output_size) {
	// uses the spline library to interpolate points connecting a series of x and y values
	// output is output_size number of y values beginning at y[0] with specified fixed interval

	if (x_points.size() != y_points.size()) {
		return { 0 };
	}

	
	s.set_points(x_points, y_points);    
	vector<double> output;
	for (int i = 0; i < output_size; i++) {
		output.push_back(s(x_points[0] + i * interval));
	}
	return output;
}

vector<double> Helpers::interpolate_points(vector<double> x_points, vector<double> y_points, vector<double> eval_at_x) {
	// uses the spline library to interpolate points connecting a series of x and y values
	// output is spline evaluated at each eval_at_x point

	if (x_points.size() != y_points.size()) {
		return { 0 };
	}

	s.set_points(x_points, y_points);    
	vector<double> output;
	for (double x : eval_at_x) {
		output.push_back(s(x));
	}
	return output;
}
