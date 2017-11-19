#include "vehicle.h"
#include "constants.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"




/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

  this->s    = s;         // s position
  this->s_d  = s_d;       // s dot - velocity in s
  this->s_dd = s_dd;      // s dot-dot - acceleration in s
  this->d    = d;         // d position
  this->d_d  = d_d;       // d dot - velocity in d
  this->d_dd = d_dd;      // d dot-dot - acceleration in d

}

Vehicle::~Vehicle() { }


ArrayXd Vehicle::get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration) {

	int target_lane, current_lane = this->d / 4; 
	double target_d = 0, target_d_d = 0, target_d_dd = 0;

	double target_s = 0, target_s_d = SPEED_LIMIT, target_s_dd = 0;
	double car_in_front_s = 9999, car_in_front_sdot = 0;

	target_s = this->s + (this->s_d + target_s_d) / 2 * duration;

	if(state.compare("KL") == 0) {
		target_d = (double)current_lane * 4 + 2;
		target_lane = target_d / 4;
	}
	else if(state.compare("LCL") == 0) {
		target_d = ((double)current_lane - 1) * 4 + 2;
		target_lane = target_d / 4;
	}
	else if(state.compare("LCR") == 0) {
		target_d = ((double)current_lane + 1) * 4 + 2;
		target_lane = target_d / 4;
	}
  
  
	for (auto prediction : predictions) {
		vector<vector<double>> pred_traj = prediction.second;
		int pred_lane = (int)pred_traj[0][1] / 4;
		if (pred_lane == target_lane) {
			double start_s = pred_traj[0][0];
			double predicted_end_s = pred_traj[pred_traj.size() - 1][0];
			double next_to_last_s = pred_traj[pred_traj.size() - 2][0];
			double dt = duration / N_SAMPLES;
			double predicted_s_dot = (predicted_end_s - next_to_last_s) / dt;
			if (predicted_end_s < car_in_front_s && start_s > this->s) {
				car_in_front_s = predicted_end_s;
				car_in_front_sdot = predicted_s_dot;
			}
		}
	}

	if (car_in_front_s - target_s < FOLLOW_DISTANCE && car_in_front_s > this->s) {

		target_s_d = car_in_front_sdot;
		if (fabs(car_in_front_s - target_s) < 0.5 * FOLLOW_DISTANCE) {
			target_s_d -= 1; // slow down if too close
		}
		target_s = car_in_front_s - FOLLOW_DISTANCE;
	}

	ArrayXd target_state(6);
	target_state << target_s, target_s_d, target_s_dd, target_d, target_d_d, target_d_dd;

	return target_state;
}


vector<vector<double>> Vehicle::generate_traj_for_target(ArrayXd target, double duration) {
	// takes a target {s, s_dot, s_ddot, d, d_dot, d_ddot} and returns a Jerk-Minimized Trajectory
	// (JMT) connecting current state (s and d) to target state in a list of s points and a list of d points

	Array3d start_s, start_d, end_s, end_d;
	start_s << this->s, this->s_d, this->s_dd;
	start_d << this->d, this->d_d, this->d_dd;
	end_s << target[0], target[1], target[2];
	end_d << target[3], target[4], target[5];

	this->s_traj_coeffs = JMT(start_s, end_s, duration);
	this->d_traj_coeffs = JMT(start_d, end_d, duration);

	vector<double> s_traj;
	vector<double> d_traj;

	// populate s and t trajectories at each time step
	for (int i = 0; i < N_SAMPLES; i++) {
		double t = i * duration / N_SAMPLES;
		double s_val = 0, d_val = 0;
		for (int j = 0; j < s_traj_coeffs.size(); j++) {
			s_val += this->s_traj_coeffs[j] * pow(t, j);
			d_val += this->d_traj_coeffs[j] * pow(t, j);
		}
		s_traj.push_back(s_val);
		d_traj.push_back(d_val);
	}
	return { s_traj, d_traj };
}

ArrayXd Vehicle::JMT(ArrayXd start, ArrayXd end, double goal_t) {
	/*
	Calculates Jerk Minimizing Trajectory for start, end and T.
	*/

	ArrayXd alphas(6);
	alphas[0] = start[0];
	alphas[1] = start[1];
	alphas[2] = start[2] / 2.0;
	
	MatrixXd A = MatrixXd(3, 3);
	A << goal_t*goal_t*goal_t, goal_t*goal_t*goal_t*goal_t, goal_t*goal_t*goal_t*goal_t*goal_t,
		3 * goal_t*goal_t, 4 * goal_t*goal_t*goal_t, 5 * goal_t*goal_t*goal_t*goal_t,
		6 * goal_t, 12 * goal_t*goal_t, 20 * goal_t*goal_t*goal_t;

	MatrixXd B = MatrixXd(3, 1);
	B << end[0] - (start[0] + start[1] * goal_t + .5*start[2] * goal_t*goal_t),
		end[1] - (start[1] + start[2] * goal_t),
		end[2] - start[2];
	
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;

	alphas[3] = C(0, 0);
	alphas[4] = C(1, 0);
	alphas[5] = C(2, 0);
	
	return alphas;
}
