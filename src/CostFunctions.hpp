#ifndef COSTS
#define COSTS

#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;


double logistic(double x){
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in 
  // the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {
  double closest = 999999;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}


double nearest_approach_to_any_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = 999999;
  for (auto prediction : predictions) {
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size() - 1][1];
    int pred_lane = pred_final_d / 4;
    if (my_lane == pred_lane) {
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_dist < closest && current_dist < 120) {
        closest = current_dist;
      }
    }
  }
  return closest;
}


vector<double> velocities_for_trajectory(vector<double> traj) {

	vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / TIME_DIFF);
  }
  return velocities;
}



double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  double cost = logistic(FOLLOW_DISTANCE-nearest);
  if (nearest < 2 * VEHICLE_RADIUS) {
	  return cost;
  } else { 
    return 0;
  }
}

double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

double in_lane_buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Penalizes getting close to other vehicles.
  double nearest = nearest_approach_to_any_vehicle_in_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}



double efficiency_cost(vector<double> s_traj) {
  // Rewards high average speeds.
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  double final_s_dot, total = 0;

  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
} 

double d_diff_cost(vector<double> d_traj, ArrayXd target_state) {

	double target_d = target_state[3];
	double diff = 0;
	for (auto each : d_traj) {
		diff += pow(fabs(target_d - each), 2);
	}

	double s_dev = sqrt(diff) / d_traj.size();

	return logistic(s_dev);
}

double avoid_right_lane(ArrayXd target_state) {
	if (target_state[3] >= 10) {
		return 1.0;
	}

	return 0;
}

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, ArrayXd target_state, map<int,vector<vector<double>>> predictions) {
	
	double total_cost = 0;
	double cost_collision = 0, cost_buffer = 0, cost_in_lane_buffer = 0, cost_effciency = 0, nml = 0, cost_d_diff = 0, cost_right_lane = 0;

	cost_collision = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
	cost_buffer = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
	cost_in_lane_buffer = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
	cost_effciency = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
	cost_d_diff = d_diff_cost(d_traj, target_state) * D_DIFF_COST_WEIGHT;
	cost_right_lane = avoid_right_lane(target_state) * RIGHT_LANE_COST_WEIGHT;
	
	total_cost = cost_collision + cost_buffer + cost_effciency + cost_d_diff + cost_right_lane ;
	return total_cost;
}

#endif