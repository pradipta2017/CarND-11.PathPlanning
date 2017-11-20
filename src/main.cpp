#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "Helpers.h"
//#include "vehicle.cpp"
#include "vehicle.h"
#include "CostFunctions.hpp"


//#ifndef M_PI
//#define M_PI (3.14159265358979323846)
//#endif

using namespace std;
using Eigen::ArrayXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
//constexpr double pi() { return M_PI; }
constexpr double pi() { return 22/7; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2( (map_y-y),(map_x-x) );
	double angle = abs(theta-heading);
	angle = min(2 * pi() - angle, angle); // XXX bug fix

	if(angle > pi()/4)
	{
		closestWaypoint++;
		if (closestWaypoint == maps_x.size())
		{
			closestWaypoint = 0; // XXX bug fix
		}
	}
	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
	
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate s value
	double frenet_s = maps_s[0];
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	int wp2 = (prev_wp+1)%maps_x.size();
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
	double perp_heading = heading-pi()/2;
	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};
}


int main() {
	  uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	Vehicle ego_car = Vehicle();

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
  

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	ofstream log_file;
	log_file.open("path_planning_log.txt");

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego_car, &log_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];
					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];
					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					/////code begin

					Helpers helper_func;

					///////////////////////// construct some closeby waypoints  //////////////////////////////
					vector<double> closeby_waypoints_s, closeby_waypoints_x, closeby_waypoints_y, closeby_waypoints_dx, closeby_waypoints_dy;
					vector<double> temp_waypoints_s, temp_waypoints_x, temp_waypoints_y, temp_waypoints_dx, temp_waypoints_dy;

					int num_waypoints = map_waypoints_x.size();
					int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);

					// take 10 points before and after waypoints
					for (int i = -5; i < 5; i++) {
						int idx = (next_waypoint_index + i) % num_waypoints;
						if (idx < 0) {
							idx += num_waypoints;
						}
						double current_s = map_waypoints_s[idx];
						double base_s = map_waypoints_s[next_waypoint_index];
						if (i < 0 && current_s > base_s) {
							current_s -= TRACK_LENGTH;
						}
						if (i > 0 && current_s < base_s) {
							current_s += TRACK_LENGTH;
						}
						temp_waypoints_s.push_back(current_s);
						temp_waypoints_x.push_back(map_waypoints_x[idx]);
						temp_waypoints_y.push_back(map_waypoints_y[idx]);
						temp_waypoints_dx.push_back(map_waypoints_dx[idx]);
						temp_waypoints_dy.push_back(map_waypoints_dy[idx]);
					}


					double dist_inc = 0.5;
					int num_interpolation_points = (temp_waypoints_s[temp_waypoints_s.size() - 1] - temp_waypoints_s[0]) / dist_inc;

					closeby_waypoints_s.push_back(temp_waypoints_s[0]);
					for (int i = 1; i < num_interpolation_points; i++) {
						closeby_waypoints_s.push_back(temp_waypoints_s[0] + i * dist_inc);
					}
					closeby_waypoints_x = helper_func.interpolate_points(temp_waypoints_s, temp_waypoints_x, dist_inc, num_interpolation_points);
					closeby_waypoints_y = helper_func.interpolate_points(temp_waypoints_s, temp_waypoints_y, dist_inc, num_interpolation_points);
					closeby_waypoints_dx = helper_func.interpolate_points(temp_waypoints_s, temp_waypoints_dx, dist_inc, num_interpolation_points);
					closeby_waypoints_dy = helper_func.interpolate_points(temp_waypoints_s, temp_waypoints_dy, dist_inc, num_interpolation_points);

					int prev_path_size = min(MIN_PATH_SIZE, (int)previous_path_x.size());
					double traj_start_time = prev_path_size * EACH_POINT_TIME;

					vector<double> x_points, y_points, s_points;

					double ego_s, ego_s_dot, ego_s_ddot;
					double ego_d, ego_d_dot, ego_d_ddot;

					double pos_x, pos_y, pos_x2, pos_y2, angle, vel_x1, vel_y1,
								 pos_x3, pos_y3, vel_x2, vel_y2, acc_x, acc_y;


					// When there are no previous points
					if (prev_path_size < 2) {
						pos_x = car_x;
						pos_y = car_y;
						angle = deg2rad(car_yaw);
						ego_s = car_s;
						ego_s_dot = car_speed*0.224; //converted to m/s
						ego_s_ddot = 0;
						ego_d = car_d;
						ego_d_dot = 0;
						ego_d_ddot = 0;

						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						x_points.push_back(prev_car_x);
						x_points.push_back(car_x);

						y_points.push_back(prev_car_y);
						y_points.push_back(car_y);

						double prev_s = ego_s - 1;
						s_points.push_back(prev_s);
						s_points.push_back(ego_s);

					} else {

						// consider current position to be last point of previous path to be kept
						pos_x = previous_path_x[prev_path_size -1];
						pos_y = previous_path_y[prev_path_size -1];
						pos_x2 = previous_path_x[prev_path_size -2];
						pos_y2 = previous_path_y[prev_path_size -2];

						// calculate s_dot & d_dot
						vel_x1 = (pos_x - pos_x2) / EACH_POINT_TIME;
						vel_y1 = (pos_y - pos_y2) / EACH_POINT_TIME;

						angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

						pos_x3 = previous_path_x[prev_path_size - 3];
						pos_y3 = previous_path_y[prev_path_size - 3];

						vel_x2 = (pos_x2 - pos_x3) / EACH_POINT_TIME;
						vel_y2 = (pos_y2 - pos_y3) / EACH_POINT_TIME;
						acc_x = (vel_x1 - vel_x2) / EACH_POINT_TIME;
						acc_y = (vel_y1 - vel_y2) / EACH_POINT_TIME;

						vector<double> frenet = getFrenet(pos_x, pos_y, angle, closeby_waypoints_x, closeby_waypoints_y, closeby_waypoints_s);
						ego_s = frenet[0];
						ego_d = frenet[1];

						// determine dx, dy vector from set of closeby waypoints, with pos_x,pos_y as reference point;
						int next_interp_waypoint_index = NextWaypoint(pos_x, pos_y, angle, closeby_waypoints_x, closeby_waypoints_y);
						double dx = closeby_waypoints_dx[next_interp_waypoint_index - 1];
						double dy = closeby_waypoints_dy[next_interp_waypoint_index - 1];

						// sx,sy vector is perpendicular to dx,dy
						double sx = -dy;
						double sy = dx;

						ego_s_dot = vel_x1 * sx + vel_y1 * sy;
						ego_s_ddot = acc_x * sx + acc_y * sy;

						ego_d_dot = vel_x1 * dx + vel_y1 * dy;
						ego_d_ddot = acc_x * dx + acc_y * dy;		

						x_points.push_back(pos_x2);
						x_points.push_back(pos_x);

						y_points.push_back(pos_y2);
						y_points.push_back(pos_y);

						double prev_s = ego_s - ego_s_dot * EACH_POINT_TIME;

						s_points.push_back(prev_s);
						s_points.push_back(ego_s);
					}		
					
					ego_car.s    = ego_s;           
					ego_car.s_d  = ego_s_dot;           
					ego_car.s_dd = ego_s_ddot;          
					ego_car.d    = ego_d;           
					ego_car.d_d  = ego_d_dot;           
					ego_car.d_dd = ego_d_ddot;          
					

					// ********************* GENERATE PREDICTIONS FROM SENSOR FUSION DATA [ id, x, y, vx, vy, s, d] **************************
					double goal_t = HORIZON - prev_path_size * EACH_POINT_TIME;
					vector<Vehicle> non_ego_cars;
					map<int, vector<vector<double>>> predictions;
					for (auto lsf: sensor_fusion) {
						double nego_car_vel = sqrt(pow((double)lsf[3], 2) + pow((double)lsf[4], 2));
						Vehicle non_ego_car = Vehicle(lsf[5], nego_car_vel, 0, lsf[6], 0, 0);
						non_ego_cars.push_back(non_ego_car);
						int v_id = lsf[0];
						vector<vector<double>> preds;
						for (int i = 0; i < N_SAMPLES; i++) {
							double t = traj_start_time + (i * goal_t / N_SAMPLES);
							double pred_s = (double)lsf[5] + nego_car_vel * t;
							vector<double> pred_s_d;
							pred_s_d = { pred_s, lsf[6] };
							preds.push_back(pred_s_d);
						}
						predictions[v_id] = preds;
					}
					

					bool car_to_left = false, car_to_right = false, car_just_ahead = false;
					
					for (Vehicle other_car: non_ego_cars) {
						double s_diff = fabs(other_car.s - car_s);
						if (s_diff < FOLLOW_DISTANCE) {
							double d_diff = other_car.d - car_d;
							if (d_diff > 2 && d_diff < 6) {
								car_to_right = true;
							} else if (d_diff < -2 && d_diff > -6) {
								car_to_left = true;
							} 
							else if (d_diff > -2 && d_diff < 2) {
								car_just_ahead = true;
							}
						}
					}
					

					int my_lane = (int)ego_car.d / 4;
					ego_car.available_states = { "KL" };
					if (my_lane == 0 && (car_to_right == false)) {
						ego_car.available_states.push_back("LCR");
					}
					else if (my_lane == 1) {
						if (car_to_right == false) {
							ego_car.available_states.push_back("LCR");
						}
						if (car_to_left == false) {
							ego_car.available_states.push_back("LCL");
						}
					}
					else if (my_lane == 2 && (car_to_left == false)) {
						ego_car.available_states.push_back("LCL");
					}
					
					vector<vector<double>> best_trajectory ;
					ArrayXd best_target(6);
					double best_cost = 999999;
					string best_traj_state = "";
					
					for (string state: ego_car.available_states) {
						ArrayXd target_state = ego_car.get_target_for_state(state, predictions, goal_t);
						vector<vector<double>> possible_traj = ego_car.generate_traj_for_target(target_state, goal_t);

						double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], target_state, predictions);

						if (current_cost < best_cost) {
								best_cost = current_cost;
								best_trajectory = possible_traj;
								best_traj_state = state;
								best_target = target_state;
						}
					}
					
					// emergency break
					if (car_just_ahead) { 
						best_target[1] = 0.0; 
					} 

					vector<double> s_points_next, x_points_next, y_points_next;

					
					double target_s1 = ego_s + 30;
					double target_d1 = best_target[3];
					vector<double> target_xy1 = getXY(target_s1, target_d1, closeby_waypoints_s, closeby_waypoints_x, closeby_waypoints_y);
					double target_x1 = target_xy1[0];
					double target_y1 = target_xy1[1];
					s_points.push_back(target_s1);
					x_points.push_back(target_x1);
					y_points.push_back(target_y1);

					double target_s2 = ego_s + 60;
					double target_d2 = target_d1;
					vector<double> target_xy2 = getXY(target_s2, target_d2, closeby_waypoints_s, closeby_waypoints_x, closeby_waypoints_y);
					double target_x2 = target_xy2[0];
					double target_y2 = target_xy2[1];
					s_points.push_back(target_s2);
					x_points.push_back(target_x2);
					y_points.push_back(target_y2);

					
					double target_s_dot = best_target[1];

					double current_s = ego_s;
					double current_v = ego_s_dot;
					double v_incr;
					for (int i = 0; i < (50 - prev_path_size); i++) {
						
						if (fabs(target_s_dot - current_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
							v_incr = 0;
						} else {
	
							v_incr = (target_s_dot - current_v)/(fabs(target_s_dot - current_v)) * VELOCITY_INCREMENT_LIMIT;
						}
						current_v += v_incr;
						current_s += current_v * EACH_POINT_TIME;
						s_points_next.push_back(current_s);
					}

					x_points_next = helper_func.interpolate_points(s_points, x_points, s_points_next);
					y_points_next = helper_func.interpolate_points(s_points, y_points, s_points_next);


					// add previous path, if any, to next path
					for(int i = 0; i < prev_path_size; i++) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					} 
					// add xy points from newly generated path
					for (int i = 0; i < x_points_next.size(); i++) {
						next_x_vals.push_back(x_points_next[i]);
						next_y_vals.push_back(y_points_next[i]);
					} 


					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					
					
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  //if (h.listen(port)) {
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

	log_file.close();
}