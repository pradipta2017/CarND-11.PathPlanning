#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"


using Eigen::ArrayXd;
using Eigen::Array3d;
using Eigen::MatrixXd;

using namespace std;

class Vehicle {
public:

	double s;
	double s_d;
	double s_dd;
	double d;
	double d_d;
	double d_dd;
	vector<string> available_states;
	ArrayXd s_traj_coeffs, d_traj_coeffs;


	/**
	* Constructors
	*/
	Vehicle();
	Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

	/**
	* Destructor
	*/
	virtual ~Vehicle();


	ArrayXd get_target_for_state(string state, map<int, vector<vector<double>>> predictions, double duration);
	vector<vector<double>> generate_traj_for_target(ArrayXd target, double duration);
	ArrayXd JMT(ArrayXd start, ArrayXd end, double goal_t);
};