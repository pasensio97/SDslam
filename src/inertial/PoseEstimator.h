
#include <Eigen/Dense>

/*
const double COV_A_2 = 0.000625;  // 0.025^2
const double SIGMA_GYRO = 2.60; // rad/s^2
const double SIGMA_ACC = 8.94;  // m/s^3
*/

class PoseEstimator{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseEstimator();

  Eigen::VectorXd predict(double dt);
  void update(const Eigen::VectorXd & measurements);
  const Eigen::VectorXd state_vector();

	Eigen::MatrixXd f(Eigen::MatrixXd & x, double dt);
  Eigen::MatrixXd jacobian_f(const Eigen::MatrixXd & x, double dt);
  Eigen::MatrixXd h(const Eigen::MatrixXd & x, double dt);
  Eigen::MatrixXd jacobian_h(Eigen::MatrixXd & x, double dt);

	Eigen::MatrixXd R_update(double time);
	Eigen::MatrixXd Q_update(double time);
	Eigen::MatrixXd jacobian_Q(double time);

	void print_state_vector();
	
	Eigen::Vector3d get_position();

 private:
	
	int dim_state_vector = 9;
	int dim_measurements = 3;

	double dt;  // k in the literature
	bool initilized;
	Eigen::MatrixXd x, y, z;
	Eigen::MatrixXd F, P, Q;
	Eigen::MatrixXd H, S, K, R;
	//Eigen::MatrixXd Id;

};