#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "StateVector.h"

using namespace KF;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define TWOPI 6.28318530718

inline double NormalizeAngle(double ang)
{
	while (ang > M_PI / 2.0) ang -= TWOPI;
	while (ang <-M_PI / 2.0) ang += TWOPI;
//	return fmod(ang, TWOPI);
	return ang;
}


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF(StateVector *psv) : _pSV(psv)
{
	// Process noise standard deviation longitudinal acceleration in m/s^2
	double std_a_ = 1.0;

	// Process noise standard deviation yaw acceleration in rad/s^2
	double std_yawdd_ = 0.6;

	//std_a_squared_ = 0.2*0.2;
	//std_yawdd_squared_ = 0.2*0.2;

	//Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_squared_ = std_a_* std_a_;
	//Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_squared_ = std_yawdd_ * std_yawdd_;

	// State dimension
	n_x_ = psv->size();
	n_sig_pts_ = 2 * n_x_ + 1;

	n_z_ = 0;

	//* Augmented state dimension
	n_aug_ = n_x_+2;

	n_sig_pts_aug_ = 2 * n_aug_ + 1;

//	lambda_ = 3 - n_x_;
	lambda_ = 3 - n_aug_;

	// Initialize weights
	double wt = 0.5 / (n_aug_ + lambda_);
	weights_ = VectorXd::Constant(n_sig_pts_aug_,wt);
	weights_(0) = lambda_ / (lambda_ + n_aug_);

	// Initialize temporary variables
	Xsig_pred_ = MatrixXd(n_x_, n_sig_pts_aug_);

	x_aug_ = VectorXd(n_aug_);
	P_aug_ = MatrixXd(n_aug_, n_aug_);

	Xsig_pts_ = MatrixXd::Zero(n_aug_, n_sig_pts_aug_);
	x_pred_mean_ = VectorXd(n_x_);
	P_pred_covar_ = MatrixXd(n_x_, n_x_);
}


void UKF::ProcessMeasurement(const MeasurementPackage &meas_package)
{
	// Processes the measurement based on sensor type
	// in this derived method
	// Handles first measurement and loads a vector with the measurement.
	if (InitializeMeasurement(meas_package)) return;

	if (!ProcessData()) return;

	float delta_t = (float) (meas_package.timestamp_ - _pSV->previous_timestamp_) / 1000000.0f;
	_pSV->previous_timestamp_ = meas_package.timestamp_;

	Prediction(delta_t);


	Update(meas_package);


}


void UKF::Prediction(const double delta_t)
{
	// Generate Sigma Points
	AugmentedSigmaPoints();

	// Predict Points using process model
	SigmaPointPrediction(delta_t);

	// Average predicted sigma points to get
	// the predicted mean and covariance
	PredictMeanAndCovariance();
}

void UKF::AugmentedSigmaPoints()
{
	//create augmented mean state
	x_aug_.head(n_x_) = _pSV->x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;

	//create augmented covariance matrix
	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(n_x_, n_x_) = this->_pSV->P_;
	P_aug_(5, 5) = std_a_squared_;
	P_aug_(6, 6) = std_yawdd_squared_;

	//create square root matrix
	MatrixXd L = P_aug_.llt().matrixL();
	Eigen::LLT<MatrixXd> lltOfPaug(P_aug_);
	if (lltOfPaug.info() == Eigen::NumericalIssue) {
		cout << "numerical " << endl;
	}

	//create augmented sigma points
	Xsig_pts_.col(0) = x_aug_;
	double alpha = sqrt(lambda_ + n_aug_);
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_pts_.col(i + 1)          = x_aug_ + alpha * L.col(i);
		Xsig_pts_.col(i + 1 + n_aug_) = x_aug_ - alpha * L.col(i);
	}
}

/*
* Propagate Sigma Points with Solution to equations of motion.
*/
void UKF::SigmaPointPrediction(const double delta_t)
{
	//predict sigma points
	for (int i = 0; i< n_sig_pts_aug_; i++)
	{
		//extract values for better readability
		double p_x  = Xsig_pts_(0, i);
		double p_y  = Xsig_pts_(1, i);
		double v    = Xsig_pts_(2, i);
		double yaw  = NormalizeAngle(Xsig_pts_(3, i));
		double yawd = Xsig_pts_(4, i);
		double nu_a = Xsig_pts_(5, i);
		double nu_yawdd = Xsig_pts_(6, i);

		//predicted state values
		double px_p;
		double py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		}
		else {
			px_p = p_x + v * delta_t*cos(yaw);
			py_p = p_y + v * delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		double dt2 = delta_t * delta_t;
		//add noise
		px_p = px_p + 0.5*nu_a*dt2 * cos(yaw);
		py_p = py_p + 0.5*nu_a*dt2 * sin(yaw);
		v_p = v_p + nu_a * delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*dt2;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = NormalizeAngle(yaw_p);
		Xsig_pred_(4, i) = yawd_p;
	}
}

void UKF::PredictMeanAndCovariance()
{
	//predicted state mean
	x_pred_mean_ = Xsig_pred_ * weights_; // (n_x_, n_aug_) * (n_aug_)

	//predicted state covariance matrix
	P_pred_covar_.fill(0.0);

	//iterate over sigma points
	for (int i = 0; i < n_sig_pts_aug_ ; i++) {
		// state difference
		x_diff_ = Xsig_pred_.col(i) - x_pred_mean_;

		x_diff_(3) = NormalizeAngle(x_diff_(3));

		P_pred_covar_ = P_pred_covar_ + weights_(i) * x_diff_ * x_diff_.transpose();
	}
}


// Update the state based on difference between Predicted Measurement and actual Measurement
/*
* Update
* Based on predicted sigma points
* 1) Compute the predicted in measurement space
* 2) Combine with actual measurment via Kalman Filter algorithm
* 3) Update the state vector with mean and covariance
* 4) Compute NIS and updat state vector
* Input:
*	meas - measurement for sensor
*/
void UKF::Update(const MeasurementPackage& meas)
{
	if (!ProcessData())
	{
		_pSV->x_ = this->x_pred_mean_;
		_pSV->P_ = this->P_pred_covar_;
		return;
	}

	/*
	* Transform to measurement space fomr the predicted sigma points in state space
	*/
	MatrixXd Zsig_pred = MatrixXd(n_z_, n_sig_pts_aug_);

	// Convert to measurment is implemented by derived method specify to sensor type
	Convert2MeasurementSpace(Xsig_pred_, Zsig_pred);

	// Compute the mean of the predicted measurement based on predict sigma points
	z_pred_ = Zsig_pred * weights_;

	// Compute the innovation covariance and the cross correlation matrix
	// of the predicted measurement based on predict sigma points
	S_.fill(0.0);
	Tc_.fill(0.0);
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		//residual
		z_diff_ = Zsig_pred.col(i) - z_pred_;

		// Angle normalization in measurement space
		NormalizeMeasurementSpaceAngles(z_diff_);

		// Save transpose
		z_diff_transpose = z_diff_.transpose();

		// Weight contribution for current sigma point
		S_ = S_ + weights_(i) * z_diff_ * z_diff_transpose;

		// predicted sigma point difference with mean
		x_diff_ = Xsig_pred_.col(i) - x_pred_mean_;

		// Angle normalization of state angle variables
		x_diff_(3) = NormalizeAngle(x_diff_(3));

		//Tc_ = MatrixXd(n_x_, n_z_); // GetNZ
		Tc_ = Tc_ + weights_(i) * x_diff_ * z_diff_transpose;
	}

	// Add measurement noise covariance matrix to innovation
	S_ = S_ + R_;

	//Kalman gain K;
	MatrixXd K = Tc_ * S_.inverse();

	// Residual between the actual measurement and the predicted measurement
	zm_ = meas.raw_measurements_;
	z_diff_ = zm_ - z_pred_;

	// Angle normalization
	NormalizeMeasurementSpaceAngles(z_diff_);

	//update state mean and covariance matrix
	_pSV->x_ = x_pred_mean_ + K * z_diff_;
	_pSV->P_ = P_pred_covar_ - K * S_ * K.transpose();

	// NIS
	_pSV->nis_ = z_diff_.transpose() * S_.inverse() * z_diff_;
}

/**  Radar UKF Class

*/


/*
//DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
// Radar measurement noise standard deviation radius in m
std_radr_ = 0.3;

// Radar measurement noise standard deviation angle in rad
std_radphi_ = 0.03;

// Radar measurement noise standard deviation radius change in m/s
std_radrd_ = 0.3;
*/
UKFRadar::UKFRadar(StateVector *pSV) : UKF(pSV), std_radr_(0.3),
std_radphi_(0.03), std_radrd_(0.3), process_data(true)
{
	// Degrees of freedom in radar data
	n_z_ = 3;

	//add measurement noise covariance matrix
	VectorXd tmp(n_z_);
	tmp << std_radr_ * std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_;
	R_ = tmp.asDiagonal();

	zm_ = VectorXd(n_z_);
	z_pred_ = VectorXd(n_z_);
	Tc_ = MatrixXd(n_x_, n_z_);
	S_ = MatrixXd(n_z_, n_z_);
}

bool UKFRadar::InitializeMeasurement(const MeasurementPackage &meas_package)
{
	if (!_pSV->is_initialized_)
	{
		double r = meas_package.raw_measurements_(0);
		double theta = meas_package.raw_measurements_(1);
		double rdot = meas_package.raw_measurements_(2);

		double px = r * cos(theta);
		double py = r * sin(theta);
		//double vx = rdot * cos(theta);
		//double vy = rdot * sin(theta);

		double v = fabs(rdot);
		double phi = theta;  // yaw rotation about axis perpendicular to plane
		double phidot = 0;

		this->_pSV->x_ << px, py, v, phi, phidot;
		this->_pSV->previous_timestamp_ = meas_package.timestamp_;
		_pSV->is_initialized_ = true;
		return true;
	}
	return false;
}

void UKFRadar::Convert2MeasurementSpace(const MatrixXd &Xsig_pred, MatrixXd &Zsig_pred)
{
	// Transform predicted sigma points into measurement space
	// 2n+1 sigma points
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		// Extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);
		double v = Xsig_pred(2, i);
		double yaw = Xsig_pred(3, i);
		double yawdot = Xsig_pred(4, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		double r = sqrt(p_x*p_x + p_y * p_y);
		if (r < 0.00000001) throw ("Predicted sigma point radius zero!");
		Zsig_pred(0, i) = r;//r
		Zsig_pred(1, i) = atan2(p_y, p_x);					//phi
		Zsig_pred(2, i) = (p_x*v1 + p_y*v2) / r;   //r_dot
	}

}

void UKFRadar::NormalizeMeasurementSpaceAngles(VectorXd& z_diff)
{
	z_diff(1) = NormalizeAngle(z_diff(1));
}


UKFRadar::~UKFRadar()
{

}

/**  Laser UKF Class

*/
/**  Constructor - initialize sensor and model parameters for laser
**					and temporary work vectors/matrices
*/
UKFLaser::UKFLaser(StateVector *pSV) : UKF(pSV), process_data(true)
{
	// Number of dimensions in measurement space
	n_z_ = 2;

	//DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Setup the measurement noise covariance matrix
	VectorXd tmp(n_z_);
	tmp << std_laspx_ * std_laspx_, std_laspy_*std_laspy_;
	R_ = tmp.asDiagonal();

	// Initialize workspace variables
	zm_ = VectorXd(n_z_);
	z_pred_ = VectorXd(n_z_);
	Tc_ = MatrixXd(n_x_, n_z_);
	S_ = MatrixXd(n_z_, n_z_);
}

UKFLaser::~UKFLaser()
{

}

void UKFLaser::Convert2MeasurementSpace(const MatrixXd &Xsig_pred, MatrixXd &Zsig_pred)
{
	// Transform predicted sigma points into measurement space
	for (int i = 0; i < n_sig_pts_aug_; i++)
	{
		// Extract values for better readibility
		double p_x = Xsig_pred(0, i);
		double p_y = Xsig_pred(1, i);

		// measurement model, H, for laser
		Zsig_pred(0, i) = p_x;
		Zsig_pred(1, i) = p_y;
	}
}



bool UKFLaser::InitializeMeasurement(const MeasurementPackage &meas_package)
{
	if (!_pSV->is_initialized_)
	{
		double px = meas_package.raw_measurements_(0);
		double py = meas_package.raw_measurements_(1);
		double v = 0;
		double yaw = 0;
		double yaw_dot = 0;
		_pSV->x_ << px, py, v, yaw, yaw_dot;
		_pSV->previous_timestamp_ = meas_package.timestamp_;
		_pSV->is_initialized_ = true;

		cout << "Initial P" << endl;
		cout << _pSV->P_;
		cout << endl;

		return true;
	}
	return false;

}


void UKFLaserHybrid::Update(const MeasurementPackage &m)
{
	if(!ProcessData()) 
	{
		_pSV->x_ = this->x_pred_mean_;
		_pSV->P_ = this->P_pred_covar_;
		return;
	}

	const VectorXd & zm = m.raw_measurements_;

}