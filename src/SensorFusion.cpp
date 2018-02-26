#include "SensorFusion.h"
#include "measurement_package.h"
#include "ukf.h"
#include "KFEnums.h"
#include "KalmanFilterFactory.h"
#include <map>
#include <stdio.h>

// State vector (px, py, v, yaw, yaw_dot)
#define N_STATE_VECTOR_DIMENSION 5

// Add longitudinal and yaw acceleration noise
#define N_AUGMENTED_STATE_VECTOR_DIMENSION 7

using namespace KF;
using namespace std;

#define Square(x) ((x)*(x))

SensorFusion::SensorFusion(KFType kftypeLaser, KFType kftypeRadar,
	double alpha, double std_a, double ydd)
	: theState_(N_STATE_VECTOR_DIMENSION)
{
	StateVector *pTheState = &theState_;
	theState_.alpha_ = alpha;
	theState_.std_a_ = std_a;
	theState_.std_yawdd_ = ydd;

	theState_.P_ = alpha * MatrixXd::Identity(theState_.P_.rows(), theState_.P_.cols());
	KalmanFilterFactory KFFactory(pTheState);

	mapSensorsType2KF_.insert(pair<MeasurementPackage::SensorType, UKF *>(MeasurementPackage::LASER, KFFactory.MakeKalmanFilter(MeasurementPackage::LASER, kftypeLaser)));

	mapSensorsType2KF_.insert(pair<MeasurementPackage::SensorType, UKF *>(MeasurementPackage::RADAR, KFFactory.MakeKalmanFilter(MeasurementPackage::RADAR, kftypeRadar)));

	// Initialize P_
	// Get uncertainty from laser of position coordinates
	UKFLaser* laserfs = dynamic_cast<UKFLaser*>(mapSensorsType2KF_[MeasurementPackage::LASER]);
	double stdx = laserfs->GetStdX();
	double stdy = laserfs->GetStdY();

	// Get uncertainty from radar of longitudinal velocity
	UKFRadar* radarfs = dynamic_cast<UKFRadar*>(mapSensorsType2KF_[MeasurementPackage::RADAR]);
	double stdv = radarfs->GetStd_rd();

	VectorXd pDiag(N_STATE_VECTOR_DIMENSION);
	pDiag << stdx * stdx, stdy*stdy, stdv*stdv, alpha, alpha;
	theState_.P_ = pDiag.asDiagonal();
}

SensorFusion::~SensorFusion()
{

}

void SensorFusion::ProcessMeasurement(const MeasurementPackage &mp)
{
	mapSensorsType2KF_[mp.sensor_type_]->ProcessMeasurement(mp);
}

