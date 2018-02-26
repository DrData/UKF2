#include "KalmanFilterFactory.h"
#include "measurement_package.h"
#include "ukf.h"

using namespace KF;

UKF * KF::KalmanFilterFactory::MakeKalmanFilter(MeasurementPackage::SensorType stype, KFType kfType)
{
	UKF * kfs = NULL;
	switch (stype)
	{
		case MeasurementPackage::LASER:
			if (kfType == eUKF_hybrid)
			{
				kfs = new UKFLaser(_pSV);
				dynamic_cast<UKFLaser*>(kfs)->SetStd_Acc(this->_pSV->std_a_);
				dynamic_cast<UKFLaser*>(kfs)->SetStd_Acc(this->_pSV->std_yawdd_);
			}
			else if (kfType == KFType::eUKF_full)
			{
				kfs = new UKFLaser(_pSV);
				dynamic_cast<UKFLaser*>(kfs)->SetStd_Acc(this->_pSV->std_a_);
				dynamic_cast<UKFLaser*>(kfs)->SetStd_Acc(this->_pSV->std_yawdd_);
			}
			else
				kfs = NULL;
		break;
		case MeasurementPackage::RADAR:
			if (kfType == eUKF_hybrid || kfType == KFType::eUKF_full)
			{
				kfs = new UKFRadar(_pSV);
				dynamic_cast<UKFRadar*>(kfs)->SetStd_Acc(this->_pSV->std_a_);
				dynamic_cast<UKFRadar*>(kfs)->SetStd_Acc(this->_pSV->std_yawdd_);
			}
			else
				kfs = NULL;
		break;
	}
	return kfs;
}