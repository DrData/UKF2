#pragma once
#include "measurement_package.h"
#include "ukf.h"
#include "KFEnums.h"
#include <map>
namespace KF
{
	class SensorFusion
	{
	public:
		SensorFusion(KFType kftypeLaser, KFType kftypeRadar, double alpha, double std_a, double std_ydd);
		virtual ~SensorFusion();
		void ProcessMeasurement(const MeasurementPackage& meas_package);

		std::map<MeasurementPackage::SensorType, UKF *> mapSensorsType2KF_;
		StateVector theState_;
	
	};

}