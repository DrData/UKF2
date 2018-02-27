# Unscented Kalman Filter
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

## Code

The code may be found in this repository. The source files that are new or modified are in the src folder and comprise the following:
1. main.cpp
2. ukf.cpp and ukf.h
3. KalmanFilterFactory.cpp and KalmanFilterFactory.h
4. KFEnums.h
5. SensorFusion.cpp, SensorFusion.h
6. StateVector.h
7. tools.cpp, tools.h

The other code source in src are unmodified.

### Rubric: Builds
1. The project code compiles and builds under Windows and Ubuntu OSes.


### Rubric: Accuracy
2. The final RMSE were [0.083, 0.081, 0.39, 0.18], which is below the threshold of [.09, 0.10, 0.40, 0.30] set in the rubric.
The RMSE and the estimated and ground truth is plotted below from the simulator:
![path](./estimate_RMSE.PNG)

### Rubric: Follows the general processing follow

#### Design
##### SensorFusion

The creation and initialization of the processing was encapsulated in the class SensorFusion. It contains the state vector, the KalmanFilterFactory class that is responsible for creating the appropriate Kalman filter based on the sensor and Kalman filter type. Currently, only the UKF types are supported. It can be expanded to accommodate EKF also.

In this project, the laser and rader UKF classes are created and stored in a map. The processing method takes the sensor type and applies the mapped Kalman filter's ProcessMeasurement method.

##### StateVector
The StateVector class carries the state vector and the covariance matrix.
It also contains the time of the last process measurement and a flag indicating if the state vector was initialized.

##### UKF
UKF is the base UKF class that contains most of the process algorithm and is designed to be specialized for the specific sensor type.

UKF
Contains the variables needed for process and declares and allocates some temporary work matrices so they don't need to be repeated.

The base class UKF has one main public method ProcessMeasurement that is responsible for processing the measurement data. Within UKF, there are the private methods Prediction and Update.
1. Prediction = generates the sigma points using augmentation to accomodate noise, projects the with the process model, and computes the mean and the covariance from the projected sigma points.
2. Update processes the measurement and makes a correction to the prediction based on the difference between the prediction and the measurement.

## Rubric: Initialization


### Initialize the state variable, x

There are three virtual methods that can be specialized by derived class for the specific sensor:
1. **InitializeMeasurement** - handles the first measurement, initializing the state vector based on the first measurment and sets up the subsequent process. If the first measurement is a lidar measurement, then we populate the x and y coordinate of the state vector directly from the measurment and remaining components taken as zero. If the first measurement is a radar measurment, we convert the polar coordinate radar measurment into the Cartesian space of the state vector. The x and y coordinate can be obtained from r and phi. The longitudinal velocity is just r_dot. The rest of the state vector are set to zero.

### Initializing P
When available the covariance matrix was initialized using the measurement uncertainy of the corresponding state variable. For the variables, x and y, we use the standard deviation of the lidar uncertainty in measuring the position x and y - 0.15 and 0.15.

For the velocity, the radar measured rho_dot directly and so we used the uncertainty of that degree of freedom, std_rhodot = 0.3.

For yaw and yawdot, I left that as a variable scale parameter named alpha which I varied to (1) meet the RMSE criteria and (2) to make the NIS consistent.
The final P initialization is listed below. 

![pinit](./Pinit.png)

This was done in the the code in the file 'SensorFusion.cpp'
```
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
```

## Rubric: ProcessMeasurement
- Predicts first and then updates.

### UKF
UKF::ProcessMeasurement followed the processing as outlined in the lessons. The code was based on code from the class lessons and were modified to be more efficient.
The class signatures were updated to use reference instead of pointers.

**Prediction(double dt )**
- AugmentSigmaPoints - generates the sigma points
- SigmaPointPrediction - applies the process model to the generated sigma points
- PredictMeanAndCovariance - computes the mean and convariance by using a weighted average of the sigma points and the covariance.

**Update( const MeasurementPackage &meas )**
- Convert predicted sigma points into measurement space
- Computes the innovation and cross correlation matrix. Normalizing the angles after taking the difference between the predicted sigma point and the mean is crucial in stability.
- Computes the NIS.

## Rubric: Appropriate handling for Lidar and Radar
Update is specialized of Lidar and Radar by overriding these virtual methods described in the section below.
- UKFRadar::Convert2MeasurementSpace

### UKF Laser
Member variables for Laser:

Constructor initialized the following:
1. n\_z\_ this is the number of dimensions in the laser measurement space, 2
2. The innovation and cross correlation matrices are allocated
3. Flag to process data or not.

**InitializeMeasurement**
Initialize the state vector with information from the first measurement. The state vector for px and py are assigned the first measurment's x and y. The remaining state vector components are set to zero.

**Convert2MeasurementSpace**
Takes the predicted sigma points in state space into the measurement space. Here the conversion is simple since the laser measures the position only and the state vector contains the position.

NormalizeMeasurementAngles
Since there are no angles, it does nothing.

### UKF Radar
Member variables for Radar:

Constructor initialized the following:
1. n\_z\_ this is the number of dimensions in the radar measurement space, 3.
2. The innovation and cross correlation matrices are allocated
3. Flag to process data or not.

**InitializeMeasurement**
Since the radar measurements are in polar coordinates (rho, theta, rhodot), they need to be convert back into Cartesian coorindates, into (r cos(theta), r sin(theta), abs(rhodot), 0, 0). The remaining state vector coordinates are set to zero.

**Convert2MeasurementSpace**

Takes the predicted sigma points in state space into the measurement space. For the radar measurement this requires transforming the state vector into the polar coordinates of the radar system.

**NormalizeMeasurementAngles**
Since the radar coordinates has an angle, we normalize that angle component.


### Code Efficiency Optimizations
1. Work/storage variables are declared and pre-allocated in the constructor to reduce repeated allocation to the stack.
2. The cross correlation and innovation matrices in the update state are computed in the same loop to reduce repeated calculations.
3. The same code is used for both sensor types.

### Tuning Parameters
The following parameters are used to tune the model, alpha, std_a, and std_yawdd.
Alpha is a factor used to intialize a part of the convariance matrix P where we don't have specfic information.

In addition to searching for the for parameters values to meet the RMSE criteria, the parameters values must be chosen so the Gaussian distribution parameters are consistent. To this end, we use the Normalized Innovation Squared statistic, NIS, which is related to the square of the Mahalanobis distrance from multidimension statistics. The key point here is that NIS follows a chi-squared distribution.

For the final selected parameters, the NIS of the Laser and Radar data are displayed in the graphs below. We can compute the coverage from the NIS by computing the percent number of points that are below the chi-square value for 95% coverage for 2 and 3 degrees of freedom for the laser and radar data. 
```
double Tools::CalculateNISCoverage(const double dof, const vector<double>& vNIS)
{
	double n95;
	if (dof == 3)
		n95 = 7.815;
	else if (dof == 2)
		n95 = 5.991;
	else if (dof == 1)
		n95 = 3.841;
	else
		throw "dof out of range";

	if (vNIS.size() < 1) return 0.0;
	int nBelow = count_if(vNIS.begin(), vNIS.end(), [&n95](double nis)->bool { return nis <= n95; });
	return (double) nBelow / (double) vNIS.size();
}

```
For the final parameters the coverage for laser and radar data were 96% and 94%. Which means the selected parameters are consistent. The plot of the NIS are below. The horizontal line is the 95% confidence interval for a chi-square distribution for the given degrees of freedom.

The final parameters used in the submission are std_a = 1.5, std_yawdd=0.6, alpha=0.1.

![NISLaser](./NISLaser.png)

![NISRadar](./NISRadar.png)
