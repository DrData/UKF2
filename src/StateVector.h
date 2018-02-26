#pragma once

#include "Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace KF
{

	struct StateVector
	{
		StateVector(int ndim) : is_initialized_(false), previous_timestamp_(0)
		{
			x_ = VectorXd(ndim);
			x_.fill(0);

			P_ = alpha_ * MatrixXd::Identity(ndim, ndim);
		}

		VectorXd x_;
		MatrixXd P_;
		double nis_;

		long size() const { return x_.size(); }

		bool is_initialized_;
		long long previous_timestamp_;


		// Model Parameters for run
		double std_a_;
		double std_yawdd_;
		double alpha_;

	};
}