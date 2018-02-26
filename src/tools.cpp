#include <iostream>
#include "tools.h"
#include <algorithm>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::count_if;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/**
	* Calculate the RMSE here.
	*/
	VectorXd rsme(4);
	rsme.fill(0);

	long n = estimations.size();
	if (n < 3 || n != ground_truth.size())
	{
		return rsme;
	}

	for (int i = 0; i < n; i++)
	{
		VectorXd xest = estimations[i];
		VectorXd xlabel = ground_truth[i];

		VectorXd dif = xest - xlabel;
		dif = dif.array() * dif.array();

		rsme += dif;
	}

	// Get average of squared differences
	rsme /= (double)n;

	rsme = rsme.array().sqrt();
	return rsme;
}

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
