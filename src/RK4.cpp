#include "RK4.hpp"

// This file is the implementation of RK4.hpp, which does all the necessary calculations

std::vector<double> RK4::ffunc(double t)
{
	// This function gets the angular velocity components in rad/s
	double p = (PI / 6.0);
	double q = cos(6.0 * t / PI);
	double r = 3 * sin(30.0 * t / PI);
	return { p, q, r };
}


std::array<std::array<double, 9>, 2> RK4::derivative(const std::vector<double> x, const std::array<std::array<double, 3>, 3> DCM, const std::vector<double> velo, const double t)
{
	// This function will output the derivative of the state vector as well as the rate of change of the DCM angles
	std::array<std::array<double, 9>, 2> output;

	double phi = x[0];
	double theta = x[1];
	double rho = x[2];
	std::vector<double> pqr = ffunc(t);
	std::vector<double> q_dot;

	// Gimbal Equation
	std::array<std::array<double, 3>, 3> eRate =
	{ { {1,		tan(theta) * sin(phi),	tan(theta) * cos(phi)	},
		{0,		cos(phi),				-sin(phi)				},
		{0,		sin(phi) / cos(theta),	cos(phi) / cos(theta)	} } };

	q_dot = matMult(eRate, pqr);

	// Strapdown Equation
	std::array<std::array<double, 3>, 3> UpdateMatrix =
	{ { {0,		-pqr[2],	pqr[1]	},
		{pqr[2],	0,			-pqr[0]	},
		{-pqr[1],	pqr[0],		0		} } };
	std::array<std::array<double, 3>, 3> DCM_dot = matMult(DCM, UpdateMatrix);

	// Saving the d/dt of DCM as a single 9x1 vector to output variable
	int iter = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			output[1][iter] = DCM_dot[i][j];
			iter++;
		}
	}

	// Expressing xdot (in NED frame) and d_DCM (rate of change of DCM matrix)
	std::vector<double> V_NED = matMult(DCM, velo);

	// Create complete state vector: Euler angle rates, velocity derivatives, and velocity in NED frame
	for (int i = 0; i < 3; i++) {
		output[0][i] = q_dot[i];
		output[0][i + 3] = 0;
		output[0][i + 6] = V_NED[i];
	}

	return output; // 1st line of the output is the state vector. 2nd line is the CDot (d/dt of the DCM)
}
