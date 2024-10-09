#include "RK4.hpp"
#include "MatrixMath.hpp"

// This file is the implementation of RK4.hpp. It does all the necessary calculations for Runge Kutta

// Constructor
RK4::RK4(const double input_dt, const double input_t0, const double input_tmax, std::vector<double> input_x0, std::vector<double> input_Velo)
	: dt(input_dt), t0(input_t0), tmax(input_tmax), x(9), x0(input_x0), Velo(input_Velo),
	xdot1(9), xdot2(9), xdot3(9), xdot4(9),
	x_history(maxIterations, std::vector<double>(9)), xdot_history(maxIterations, std::vector<double>(9)),
	phi(x[0]), theta(x[1]), psi(x[2]), DCM({ { {1,	0,	 0}, {0,	1,	 0}, {0,	0,	 1} } })
{
	// std::cout << "RK4 object is created" << std::endl;

	// Generating the full state vector
	for (int i = 0; i < 3; i++) {
		x[i] = x0[i];	  // Angles in radians
		x[i + 3] = Velo[i]; // Velocity components
		x[i + 6] = 0;		  // Position components
	}
}

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

void RK4::change_dt(const double new_dt)
{
	dt = new_dt;
}

std::vector<std::vector<double>> RK4::getXhistData()
{
	return x_history;
}

std::vector<std::vector<double>> RK4::getXdotHistData()
{
	return xdot_history;
}

std::vector<double> RK4::getTimeVec()
{
	std::vector<double> output;
	for (int i = 0; i < tmax; i += dt)
	{
		output[i] = dt;
	}
	return output;
}

void RK4::run()
{
	int cntr = 0;
	for (double t = t0; t < tmax; t += dt) {

		// Calculates the 4 estimates of derivative over the time interval
		stateArray = derivative(x, DCM, Velo, t);
		xdot1 = getXdot(stateArray);
		Cdot1 = getCdot(stateArray);

		stateArray = derivative(matAdd(x, matMult(xdot1, dt / 2)), matAdd(DCM, matMult(Cdot1, dt / 2)), Velo, t + dt);
		xdot2 = getXdot(stateArray);
		Cdot2 = getCdot(stateArray);

		stateArray = derivative(matAdd(x, matMult(xdot2, dt / 2)), matAdd(DCM, matMult(Cdot2, dt / 2)), Velo, t + dt);
		xdot3 = getXdot(stateArray);
		Cdot3 = getCdot(stateArray);

		stateArray = derivative(matAdd(x, matMult(xdot2, dt)), matAdd(DCM, matMult(Cdot2, dt)), Velo, t + dt);
		xdot4 = getXdot(stateArray);
		Cdot4 = getCdot(stateArray);

		// Calculate our 4th order Runge Kutta estimate of derivative
		xdot_history[cntr] = matMult(matAdd(matAdd(xdot1, matMult(xdot2, 2)), matAdd(xdot4, matMult(xdot3, 2))), (1.0 / 6.0));
		totalCdot = matMult(matAdd(matAdd(Cdot1, matMult(Cdot2, 2)), matAdd(Cdot4, matMult(Cdot3, 2))), (1.0 / 6.0));

		// Update our state vector
		x_history[cntr + 1] = matAdd(x_history[cntr], matMult(xdot_history[cntr], dt));
		x = x_history[cntr + 1];
		DCM = matAdd(DCM, matMult(totalCdot, dt));

		cntr++;
	}
	// This function will iterate from t0 -> tmax
}