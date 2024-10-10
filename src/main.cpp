#include <cmath>
#include <iostream>
#include <vector>
#include <numbers>
#include <array>
#include "RK4.hpp"
#include "MatrixMath.hpp"
#include "matplotlibcpp.h"


/*
	To do list:

	- plot stuff (need to separate all the data into 1-line vectors)
		- pqr: degrees/sec vs time (output of ffunc)
		- Phi dot, Theta dot, Psi dot: degrees/sec vs time (elements 0->2 of the d/dt state vector, x_dot)
		- Phi, Theta, Psi: degrees vs time (elements 0->2 of the state vector, x)
		- velocity (elements 6->8 of the d/dt state vector, x_dot)
		- position (elements 6->8 of the state vector, x)
	
	- finish coding numberical integration (DONE)
	- break code into different files for readability (DONE)
		- Create a class that handles matrix math (DONE)
		- Create an RK4 class/function (DONE)
*/

const double PI = 3.141592653;

int main()
{
	double dt = 0.2; // Iterate with different dt: 0.2, 0.1, 0.025, 0.0125
	std::vector<double> x0 = { 0, 0, 0 }; // Initial angles
	std::vector<double> Velo = { 60 * 6076 / 3600 , 0 , 0 }; // Speed of the aircraft in ft/s
	const double t0 = 0.0;
	const double tmax = 20.0;

	// Creating an RK4 object that does the numerical integration
	RK4 NumericalIterator(dt, t0, tmax, x0, Velo);

	// Running the numerical integrator and saving the data
	NumericalIterator.run();
	std::vector<double> timeVector1;
	std::vector<std::vector<double>> X_totalData = NumericalIterator.getXhistData();
	std::vector<std::vector<double>> Xdot_totalData = NumericalIterator.getXdotHistData();

	// Getting pqr
	std::vector<double> p(timeVector1.size());
	std::vector<double> q(timeVector1.size());
	std::vector<double> r(timeVector1.size());
	std::vector<double> ffuncOutput(3);
	for (int i = 0; i < timeVector1.size(); i++)
	{
		ffuncOutput = NumericalIterator.ffunc(timeVector1[i]);
		p[i] = ffuncOutput[0];
		q[i] = ffuncOutput[1];
		r[i] = ffuncOutput[0];
	}

	// Getting Phi dot, Theta dot, Psi dot
	std::vector<double> phi_dot(timeVector1.size());
	std::vector<double> theta_dot(timeVector1.size());
	std::vector<double> psi_dot(timeVector1.size());
	for (int i = 0; i < timeVector1.size(); i++)
	{
		phi_dot[i] = Xdot_totalData[i][0];
		theta_dot[i] = Xdot_totalData[i][1];
		psi_dot[i] = Xdot_totalData[i][2];
	}

	// Getting Phi, Theta, Psi
	std::vector<double> phi(timeVector1.size());
	std::vector<double> theta(timeVector1.size());
	std::vector<double> psi(timeVector1.size());
	for (int i = 0; i < timeVector1.size(); i++)
	{
		phi[i] = X_totalData[i][0];
		theta[i] = X_totalData[i][1];
		psi[i] = X_totalData[i][2];
	}

	// Getting Velocity (individual components and also the magnitude)
	std::vector<double> V1(timeVector1.size());
	std::vector<double> V2(timeVector1.size());
	std::vector<double> V3(timeVector1.size());
	std::vector<double> Vmag(timeVector1.size());
	for (int i = 0; i < timeVector1.size(); i++)
	{
		V1[i] = Xdot_totalData[i][6];
		V2[i] = Xdot_totalData[i][7];
		V3[i] = Xdot_totalData[i][8];
		Vmag[i] = sqrt((V1[i] * V1[i]) + (V2[i] * V2[i]) + (V3[i] * V3[i]));
	}

	// Getting Position (individual components and also the magnitude)
	std::vector<double> X1(timeVector1.size());
	std::vector<double> X2(timeVector1.size());
	std::vector<double> X3(timeVector1.size());
	std::vector<double> Xmag(timeVector1.size());
	for (int i = 0; i < timeVector1.size(); i++)
	{
		V1[i] = X_totalData[i][6];
		V2[i] = X_totalData[i][7];
		V3[i] = X_totalData[i][8];
		Vmag[i] = sqrt((V1[i] * V1[i]) + (V2[i] * V2[i]) + (V3[i] * V3[i]));
	}

	// Plotting Stuff


	return 0;
}