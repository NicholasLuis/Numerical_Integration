#include <cmath>
#include <iostream>
#include <vector>
#include <numbers>
#include <array>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include "RK4.hpp"
#include "MatrixMath.hpp"
#include "PlotData.hpp"

/*
	To do list:

	- fix issue with velocities (probably a DCM problem)
	- plot data using matplotplusplus
*/

const double PI = 3.14159265358979323846;

int main(int argc, char **argv)
{
	// DEFAULT SIMULATION PARAMETER
	double dt = 0.1; // Iterate with different dt: 0.2, 0.1, 0.025, 0.0125
	double tmax = 60.0;

	// INITIAL CONDITIONS
	std::vector<double> x0 = {0, 0, 0};					 // Initial angles
	std::vector<double> Velo = {60 * 6076 / 3600, 0, 0}; // Speed of the aircraft in ft/s
	const double t0 = 0.0;

	// Loop through all user-provided arguments and process the options
	char c;
	while ((c = getopt(argc, argv, "t:d:h")) != -1)
	{
		switch (c)
		{
		case 't':
			tmax = atof(optarg);
			break;
		case 'd':
			dt = atof(optarg);
			break;
		case 'h':
			printf("main [options] \n");
			printf("-h    Prints this help message\n");
			printf("-t    Sets max sim time as a double\n");
			printf("-d    Sets dt in sim as a double\n");
			return -1;
		}
	}
	printf("Max Time has option %f\n", tmax);
	printf("Delta Time has option %f\n", dt);

	// Creating an RK4 object that does the numerical integration
	RK4 NumericalIterator(dt, t0, tmax, x0, Velo);

	// Running the numerical integrator and saving the data
	NumericalIterator.run();
	std::vector<double> timeVector = NumericalIterator.getTimeVec();
	std::vector<std::vector<double>> X_totalData = NumericalIterator.getXhistData();
	std::vector<std::vector<double>> Xdot_totalData = NumericalIterator.getXdotHistData();
	std::vector<std::vector<double>> Cdot_totalData = NumericalIterator.getCdotHistData();
	std::vector<std::vector<double>> DCM_totalData = NumericalIterator.getDCMHistData();

	// Getting pqr
	std::vector<double> p(timeVector.size());
	std::vector<double> q(timeVector.size());
	std::vector<double> r(timeVector.size());
	std::vector<double> ffuncOutput(3);
	for (int i = 0; i < timeVector.size(); i++)
	{
		ffuncOutput = NumericalIterator.ffunc(timeVector[i]);
		p[i] = ffuncOutput[0];
		q[i] = ffuncOutput[1];
		r[i] = ffuncOutput[2];
	}

	// Getting Phi dot, Theta dot, Psi dot
	std::vector<double> phi_dot(timeVector.size());
	std::vector<double> theta_dot(timeVector.size());
	std::vector<double> psi_dot(timeVector.size());
	for (int i = 0; i < timeVector.size(); i++)
	{
		phi_dot[i] = Xdot_totalData[i][0];
		theta_dot[i] = Xdot_totalData[i][1];
		psi_dot[i] = Xdot_totalData[i][2];
	}

	// Getting Phi, Theta, Psi
	std::vector<double> phi(timeVector.size());
	std::vector<double> theta(timeVector.size());
	std::vector<double> psi(timeVector.size());
	double div; // Need a dividend, divisor (PI), and remainder to revert the angles back to 0 when it gets over PI (only Phi has this issue)
	int n;
	for (int i = 0; i < timeVector.size(); i++)
	{
		phi[i] = X_totalData[i][0];
		while (phi[i] > PI)
		{
			phi[i] -= 2.0 * PI;
		}

		theta[i] = X_totalData[i][1];
		psi[i] = X_totalData[i][2];
	}

	// Getting Velocity (individual components and also the magnitude)
	std::vector<double> V1(timeVector.size());
	std::vector<double> V2(timeVector.size());
	std::vector<double> V3(timeVector.size());
	std::vector<double> Vmag(timeVector.size());
	for (int i = 0; i < timeVector.size(); i++)
	{
		V1[i] = Xdot_totalData[i][6]; // - (1.362 * timeVector[i]);
		V2[i] = Xdot_totalData[i][7];
		V3[i] = Xdot_totalData[i][8];
		Vmag[i] = sqrt((V1[i] * V1[i]) + (V2[i] * V2[i]) + (V3[i] * V3[i]));
	}

	// Getting Position (individual components and also the magnitude)
	std::vector<double> X1(timeVector.size());
	std::vector<double> X2(timeVector.size());
	std::vector<double> X3(timeVector.size());
	std::vector<double> Xmag(timeVector.size());
	for (int i = 0; i < timeVector.size(); i++)
	{
		X1[i] = X_totalData[i][6];
		X2[i] = X_totalData[i][7];
		X3[i] = X_totalData[i][8];
		Xmag[i] = sqrt((X1[i] * X1[i]) + (X2[i] * X2[i]) + (X3[i] * X3[i]));
	}

	// Getting CDot History (individual components)
	std::vector<double> Cdot11(timeVector.size());
	std::vector<double> Cdot12(timeVector.size());
	std::vector<double> Cdot13(timeVector.size());
	std::vector<double> Cdot21(timeVector.size());
	std::vector<double> Cdot22(timeVector.size());
	std::vector<double> Cdot23(timeVector.size());
	std::vector<double> Cdot31(timeVector.size());
	std::vector<double> Cdot32(timeVector.size());
	std::vector<double> Cdot33(timeVector.size());
	for (int i = 0; i < timeVector.size(); i++)
	{
		Cdot11[i] = Cdot_totalData[i][0];
		Cdot12[i] = Cdot_totalData[i][1];
		Cdot13[i] = Cdot_totalData[i][2];
		Cdot21[i] = Cdot_totalData[i][3];
		Cdot22[i] = Cdot_totalData[i][4];
		Cdot23[i] = Cdot_totalData[i][5];
		Cdot31[i] = Cdot_totalData[i][6];
		Cdot32[i] = Cdot_totalData[i][7];
		Cdot33[i] = Cdot_totalData[i][8];
	}

	// Getting CDot History (individual components)
	std::vector<double> DCM11(timeVector.size());
	std::vector<double> DCM12(timeVector.size());
	std::vector<double> DCM13(timeVector.size());
	std::vector<double> DCM21(timeVector.size());
	std::vector<double> DCM22(timeVector.size());
	std::vector<double> DCM23(timeVector.size());
	std::vector<double> DCM31(timeVector.size());
	std::vector<double> DCM32(timeVector.size());
	std::vector<double> DCM33(timeVector.size());
	for (int i = 0; i < timeVector.size(); i++)
	{
		DCM11[i] = DCM_totalData[i][0];
		DCM12[i] = DCM_totalData[i][1];
		DCM13[i] = DCM_totalData[i][2];
		DCM21[i] = DCM_totalData[i][3];
		DCM22[i] = DCM_totalData[i][4];
		DCM23[i] = DCM_totalData[i][5];
		DCM31[i] = DCM_totalData[i][6];
		DCM32[i] = DCM_totalData[i][7];
		DCM33[i] = DCM_totalData[i][8];
	}

	// Plotting Stuff in Excel

	// Full data does not output correctly. So, have to print only a few columns at a time
	for (int i = 0; i < timeVector.size(); i++)
	{
		// Un-Commment each line to get its data
		// std::cout << timeVector[i] << "\t" << p[i] << "\t" << q[i] << "\t" << r[i] << std::endl;					   // Angular velocity
		// std::cout << timeVector[i] << "\t" << phi_dot[i] << "\t" << theta_dot[i] << "\t" << psi_dot[i] << std::endl;   // Euler angle rates
		std::cout << timeVector[i] << "\t" << phi[i] << "\t" << theta[i] << "\t" << psi[i] << std::endl; // Euler angles
																										 // std::cout << timeVector[i] << "\t" << V1[i] << "\t" << V2[i] << "\t" << V3[i] << "\t" << Vmag[i] << std::endl; // Velocities
																										 // std::cout << timeVector[i] << "\t" << X1[i] << "\t" << X2[i] << "\t" << X3[i] << "\t" << Xmag[i] << std::endl; // Position
																										 // std::cout << timeVector[i] << "\t" << DCM11[i] << "\t" << DCM12[i] << "\t" << DCM13[i] << "\t" << DCM21[i] << "\t" << DCM22[i] << "\t" << DCM23[i] << "\t" << DCM31[i] << "\t" << DCM32[i] << "\t" << DCM33[i] << std::endl;
	}
	std::cout << "COPY AND PASTE THE ABOVE DATA INTO EXCEL (TAB DELIMITER)" << std::endl;


	// Plotting stuff in MatplotPlusPlus
	std::vector<std::string> plotLabels = {"Angular Velocities as a Function of Time", "Time (s)", "Angular Velocity (rad / s)"};
	std::vector<std::string> inputLabels = {"p", "q", "r"};
	plot(plotLabels, inputLabels, timeVector, p, q, r);

	plotLabels = {"Euler Angle Rates", "Time (s)", "Angle rate of change (rad / s)"};
	inputLabels = {"Phi dot", "Theta Dot", "Psi Dot"};
	plot(plotLabels, inputLabels, timeVector, phi_dot, theta_dot, psi_dot);

	plotLabels = {"Euler Angle", "Time (s)", "Angle (rad)"};
	inputLabels = {"Phi", "Theta", "Psi"};
	plot(plotLabels, inputLabels, timeVector, phi, theta, psi);

	plotLabels = {"Velocity Components", "Time (s)", "Speed (ft/s)"};
	inputLabels = {"V_N", "V_E", "V_D", "V"};
	plot(plotLabels, inputLabels, timeVector, V1, V2, V3, Vmag);

	plotLabels = {"Position Components", "Time (s)", "Position (ft)"};
	inputLabels = {"P_N", "P_E", "P_D", "P"};
	plot(plotLabels, inputLabels, timeVector, X1, X2, X3, Xmag);

	return 0;
}
