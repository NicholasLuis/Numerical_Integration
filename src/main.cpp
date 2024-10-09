#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <numbers>
#include <array>


/*
	To do list:
	- finish coding numerical stuff
	- break code into different files for readability
		- Create a class that handles matrix multiplication and stuff
		- Create an RK4 class/function
	- plot stuff
*/

const double PI = 3.141592653;

// Vector stuff
std::vector<double> matMult(const std::array<std::array<double, 3>, 3> arr, const std::vector<double> pqr_input)
{
	// Hard-coded matrix multiplcation of a 3x3 by a 3x1
	// see https://images.app.goo.gl/sqXoj4H7qKbAi8V48
	std::vector<double> outputVec(3);
	
	outputVec[0] = (arr[0][0] * pqr_input[0]) + (arr[0][1] * pqr_input[1]) + (arr[0][2] * pqr_input[2]);
	outputVec[1] = (arr[1][0] * pqr_input[0]) + (arr[1][1] * pqr_input[1]) + (arr[1][2] * pqr_input[2]);
	outputVec[2] = (arr[2][0] * pqr_input[0]) + (arr[2][1] * pqr_input[1]) + (arr[2][2] * pqr_input[2]);

	return outputVec;
}

std::vector<double> matMult(const std::vector<double> vec, const double scalar)
{
	// Hard-coded matrix multiplcation of a 1x9 by a scalar (Overload function)
	std::vector<double> output(9);

	for (int i = 0; i < vec.size(); i++)
	{
		output[i] = vec[i] * scalar;
	}

	return output;
}

std::array<std::array<double, 3>, 3> matMult(const std::array<std::array<double, 3>, 3> arr1, const std::array<std::array<double, 3>, 3> arr2)
{
	// Hard-coded matrix multiplcation of a 3x3 by a 3x3 (Overload function)
	std::array<std::array<double, 3>, 3> arrOut;

	for (int i = 0; i < 3; i++) {         // Iterates over rows of arr1
		for (int j = 0; j < 3; j++) {     // Iterates over cols of arr2
			for (int k = 0; k < 3; k++) { // Iterates over cols of arr1 & rows of arr2
				arrOut[i][j] = arr1[i][k] * arr2[k][j];
			}
		}
	}

	return arrOut;
}

std::array<std::array<double, 3>, 3> matMult(const std::array<std::array<double, 3>, 3> arr, const double scalar)
{
	// Hard-coded multiplcation of a 3x3 matrix by a scalar (Overload function)
	std::array<std::array<double, 3>, 3> arrOut;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			arrOut[i][j] = arr[i][j] * scalar;
		}
	}

	return arrOut;
}

std::array<std::array<double, 3>, 3> matAdd(const std::array<std::array<double, 3>, 3> arr1, const std::array<std::array<double, 3>, 3> arr2)
{
	// Overload function that adds two matrices of size 3x3
	std::array<std::array<double, 3>, 3> output;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			output[i][j] = arr1[i][j] + arr2[i][j];
		}
	}

	return output;
}

std::vector<double> matAdd(const std::vector<double> vec1, const std::vector<double> vec2)
{
	// This function does basic matrix addition of two 1xN matrices (vectors)
	std::vector<double> output(vec1.size());

	// Checks if both vectors are the same size
	if (vec1.size() != vec2.size())
	{
		std::cout << "ERROR, both vectors should be the same size to add them together" << std::endl;
	}

	for (int i = 0; i < vec1.size(); i++)
	{
		output[i] = vec1[i] + vec2[i];
	}

	return output;
}

std::array<std::array<double, 3>, 3> getCdot(std::array<std::array<double, 9>, 2> input) 
{
	// This function extracts and re-builds the DCM (3x3) from the state matrix (2x9, where the row 2 is the DCM)

	std::array<std::array<double, 3>, 3> extractedDCM;

	int iter = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			extractedDCM[i][j] = input[1][iter];
			iter++;
		}
	}

	return extractedDCM;
}

std::vector<double> getXdot(std::array<std::array<double, 9>, 2> input)
{
	std::vector<double> extractedStateVector(9);

	int iter = 0;
	for (int i = 0; i < 9; i++) {
		extractedStateVector[i] = input[0][i];
	}

	return extractedStateVector;
}

// Other functions

std::vector<double> ffunc(double t)
{
	// This function gets the angular velocity components in rad/s
	double p = (PI / 6.0);
	double q = cos(6.0 * t / PI);
	double r = 3 * sin(30.0 * t / PI);
	return { p, q, r };
}

std::array<std::array<double, 9>, 2> derivative(const std::vector<double> x, const std::array<std::array<double, 3>, 3> DCM, const std::vector<double> velo, const double t)
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
		output[0][i]   = q_dot[i];
		output[0][i+3] = 0;
		output[0][i+6] = V_NED[i];
	}
	
	return output; // 1st line of the output is the state vector. 2nd line is the CDot (d/dt of the DCM)
}

int main()
{
	double dt = 0.2; // Iterate with different dt: 0.2, 0.1, 0.025, 0.0125
	std::vector<double> x0 = { 0, 0, 0 }; // Initial angles
	std::vector<double> Velo = {60 * 6076 / 3600 , 0 , 0}; // Speed of the aircraft in ft/s
	const double t0 = 0.0;
	const double tmax = 20.0;

	// Generating the full state vector
	std::vector<double> x(9);
	for (int i = 0; i < 3; i++) {
		x[i]   = x0[i];	  // Angles in radians
		x[i+3] = Velo[i]; // Velocity components
		x[i+6] = 0;		  // Position components
	}
	double phi =	x[0];
	double theta =	x[1];
	double psi =	x[2];


	// Creating DCM
	std::array<std::array<double, 3>, 3> DCM = 
		{ { {1,	0,	 0},
			{0,	1,	 0},
			{0,	0,	 1} } }; // Initial DCM is identity because nothign is rotated yet

	// RUNGE-KUTTA
	std::array<std::array<double, 9>, 2> stateArray;
	std::array<std::array<double, 3>, 3> Cdot1, Cdot2, Cdot3, Cdot4;
	std::vector<double> xdot1(9), xdot2(9), xdot3(9), xdot4(9);
	const int maxIterations = (int)((tmax - t0) / dt) + 2;
	std::vector<std::vector<double>> x_history(maxIterations, std::vector<double>(9)); // this matrix saves the state vector data at every time step; will be used to plot later
	std::vector<std::vector<double>> xdot_history(maxIterations, std::vector<double>(9)); // this matrix saves the state d/dt of vector data at every time step; will be used to plot later
	std::array<std::array<double, 3>, 3>  totalCdot;
	std::cout << "iterations = " << maxIterations << std::endl;
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
		xdot_history[cntr] = matMult(matAdd( matAdd(xdot1, matMult(xdot2, 2)),matAdd(xdot4, matMult(xdot3, 2)) ), (1.0/6.0) );
		totalCdot = matMult(matAdd(matAdd(Cdot1, matMult(Cdot2, 2)), matAdd(Cdot4, matMult(Cdot3, 2))), (1.0/6.0));

		// Update our state vector
		x_history[cntr + 1] = matAdd(x_history[cntr], matMult(xdot_history[cntr],dt));
		x = x_history[cntr+1];
		DCM = matAdd(DCM, matMult(totalCdot, dt));

		std::cout << "x_hist  = " << x_history[cntr + 1][1] << std::endl;

		cntr++;
	}


	// Plotting Stuff
	{
		if (!glfwInit()) {
			std::cerr << "Failed to initialize GLFW" << std::endl;
			return -1;
		}
		GLFWwindow* window = glfwCreateWindow(1000, 500, "Plotting stuff", NULL, NULL);
		if (!window) {
			std::cerr << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return -1;
		}
		glfwMakeContextCurrent(window);
		if (glewInit() != GLEW_OK) {
			std::cerr << "Failed to initialize GLEW" << std::endl;
			return -1;
		}
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		while (!glfwWindowShouldClose(window)) {
			glClear(GL_COLOR_BUFFER_BIT);
			glBegin(GL_LINES);
			// Draw X and Y axis
			glColor3f(1.0, 1.0, 1.0);
			glVertex2f(-1.0, 0.0);
			glVertex2f(1.0, 0.0);
			glVertex2f(0.0, 1.0);
			glVertex2f(0.0, -1.0);

			// Plot stuff here

			glEnd();
			glfwSwapBuffers(window);
			glfwPollEvents();
		}
		glfwTerminate();
	}

	return 0;
}