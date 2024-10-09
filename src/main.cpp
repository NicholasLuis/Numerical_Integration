#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <numbers>
#include <array>
#include "RK4.hpp"
#include "MatrixMath.hpp"




/*
	To do list:
	- finish coding numberical integration (DONE)
	- break code into different files for readability (1/2)
		- Create a class that handles matrix math (DONE)
		- Create an RK4 class/function
	- plot stuff
		- Use 
*/

const double PI = 3.141592653;

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

		std::cout << "x pos  = " << x_history[cntr + 1][6] << std::endl;
		std::cout << "y pos  = " << x_history[cntr + 1][7] << std::endl;
		std::cout << "z pos  = " << x_history[cntr + 1][8] << std::endl;


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