#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <numbers>
#include <array>

const double PI = 3.141592653;

std::vector<double> matMult(const double arr[3][3], const std::vector<double> pqr_input)
{
	// Hard-coded matrix multiplcation of a 3x3 by a 3x1
	// see https://images.app.goo.gl/sqXoj4H7qKbAi8V48
	std::vector<double> outputVec;

	outputVec[0] = (arr[0][0] * pqr_input[0]) + (arr[0][1] * pqr_input[1]) + (arr[0][2] * pqr_input[2]);
	outputVec[1] = (arr[1][0] * pqr_input[0]) + (arr[1][1] * pqr_input[1]) + (arr[1][2] * pqr_input[2]);
	outputVec[2] = (arr[2][0] * pqr_input[0]) + (arr[2][1] * pqr_input[1]) + (arr[2][2] * pqr_input[2]);

	return outputVec;
}

std::array<std::array<double, 3>, 3> matMult(const double arr1[3][3], const double arr2[3][3])
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


std::vector<double> ffunc(double t)
{
	// This function gets the angular velocity components in rad/s
	double p = (PI / 6.0);
	double q = cos(6.0 * t / PI);
	double r = 3 * sin(30.0 * t / PI);
	return { p, q, r };
}

std::array<std::array<double, 9>, 2> derivative(const std::vector<double> x, const double DCM[3][3], const std::vector<double> velo, double t)
{
	// This function will output the derivative of the state vector as well as the rate of change of the DCM angles
	std::array<std::array<double, 9>, 2> output;

	double phi = x[0];
	double theta = x[1];
	double rho = x[2];
	std::vector<double> pqr = ffunc(t);
	std::vector<double> q_dot;

	// Gimbal Equation
	double eRate[3][3] = {{1,		tan(theta)* sin(phi),	tan(theta)*cos(phi)	 },
						  {0,		cos(phi),				-sin(phi)			 },
						  {0,		sin(phi)/cos(theta),	cos(phi) / cos(theta)} };

	q_dot = matMult(eRate, pqr);

	// Strapdown Equation
	double UpdateMatrix[3][3] = { {0,		-pqr[2],	pqr[1]	},
								  {pqr[2],	0,			-pqr[0]	},
								  {-pqr[1],	pqr[0],		0		} };
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
	double x0[3] = { 0, 0, 0 };
	double Velo[3] = { 60 * 6076 / 3600 , 0 , 0}; // Speed of the aircraft in ft/s
	double t = 0.0;

	// Generating the full state vector
	std::array<double, 9> x;
	for (int i = 0; i < 3; i++) {
		x[i]   = x0[i];	  // Angles in radians
		x[i+3] = Velo[i]; // Velocity components
		x[i+6] = 0;		  // Position components
	}
	double phi =	x[0];
	double theta =	x[1];
	double psi =	x[2];

	double tmax = 20.0;

	// Creating DCM
	double DCM[3][3] = { {1,	0,	 0},
						 {0,	1,	 0},
						 {0,	0,	 1} }; // Initial DCM because nothign is rotated yet

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