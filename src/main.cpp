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
	// Hard-coded matrix multiplcation of a 3x3 by a 3x3 
}

std::vector<double> ffunc(double t)
{
	// This function gets the angular velocity components in rad/s
	double p = (PI / 6.0);
	double q = cos(6.0 * t / PI);
	double r = 3 * sin(30.0 * t / PI);
	return { p, q, r };
}

std::vector<std::vector<double>> dfunc(const std::vector<double> x, const double DCM[3][3], const std::vector<double> velo, double t)
{
	// This function will return xdot and cdot

	double phi = x[0];
	double theta = x[1];
	double rho = x[2];
	std::vector<double> pqr = ffunc(t);
	std::vector<double> xdot;

	// Gimbal Equation
	double d_DCM[3][3] = {{1,		tan(theta)* sin(phi),	tan(theta)*cos(phi)	 },
						  {0,		cos(phi),				-sin(phi)			 },
						  {0,		sin(phi)/cos(theta),	cos(phi) / cos(theta)} };

	xdot = matMult(d_DCM, pqr);

	// Strapdown Equation
	double UpdateMatrix[3][3] = { {0,		-pqr[2],	pqr[1]	},
								  {pqr[2],	0,			-pqr[0]	},
								  {-pqr[1],	pqr[0],		0		} };
	Cdot = matMult(d_DCM, UpdateMatrix)
}

int main()
{
	double dt = 0.01;
	double x0[3] = { 0, 0, 0 };
	double Velo[3] = { 60 * 6076 / 3600 , 0 , 0}; // Speed of the aircraft in ft/s

	// Plotting Stuff
	{
		if (!glfwInit()) {
			std::cerr << "Failed to initialize GLFW" << std::endl;
			return -1;
		}
		GLFWwindow* window = glfwCreateWindow(500, 500, "Plotting stuff", NULL, NULL);
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