#include <vector>
#include <array>
#include <iostream>

#ifndef RK4_H
#define RK4_H

class RK4
{
	/*
		RK4 Object:
			Inputs:
				Initial State of the system
				Size of time steps
				Max time
			Outputs: 
				Matrix, where each row is the state of the system at each time step
	*/
private:
	const double PI = 3.141592653;

	// States of the system
	std::vector<double> x0;
	std::vector<double> Velo;

	// Iteration variables
	const double dt;
	const double t0;
	const double tmax;

	// Functions
	std::vector<double> ffunc(double t);
	std::array<std::array<double, 9>, 2> derivative(const std::vector<double> x, const std::array<std::array<double, 3>, 3> DCM, const std::vector<double> velo, const double t);
public:
	RK4(const double input_dt, const double input_t0, const double input_tmax, std::vector<double> input_x0, std::vector<double> input_Velo); // Constructor
	void run();
};

#endif