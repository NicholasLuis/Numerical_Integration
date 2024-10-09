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
	std::vector<double> x;
	std::vector<double> Velo;
	double phi;
	double theta;
	double psi;
	std::array<std::array<double, 3>, 3> DCM;

	// Iteration variables
	double dt;
	const double t0;
	const double tmax;
	std::vector<double> output;

	// Runge-Kutta Data
	std::array<std::array<double, 9>, 2> stateArray;
	std::array<std::array<double, 3>, 3> Cdot1, Cdot2, Cdot3, Cdot4;
	std::vector<double> xdot1, xdot2, xdot3, xdot4;
	const int maxIterations = (int)((tmax - t0) / dt) + 2;
	std::vector<std::vector<double>> x_history; // this matrix saves the state vector data at every time step; will be used to plot later
	std::vector<std::vector<double>> xdot_history; // this matrix saves the state d/dt of vector data at every time step; will be used to plot later
	std::array<std::array<double, 3>, 3>  totalCdot;

public:
	// Functions
	void change_dt(const double new_dt);
	std::vector<double> ffunc(double t);
	std::array<std::array<double, 9>, 2> derivative(const std::vector<double> x, const std::array<std::array<double, 3>, 3> DCM, const std::vector<double> velo, const double t);
	RK4(const double input_dt, const double input_t0, const double input_tmax, std::vector<double> input_x0, std::vector<double> input_Velo); // Constructor
	void run();
	std::vector<std::vector<double>> getXhistData(); // Gets the matrix that contains all the state data at every iteration
	std::vector<std::vector<double>> getXdotHistData(); 
	std::vector<double> getTimeVec(); // Returns a vector of all the time values during iteration
};

#endif