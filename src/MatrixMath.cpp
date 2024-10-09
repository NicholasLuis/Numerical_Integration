#include "MatrixMath.hpp"

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