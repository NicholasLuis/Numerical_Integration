#include <matplot/matplot.h>
#include <vector>
#include <string>

#pragma once
void plot(const std::vector<std::string> plotLabels, const std::vector<std::string> inputDataLabels,
	const std::vector<double> x, const std::vector<double> y1, const std::vector<double> y2, const std::vector<double> y3);

void plot(const std::vector<std::string> plotLabels, const std::vector<std::string> inputDataLabels,
	const std::vector<double> x, const std::vector<double> y1, const std::vector<double> y2, const std::vector<double> y3, const std::vector<double> y4);
