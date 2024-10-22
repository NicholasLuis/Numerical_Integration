#include "PlotData.hpp"


void plot(const std::vector<std::string> plotLabels, const std::vector<std::string> inputDataLabels,
	const std::vector<double> x, const std::vector<double> y1, const std::vector<double> y2, const std::vector<double> y3)
{
	// This function plots 3 sets of data vs a common x
	
	// Initial Setup
	auto fig = matplot::figure();
	fig->size(1800, 600);

	// Plotting the data
	auto plot1 = matplot::plot(x, y1, "r");
	matplot::hold("on");
	auto plot2 = matplot::plot(x, y2, "g");
	auto plot3 = matplot::plot(x, y3, "b");

	// Formatting
	auto l = ::matplot::legend({ inputDataLabels[0], inputDataLabels[1], inputDataLabels[2] });
	l->location(matplot::legend::general_alignment::topright);
	matplot::title(plotLabels[0]);
	matplot::xlabel(plotLabels[1]);
	matplot::ylabel(plotLabels[2]);
	matplot::hold("off");

	matplot::show();
	matplot::save(plotLabels[0], "jpeg");
}

void plot(const std::vector<std::string> plotLabels, const std::vector<std::string> inputDataLabels,
	const std::vector<double> x, const std::vector<double> y1, const std::vector<double> y2, const std::vector<double> y3, const std::vector<double> y4) 
{	
	// Overload function that plots 4 sets of data vs a common x

	// Initial Setup
	auto fig = matplot::figure();
	fig->size(1800, 600);

	// Plotting the data
	auto plot1 = matplot::plot(x, y1, "r");
	matplot::hold("on");
	auto plot2 = matplot::plot(x, y2, "g");
	auto plot3 = matplot::plot(x, y3, "b");
	auto plot4 = matplot::plot(x, y4, "black");

	// Formatting
	auto l = ::matplot::legend({ inputDataLabels[0], inputDataLabels[1], inputDataLabels[2] , inputDataLabels[3] });
	l->location(matplot::legend::general_alignment::topright);
	matplot::title(plotLabels[0]);
	matplot::xlabel(plotLabels[1]);
	matplot::ylabel(plotLabels[2]);
	matplot::hold("off");

	matplot::show();
	matplot::save(plotLabels[0], "jpeg");

}
