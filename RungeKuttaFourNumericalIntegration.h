//==============================================================
// Filename : RungeKuttaFourNumericalIntegration.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for RungeKutta4 numerical integration - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef RUNGEKUTTAFOURNUMERICALINTEGRATION_H
#define RUNGEKUTTAFOURNUMERICALINTEGRATION_H


// Libraries
#include "NumericalIntegrationProperties.h"
#include <functional>
#include <vector>


// RungeKuttaFourNumericalIntegration-class
class RungeKuttaFourNumericalIntegration : public virtual NumericalIntegrationProperties {
public:
	// Constructor (default)
	RungeKuttaFourNumericalIntegration() = default;

	// Constructor (with parameters)
	RungeKuttaFourNumericalIntegration(double timeStep);


	// Calculate (step)
	std::vector<double> calculateStep(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)>, 
									  std::vector<double>, std::vector<double>, std::vector<double>, 
									  std::vector<double>, bool);
};


// [END]: Prevent multiple inclusions of header
#endif