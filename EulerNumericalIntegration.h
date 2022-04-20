//==============================================================
// Filename : EulerNumericalIntegration.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for Euler Numerical Integration - source
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef EULERNUMERICALINTEGRATION_H
#define EULERNUMERICALINTEGRATION_H


// Libraries
#include "NumericalIntegrationProperties.h"
#include <functional>
#include <vector>

// EulerIntegration-class
class EulerNumericalIntegration : public virtual NumericalIntegrationProperties {
public:
	// Constructor (default)
	EulerNumericalIntegration() = default;

	// Constructor (with arguments)
	EulerNumericalIntegration(double timeStep);

	// Destructor (virtual)
	virtual ~EulerNumericalIntegration() {}


	// Calculate (step)
	std::vector<double> calculateStep(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)>, 
									  std::vector<double>, std::vector<double>, std::vector<double>, 
									  std::vector<double>, bool);
};


// [END]: Prevent multiple inclusions of header
#endif