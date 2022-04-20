//==============================================================
// Filename : NumericalIntegrationProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for numerical integration properties - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef NUMERICALINTEGRATIONPROPERTIES_H
#define NUMERICALINTEGRATIONPROPERTIES_H

// Libraries
#include <vector>
#include <functional>

// NumericalIntegrationProperties-class
class NumericalIntegrationProperties {
public:
	// Constructor (default)
	NumericalIntegrationProperties() = default;

	// Constructor (with arguments)
	NumericalIntegrationProperties(double timeStep);

	// Destructor (virtual)
	virtual ~NumericalIntegrationProperties() {}


	// Getters (time)
	double getTimeStep() const { return m_timeStep; }


	// Setters (time)
	void setTimeStep(double);

	// Calculate (derivative system function)
	std::vector<double> calculateDerivativeSystemEquation(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)>, std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool);

private:
	// Attributes (time)
	double m_timeStep;
};


// [END]: Prevent multiple inclusions of header
#endif