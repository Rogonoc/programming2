//==============================================================
// Filename : NumericalIntegrationProperties.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for numerical integration properties - source
//==============================================================

// Libraries
#include "NumericalIntegrationProperties.h"
#include <functional>

// Constructor
NumericalIntegrationProperties::NumericalIntegrationProperties(double timeStep) {
	setTimeStep(timeStep);
}


// Setters (time)
void NumericalIntegrationProperties::setTimeStep(double timeStep) {
	m_timeStep = timeStep;
}


// Calculate (derivative system function)
std::vector<double> NumericalIntegrationProperties::calculateDerivativeSystemEquation(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)> f, 
																					  std::vector<double> stateVector, std::vector<double> controlVector, std::vector<double> outputVector,
																					  std::vector<double> parameterList, bool dynamicsType) 
{
	// Return derivative of state vector
	return f(stateVector, controlVector, outputVector, parameterList, dynamicsType);
}