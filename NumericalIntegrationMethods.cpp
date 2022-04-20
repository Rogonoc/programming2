//==============================================================
// Filename : NumericalIntegrationMethods.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for combining and switching numerical 
//				 integration methods - header 
//==============================================================


// Libraries
#include "NumericalIntegrationMethods.h"


// Constructor
NumericalIntegrationMethods::NumericalIntegrationMethods(double timeStep, double integrationType)
	: NumericalIntegrationProperties(timeStep) {
	setIntegrationType(integrationType);
}


// Setters
void NumericalIntegrationMethods::setIntegrationType(bool integrationType) {
	m_integrationType = integrationType;
}


// Calculate
/**
 * Computes the integration of the derivative function using either an Euler-approach or a RK-4 approach, specified by the user
 *
 * @param	function : the derivative function specified as a std::function object --> this function must adhere a [ f(x,u,y,P,n) ] format
 * @param	stateVector : the current state vector of the system
 * @param	controlVector : the current control vector
 * @param	outputVector : the current output vector of the system
 * @param	parametersList : vector containing all necessary parameters to compute the derivative equation
 * @param	dynamicsType : specifies the dynamics type to be used of the system
 * @return	A type (std::vector<double>) which is the integrated result of the derivative function
 */
std::vector<double> NumericalIntegrationMethods::calculateNextState(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)> function, 
																	std::vector<double> stateVector, std::vector<double> controlVector, std::vector<double> outputVector,
																	std::vector<double> parameterList, bool dynamicsType) 
{
	// Initialize variables
	std::vector<double> nextStateVector;

	// Choose integration type based on specified bool value
	if (getIntegrationType() == false) { // Euler-case
		nextStateVector = EulerNumericalIntegration::calculateStep(function, stateVector, controlVector, outputVector, parameterList, dynamicsType);
	}
	else if (getIntegrationType() == true) { // Runge Kutta 4-case
		nextStateVector = RungeKuttaFourNumericalIntegration::calculateStep(function, stateVector, controlVector, outputVector, parameterList, dynamicsType);
	}

	// Return next state vector
	return nextStateVector;
}