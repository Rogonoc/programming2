//==============================================================
// Filename : EulerNumericalIntegration.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for Euler Numerical Integration - source
//==============================================================

// Libraries
#include "EulerNumericalIntegration.h"
#include <algorithm>			// For std::transform
#include <functional>			// For std::plus
#include <vector>

// Constructor
EulerNumericalIntegration::EulerNumericalIntegration(double timeStep) : NumericalIntegrationProperties(timeStep) {}

// Calculate (step)
/**
 * Computes the integration of the derivative function using the Euler-approach with some timestep
 *
 * @param	function : the derivative function specified as a std::function object --> this function must adhere a [ f(x,u,y,P,n) ] format
 * @param	stateVector : the current state vector of the system
 * @param	controlVector : the current control vector
 * @param	outputVector : the current output vector of the system
 * @param	parametersList : vector containing all necessary parameters to compute the derivative equation
 * @param	dynamicsType : specifies the dynamics type to be used of the system
 * @return	A type (std::vector<double>) which is the integrated result of the derivative function
 */
std::vector<double> EulerNumericalIntegration::calculateStep(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)> function, 
															 std::vector<double> stateVector, std::vector<double> controlVector, std::vector<double> outputVector,
															 std::vector<double> parameterList, bool dynamicsType) 
{
	// Initialize variables 
	std::vector<double> nextStateVector = stateVector;
	std::vector<double> currentDynamicsVector{};

	// Computte dynamics
	currentDynamicsVector = calculateDerivativeSystemEquation(function, stateVector, controlVector, outputVector, parameterList, dynamicsType);

	// Get time step
	double timeStep = getTimeStep();

	// Multiply dynamics vector with time step [h * f()]
	for (auto &element : currentDynamicsVector) {
		element *= timeStep;
	}

	// Add step to current state vector
	std::transform(nextStateVector.begin(), nextStateVector.end(), currentDynamicsVector.begin(), nextStateVector.begin(), std::plus<double>());

	// Return next state vector
	return nextStateVector;
}