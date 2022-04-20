//==============================================================
// Filename : RungeKuttaFourNumericalIntegration.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for RungeKutta4 numerical integration - source
//==============================================================

// Libraries
#include "RungeKuttaFourNumericalIntegration.h"
#include <algorithm>

// Constructor
RungeKuttaFourNumericalIntegration::RungeKuttaFourNumericalIntegration(double timeStep) : NumericalIntegrationProperties(timeStep) {}


// Calculate (step)
/**
 * Computes the integration of the derivative function using the RK4-approach with some timestep
 *
 * @param	function : the derivative function specified as a std::function object --> this function must adhere a [ f(x,u,y,P,n) ] format
 * @param	stateVector : the current state vector of the system
 * @param	controlVector : the current control vector
 * @param	outputVector : the current output vector of the system
 * @param	parametersList : vector containing all necessary parameters to compute the derivative equation
 * @param	dynamicsType : specifies the dynamics type to be used of the system
 * @return	A type (std::vector<double>) which is the integrated result of the derivative function
 */
std::vector<double> RungeKuttaFourNumericalIntegration::calculateStep(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)> function,
																	  std::vector<double> stateVector, std::vector<double> controlVector, std::vector<double> outputVector,
																	  std::vector<double> parameterList, bool dynamicsType) 
{
	// Initialize variables
	std::vector<double> nextStateVector = stateVector;						// State vector x
	std::vector<double> K1{}, K2{}, K3{}, K4{};								// Original outputs of K's of RK4-method
	std::vector<double> _K1{}, _K2{}, _K3{};								// Copy of K's to perform computations on
	std::vector<double> stateVectorK2{}, stateVectorK3{}, stateVectorK4{};	// Used for f(x + K)
	double timeStep = getTimeStep();										// Time step for integration


	/* --------------------------------- COMPUTE THE K'S FOR THE RUNGE-KUTTA-4 METHOD --------------------------------- */


	// Compute K1
	K1 = calculateDerivativeSystemEquation(function, stateVector, controlVector, outputVector, parameterList, dynamicsType);
	// Save a copy of K1
	_K1 = K1; 
	// Multiply above copy of K1 with time step
	for (auto &element : _K1) { element *= (timeStep / 2); } // [K1 * (h/2)] component
	// Save copy of state vector for processing
	stateVectorK2 = stateVector;
	// Add [K1 * (h/2)] to above copy of state vector; user for computation of K2
	std::transform(stateVectorK2.begin(), stateVectorK2.end(), _K1.begin(), stateVectorK2.begin(), std::plus<double>()); // x + [K1 * (h/2)]


	// Compute K2
	K2 = calculateDerivativeSystemEquation(function, stateVectorK2, controlVector, outputVector, parameterList, dynamicsType);
	// Save a copy of K2
	_K2 = K2;
	// Multiply above copy of K2 with time step
	for (auto &element : _K2) { element *= (timeStep / 2); } // [K2 * (h/2)] component
	// Save copy of state vector for processing
	stateVectorK3 = stateVector;
	// Add [K1 * (h/2)] to above copy of state vector; used for computation of K3
	std::transform(stateVectorK3.begin(), stateVectorK3.end(), _K2.begin(), stateVectorK3.begin(), std::plus<double>()); // x + [K2 * (h/2)]


	// Compute K3
	K3 = calculateDerivativeSystemEquation(function, stateVectorK3, controlVector, outputVector, parameterList, dynamicsType);
	// Save a copy of K3
	_K3 = K3;
	// Multiply above copy of K3 with time step
	for (auto &element : _K3) { element *= (timeStep); } // [K3 * h] component
	// Save copy of state vector for processing
	stateVectorK4 = stateVector;
	// Add [K3 * h] to above copy of state vector; used for computation of K4
	std::transform(stateVectorK4.begin(), stateVectorK4.end(), _K3.begin(), stateVectorK4.begin(), std::plus<double>()); // x + [K3 * h]


	// Compute K4
	K4 = calculateDerivativeSystemEquation(function, stateVectorK4, controlVector, outputVector, parameterList, dynamicsType);


	/* ---------------------------------------------------------------------------------------------------------------- */


	// Multiply with the time step for weighed average of derivatives
	for (auto &element : K1) { element *= (timeStep / 6); } // K1-component [1/6]
	for (auto &element : K2) { element *= (timeStep / 3); } // K2-component [2/6] = [1/3]
	for (auto &element : K3) { element *= (timeStep / 3); } // K3-component [2/6] = [1/3]
	for (auto &element : K4) { element *= (timeStep / 6); } // K4-component [1/6]


	// Sum vectors to construct final state vector
	//		EQUATION: x_next = x_current + (1/6) * h * (K1 + 2*K2 + 2*K3 + 1*K4)
	std::transform(nextStateVector.begin(), nextStateVector.end(), K1.begin(), nextStateVector.begin(), std::plus<double>()); // K1 component
	std::transform(nextStateVector.begin(), nextStateVector.end(), K2.begin(), nextStateVector.begin(), std::plus<double>()); // K2 component
	std::transform(nextStateVector.begin(), nextStateVector.end(), K3.begin(), nextStateVector.begin(), std::plus<double>()); // K3 component
	std::transform(nextStateVector.begin(), nextStateVector.end(), K4.begin(), nextStateVector.begin(), std::plus<double>()); // K4 component

	// Return next state vector
	return nextStateVector;
}