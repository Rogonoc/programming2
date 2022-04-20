//==============================================================
// Filename : DroneRopeCargoSimulator.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class containing object to simulate the behavior 
//				 a drone with or without cargo, using different
//				 integration methods - source
//==============================================================

// Libraries
#include "DroneRopeCargoSimulator.h"
#include <algorithm>


// Setters (implementation)
/**
 * Sets the implementation of the simulator, being able to choose from 
 * (i) Cargo;					(1) Euler-integration
 * (ii) Drone-rope-cargo		(2) RK4-integration
 *
 */
void DroneRopeCargoSimulator::setImplementation(bool dynamicsType, bool integrationType) {
	// Set dynamics type
	setDynamicsType(dynamicsType);

	// Set integration type
	setIntegrationType(integrationType);

	// Choose suitable time step based on chosen combination
	double h{}; // Time-step

	if ((dynamicsType == false)) {									 // Drone - Euler / RK4
		h = 0.01;	 // h = 0.01 s
	}
	else if ((dynamicsType == true) && (integrationType == false)) { // Drone with cargo - Euler
		h = 0.0005;  // h = 0.0005 s
	}
	else if ((dynamicsType == true) && (integrationType == true)) {  // Drone with cargo - RK4
		h = 0.01;	 // h = 0.01 s
	}

	// Set time step
	setTimeStep(h);
}

// Other
/**
 * After having specified a control vector for the drone, it computes the resulting dynamics and thus the next state,
 * saves this result to the object, and returns this "next" state vector.
 *
 * @return : A (std::vector<double>) representing the next state vector of drone (+ cargo)
 */
std::vector<double> DroneRopeCargoSimulator::simulationStep(std::vector<double> droneControlVector) {
	/* CONVENTION OF PARAMETER LIST
		0 : gravitational constant
		1 : mass drone
		2 : drag constant drone
		3 : rope initial length
		4 : rope damping
		5 : rope stiffness
		6 : mass cargo 
		7 : drag constant cargo
	*/

	// Initialize variables
	std::vector<double> nextStateVector{};
	const std::vector<double> parameterList = {getGravitationalConstant("Earth"), getMassDrone(), getDragConstantCargo(), getRopeLengthInitial(), getRopeDamping(), getRopeStiffness(), getMassCargo(), getDragConstantCargo()};
	
	// Save the to-be-used derivative function as a parameter
	static std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)> derivativeFunction = calculateDerivativeStateVector;

	/* ------------------------------------------------- ALGORITHM ------------------------------------------------- */ 

	// 1. Use user-specified inputs and save this [control vector] to object
	setDroneControlVector(droneControlVector);

	// 2. Compute resulting dynamics (derivative) in [state vector] due to [control vector]; integrate (derivative) and obtain "next"[state vector]
	nextStateVector = calculateNextState(derivativeFunction, getStateVector(), getDroneControlVector(), getOutputVector(), parameterList, getDynamicsType());

	// 3. Save computed "next" [state-vector] back to object
	setStateVector(nextStateVector);

	// 4. Compute resulting [output vector] and save to object
	setOutputVector();

	// REPEAT

	/* ------------------------------------------------------------------------------------------------------------- */

	return nextStateVector;
}