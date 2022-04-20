//==============================================================
// Filename : DroneRopeCargoDynamics.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for dynamics of drone with attached  
//				 (slinging) cargo by rope - source
//==============================================================

// Libraries 
#include "DroneRopeCargoDynamics.h"
#include <cmath>

// Constructor
DroneRopeCargoDynamics::DroneRopeCargoDynamics(double dynamicsType, double massDrone, double dragConstantDrone, double ropeLengthInitial, double ropeStiffness, double ropeDamping, double massCargo, double dragConstantCargo)
	: DroneDynamics(massDrone, dragConstantDrone), RopeProperties(ropeLengthInitial, ropeStiffness, ropeDamping), CargoDynamics(massCargo, dragConstantCargo) {
	// Set attributes
	setDynamicsType(dynamicsType);
};


// Getters (state vector)
/**
 * Retrieves the state vector of the drone + cargo system
 *
 * @return	A (std::vector<double>) which represents the output vector
 */
std::vector<double> DroneRopeCargoDynamics::getStateVector() {
	// Initialize total state vector
	std::vector<double> stateVector{};
	
	// Retrieve state vector from drone
	std::vector<double> droneStateVector = getDroneStateVector();

	// Retrieve state vector from cargo
	std::vector<double> cargoStateVector = getCargoStateVector();

	// Concatenate
	stateVector.reserve(stateVector.size() + droneStateVector.size() + cargoStateVector.size()); // Allocate memory for speed
	stateVector.insert(stateVector.end(), droneStateVector.begin(), droneStateVector.end());     // Drone --> total
	stateVector.insert(stateVector.end(), cargoStateVector.begin(), cargoStateVector.end());     // Cargo --> total

	// Return concatenated (total) state vector
	return stateVector;
}

// Getters (output vector)
/**
 * Retrieves the output vector of the drone + cargo system
 * 
 * @return	A (std::vector<double>) which represents the output vector
 */
std::vector<double> DroneRopeCargoDynamics::getOutputVector() {

	// Update output vector
	setOutputVector();

	// Return vector
	return m_outputVector;
};

double DroneRopeCargoDynamics::getRopeLength() {
	// Update output vector
	setOutputVector();

	// Return value
	return m_outputVector[0];

}

double DroneRopeCargoDynamics::getRopeRateOfChange() {
	// Update output vector
	setOutputVector();

	// Return value
	return m_outputVector[1];
}

// Setters (dynamics type)
void DroneRopeCargoDynamics::setDynamicsType(bool dynamicsType) {
	m_dynamicsType = dynamicsType;
}

// Setters (state vector)
/**
 *	Sets state vector to object with an input state vector
 *
 *	@param	StateVector	: std::vector<double>
 */
void DroneRopeCargoDynamics::setStateVector(std::vector<double> stateVector) {
	// Initialize drone state vector
	std::vector<double> droneStateVector{ stateVector[0], stateVector[1], stateVector[2], stateVector[3], stateVector[4] };

	// Set state vector of drone
	setDroneStateVector(droneStateVector);

	/*-------------------------------------*/

	// Initialize cargo state vector
	std::vector<double> cargoStateVector{ stateVector[5], stateVector[6], stateVector[7], stateVector[8] };

	// Set state vector of cargo
	setCargoStateVector(cargoStateVector);
}

// Setters (output vector)
/**
 *	Calculates the output vector based on the current state vector and saves it to the object
 */
void DroneRopeCargoDynamics::setOutputVector() {

	// Compute output vector
	std::vector<double> outputVector = calculateOutputVector();

	// Save new rope length
	setRopeLength(outputVector[0]);

	// Save new rope rate of change
	setRopeRateOfChange(outputVector[1]);
};

void DroneRopeCargoDynamics::setRopeLength(double ropeLength) {
	m_outputVector[0] = ropeLength;
};

void DroneRopeCargoDynamics::setRopeRateOfChange(double ropeRateOfChange) {
	m_outputVector[1] = ropeRateOfChange;
};


// Calculate (output vector)
/**
 * Computes the output vector of the drone + cargo system
 *
 * @return	A (std::vector<double>) which represents the computation of the output vector
 */
std::vector<double> DroneRopeCargoDynamics::calculateOutputVector() {
	/* NOTATIONS:
		x1 : xDrone           y1: rope length
		x2 : yDrone           y2: rope rate of change
		x3 : thetaDrone
		x4 : xDotDrone
		x5 : yDotDrone
		x6 : xCargo
		x7 : yCargo
		x8 : xDotCargo
		x9 : yDotCargo
	*/

	// Initialize vector
	std::vector<double> outputVector{};

	// Calculation of rope length (y1)
	double ropeLength{};

	ropeLength = sqrt(pow(getXDrone() - getXCargo(), 2) + pow(getYDrone() - getYCargo(), 2));

	// Calculation of rope rate of change (y2)
	double ropeRateOfChange{};
	double ropeRateOfChangePart1{}, ropeRateOfChangePart2{};

	ropeRateOfChangePart1 = (getXDrone() - getXCargo()) + (getXDotDrone() - getXDotCargo());	// Part 1
	ropeRateOfChangePart2 = (getYDrone() - getYCargo()) + (getYDotDrone() - getYDotCargo());	// Part 2

	// Prevent division by zero
	if (ropeLength == 0) {
		ropeRateOfChange = 0;
	}
	else {
		ropeRateOfChange = (ropeRateOfChangePart1 + ropeRateOfChangePart2) / ropeLength;
	}


	// Construct vector
	outputVector = { ropeLength, ropeRateOfChange };

	// Return vector
	return outputVector;
}

// Calculate (state derivative)
/**
 * Computes the output vector of the drone + cargo system
 *
 * @param	stateVector : the current state vector of the system
 * @param	controlVector : the current control vector
 * @param	outputVector : the current output vector of the system
 * @param	parametersList : vector containing all necessary parameters to compute the derivative equation
 * @param	dynamics : specifies the dynamics type to be used of the system
 * @return	A (std::vector<double>) which represents the computation of the output vector
 */
std::vector<double> DroneRopeCargoDynamics::calculateDerivativeStateVector(std::vector<double> stateVector, std::vector<double> controlVector, std::vector<double> outputVector, std::vector<double> parametersList, bool dynamicsType) {
	/* NOTATIONS:
		x1* : xDrone		u1 : tauDrone			y1 : rope length
		x2* : yDrone		u2 : omegaDrone			y2 : rope rate of change
		x3* : thetaDrone							y3 : rope angle
		x4* : xDotDrone		  
		x5* : yDotDrone		
		x6* : xCargo		[* : derivative] 
		x7* : yCargo
		x8* : xDotCargo
		x9* : yDotCargo
	*/

	/* PARAMETER LIST CONVENTION
		0 : gravitational constant
		1 : mass drone
		2 : drag constant drone
		3 : rope initial length
		4 : rope damping
		5 : rope stiffness
		6 : mass cargo 
		7 : drag constant cargo
	*/

	// Intialize vector
	std::vector<double> dynamicsStateVector{};

	// Initialize states
	double xDot1{}, xDot2{}, xDot3{}, xDot4{}, xDot5{}, xDot6{}, xDot7{}, xDot8{}, xDot9{};

	/* ---------------------------------------------------- DRONE ---------------------------------------------------- */

	// Calculate change in x-position drone - VELOCITY (x1*)
	xDot1 = stateVector[3];

	// Calculate change in y-position drone - VELOCITY (x2*)
	xDot2 = stateVector[4];

	// Calculate change in theta-position drone - ANGULAR VELOCITY (x3*)
	xDot3 = controlVector[1];

	if (dynamicsType == false) { // Default case
		// Calculate change in x-velocity drone - ACCELERATION (x4*)
		xDot4 = (1 / parametersList[1]) *	// Mass drone														
				(
				calculateThrustComponent(false, controlVector[0], stateVector[2])									// Thrust in x-drone
				- calculateDragComponent(false, parametersList[2], stateVector[3], stateVector[4])				    // Drag in x-drone
				);

		// Calculate change in y-velocity drone - ACCELERATION (x5*)					
		xDot5 = (1 / parametersList[1]) *	// Mass drone														
			(
				calculateThrustComponent(true, controlVector[0], stateVector[2])									// Thrust in y-drone
				- calculateDragComponent(true, parametersList[2], stateVector[3], stateVector[4])					// Drag in y-drone
				)																									
				- parametersList[0];																				// Gravity in y
	}
	else if (dynamicsType == true) { // With cargo
		// Calculate change in x-velocity drone - ACCELERATION (x4*)
		xDot4 = (1 / parametersList[1]) *	// Mass drone
				(
				calculateThrustComponent(false, controlVector[0], stateVector[2])									// Thrust in x-drone
				- calculateDragComponent(false, parametersList[2], stateVector[3], stateVector[4])					// Drag in x-drone
				- calculateRopeForceComponent(stateVector[0], stateVector[5], outputVector[0], outputVector[1],		// Rope force in x
					parametersList[3], parametersList[4], parametersList[5])										//   ...
				);

		// Calculate change in y-velocity drone - ACCELERATION (x5*)
		xDot5 = (1 / parametersList[1]) *	// Mass drone
			(
				calculateThrustComponent(true, controlVector[0], stateVector[2])									// Thrust in y-drone
				- calculateDragComponent(true, parametersList[2], stateVector[3], stateVector[4])					// Drag in y-drone
				- calculateRopeForceComponent(stateVector[1], stateVector[6], outputVector[0], outputVector[1],		// Rope force in y
					parametersList[3], parametersList[4], parametersList[5])										//   ...
				)
				- parametersList[0];																				// Gravity in y;
	}

	/* ---------------------------------------------------- CARGO ---------------------------------------------------- */

	if (dynamicsType == false) { // Default case
		xDot6 = 0;
		xDot7 = 0;
		xDot8 = 0;
		xDot9 = 0;
	}
	else if (dynamicsType == true) { // With cargo

		// Calculate change in x-position cargo - VELOCITY (x6*)
		xDot6 = stateVector[7];

		// Calculate change in y-position cargo - VELOCITY (x7*)
		xDot7 = stateVector[8];

		// Calculate change in x-velocity cargo - ACCELERATION (x8*)
		xDot8 = (1 / parametersList[6]) *	// Mass cargo
				(
				-calculateDragComponent(false, parametersList[7], stateVector[7], stateVector[8])					// Drag in x-cargo
				+ calculateRopeForceComponent(stateVector[0], stateVector[5], outputVector[0], outputVector[1],		// Rope force in x
											  parametersList[3], parametersList[4], parametersList[5])				//     ... 
				);

		// Calculate change in y-velocity cargo - ACCELERATION (x9*)
		xDot9 = (1 / parametersList[6]) *	// Mass cargo
				(
				-calculateDragComponent(true, parametersList[7], stateVector[7], stateVector[8])					// Drag in y-cargo
				+ calculateRopeForceComponent(stateVector[1], stateVector[6], outputVector[0], outputVector[1],		// Rope force in y
					parametersList[3], parametersList[4], parametersList[5])										//     ... 
				)
				- parametersList[0];																				// Gravity in y
	}
	/* ---------------------------------------------------------------------------------------------------------------- */

	// Construct vector
	dynamicsStateVector = { xDot1, xDot2, xDot3, xDot4, xDot5, xDot6, xDot7, xDot8, xDot9 };

	// Save computed dynamics vector to object
	return dynamicsStateVector;
}

/**
 * Computes the thrust component of the derivative equation calculateDerivativeStateVector()
 *
 * @param	periodicFunction :	a bool that represents the periodic function and (+/-) sign to use 
 * @param	torqueDrone : torque supplied to the drone
 * @param	angleDrone : angle of the drone
 * @return	A type (double) which is the computed result of the thrust component in the derivative equation
 */
double DroneRopeCargoDynamics::calculateThrustComponent(bool periodicFunction, double torqueDrone, double angleDrone) {
	/* HERE:
		periodicFunciton: false -> -sin
						  true --> cos
	*/

	// Initialize variable
	double thrustComponent{};

	// Compute
	//		SINE CASE
	if (periodicFunction == false) {
		thrustComponent = -torqueDrone * sin(angleDrone);
	}
	//		COSINE CASE
	else if (periodicFunction == true) {
		thrustComponent = torqueDrone * cos(angleDrone);
	}

	// Return value
	return thrustComponent;
}

/**
 * Computes the drag component of the derivative equation calculateDerivativeStateVector()
 *
 * @param	direction :	which component (x/y) is taken into consideration
 * @param	dragConstant : drag constant of an element
 * @param	xVelocity : horizontal velocity of an element
 * @param	yVelocity : vertical velocity of an element
 * @return	A type (double) which is the computed result of the drag component in the derivative equation
 */
double DroneRopeCargoDynamics::calculateDragComponent(bool direction, double dragConstant, double xVelocity, double yVelocity) {
	/* HERE:
		direction : false -> x
			   	    true --> y
	*/

	// Initialize variable
	double dragComponent{};

	// Compute
	if (direction == false) {
		dragComponent = dragConstant * sqrt(pow(xVelocity, 2) + pow(yVelocity, 2)) * xVelocity;
	}
	else if (direction == true) {
		dragComponent = dragConstant * sqrt(pow(xVelocity, 2) + pow(yVelocity, 2)) * yVelocity;
	}

	// Return value
	return dragComponent;
}

/**
 * Computes the rope force component of the derivative equation calculateDerivativeStateVector()
 * Makes use of calculateRopeForce() to calculate the total rope force
 *
 * @param	direction :	which component (x/y) is taken into consideration
 * @param	dronePosition : vertical or horizontal position of the drone (x/y)
 * @param	cargoPosition : vertical or horizontal position of the cargo (x/y)
 * @param	ropeLength : variable rope length which is specified by output vector
 * @param	ropeRateOfChange : variable rope rate of change which is specified by output vector
 * @param	ropeInitialLength : the initial length of the rope without taking drone-cargo dynamics into consideration
 * @param	ropeDamping : damping of the rope if it were a rod
 * @param	ropeStiffness : stiffness of the rope if it were a rod
 * @return	A type (double) which is the computed result of the rope force component in the derivative equation
 */
double DroneRopeCargoDynamics::calculateRopeForceComponent(double dronePosition, double cargoPosition, double ropeLength, double ropeRateOfChange, double ropeInitialLength, double ropeDamping, double ropeStiffness) {
	// Initialiaze variables
	double ropeForce{};
	double ropeForceComponent{};

	// Compute rope force
	ropeForce = calculateRopeForce(ropeLength, ropeRateOfChange, ropeInitialLength, ropeDamping, ropeStiffness);

	// Compute rope force component (account for division of zero)
	if (ropeLength == 0) {
		ropeForceComponent = 0;
	}
	else {
		ropeForceComponent = ropeForce * ((dronePosition - cargoPosition) / ropeLength);
	}

	// Return value
	return ropeForceComponent;
}

/**
 * Computes the rope force component of the derivative equation calculateDerivativeStateVector()
 * Makes use of calculateRopeForce() to calculate the total rope force
 *
 * @param	ropeLength : variable rope length which is specified by output vector
 * @param	ropeRateOfChange : variable rope rate of change which is specified by output vector
 * @param	ropeInitialLength : the initial length of the rope without taking drone-cargo dynamics into consideration
 * @param	ropeDamping : damping of the rope if it were a rod
 * @param	ropeStiffness : stiffness of the rope if it were a rod
 * @return	A type (double) which is the computed result of the total rope force needed for the rope force component in the derivative equation
 */
double DroneRopeCargoDynamics::calculateRopeForce(double ropeLength, double ropeRateOfChange, double ropeInitialLength, double ropeDamping, double ropeStiffness) {
	// Initialize variables
	double ropeForce{};
	double ropeForcePart1{}, ropeForcePart2{};

	// Compute
	ropeForcePart1 = ropeStiffness * (ropeLength - ropeInitialLength);
	ropeForcePart2 = ropeDamping * ropeRateOfChange;
	ropeForce = ropeForcePart1 + ropeForcePart2;

	// Check if rope force pulls or not
	if (ropeForce > 0) { // Pulls
		// Do nothing
	}
	else { // Pushes (but a rope cannot push so force is zero)
		ropeForce = 0;
	}

	// Return value
	return ropeForce;
}