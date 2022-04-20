//==============================================================
// Filename : DroneControllerForce.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for computing the required force components
//				 of the drone controller - source
//==============================================================

// Libraries
#include "DroneControllerForce.h"

// Constructor
DroneControllerForce::DroneControllerForce(double timeConstantX, double timeConstantY, double timeConstantTheta, double oscillationDampingConstant)
	: DroneControllerProperties(timeConstantX, timeConstantY, timeConstantTheta) {
	// Set attributes
	setOscillationDampingConstant(oscillationDampingConstant);
}


// Setters (controller behavior)
void DroneControllerForce::setOscillationDampingConstant(double oscillationDampingConstant) {
	m_oscillationDampingConstant = oscillationDampingConstant;
}

// Setters (velocity vector)
void DroneControllerForce::setVelocityVector(std::vector<double> velocityVector) {
	m_velocityVector = velocityVector;
}


// Calculate (force vector)
/**
 * Computes the required reference force components for the drone.
 * Returned value is given in vector format and contains directional force components (0 is F_x, and 1 is F_y).
 * Always set a velocity vector to the object using setVelocityVector(), before using this function
 * 
 * @param	gravitationalConstant : gravitational constant of the environment
 * @param	dynamicsType : specifies the force required for drone only (false), or drone and cargo (true)
 * @param	massDrone : mass of the drone
 * @param	xDrone : horizontal position of the drone
 * @param	xDotDrone : horizontal velocity of the drone
 * @param	yDrone : vertical position of the drone
 * @param	yDotDrone : vertical velocity of the drone
 * @param	massCargo : mass of the cargo
 * @param	xCargo : horizontal position of the cargo
 * @param	yCargo : vertical position of the cargo
 *
 * @return	A (double) which is the required reference force vector based on given velocities	
 */
std::vector<double> DroneControllerForce::calculateReferenceForceVector(bool dynamicsType, double gravitationalConstant,
																		double massDrone, double xDrone, double xDotDrone, double yDrone, double yDotDrone,
																		double massCargo, double xCargo, double yCargo) 
{
	// Initialize vectors
	std::vector<double> referenceForceVector{};
	std::vector<double> referenceVelocityVector = getVelocityVector();

	/*-----------------------------------------------------------------------------*/

	// Compute horizontal component of required force
	double horizontalForceComponent{};
	if (dynamicsType == false) { // Default case (HERE: cargo mass is set to 0)
		horizontalForceComponent = calculateDynamicsForceComponent(false, massDrone, 0, xDotDrone, referenceVelocityVector[0]);			// Component due to user-specified dynamics
	}
	else if (dynamicsType == true) { // With cargo
		horizontalForceComponent = calculateDynamicsForceComponent(false, massDrone, massCargo, xDotDrone, referenceVelocityVector[0])	// Component due to user-specified dynamics
								   + calculateCargoDampingForceComponent(xDrone, xCargo);												// Component due to required controller damping for rope
	}

	/*-----------------------------------------------------------------------------*/

	// Compute vertical component of required force
	double verticalForceComponent{};
	if (dynamicsType == false) { // Default case (HERE: cargo mass is set to 0)
		verticalForceComponent = calculateDynamicsForceComponent(true, massDrone, 0, yDotDrone, referenceVelocityVector[1])			// Component due to user-specified dynamics
								 + calculateGravitationalForceComponent(gravitationalConstant, massDrone, 0);						// Component due to required gravity compensation for rope
	}
	else if (dynamicsType == true) { // With cargo
		verticalForceComponent = calculateDynamicsForceComponent(true, massDrone, massCargo, yDotDrone, referenceVelocityVector[1])	// Component due to user-specified dynamics
								 + calculateGravitationalForceComponent(gravitationalConstant, massDrone, massCargo);				// Component due to required gravity compensation for rope
	}

	/*-----------------------------------------------------------------------------*/

	// Construct vector
	referenceForceVector = { horizontalForceComponent, verticalForceComponent };

	// Return vector
	return referenceForceVector;
}


/**
 * Helper function for calculateReferenceForce(); computes the force component due to user-specified velocities
 *
 * @param	massDrone : mass of the drone
 * @param	massCargo : mass of the cargo
 * @param	velocityDrone : directional velocity of the drone
 * @param	referenceVelocityDrone : required reference velocity
 *
 * @return	A (double) which is the dynamics component in the force equation
 */
double DroneControllerForce::calculateDynamicsForceComponent(bool direction, double massDrone, double massCargo, double velocityDrone, double referenceVelocityDrone) {
	/*
		HERE (direction):
			false --> x
			true --> y
	*/
	
	// Initialize variable
	double dynamicsForceComponent{};

	// Compute
	if (direction == false) { // x-direction
		dynamicsForceComponent = ((massDrone + massCargo) / getTimeConstantX()) * (referenceVelocityDrone - velocityDrone);
	}
	else if (direction == true) { // y-direction
		dynamicsForceComponent = ((massDrone + massCargo) / getTimeConstantY()) * (referenceVelocityDrone - velocityDrone);
	}

	// Return value
	return dynamicsForceComponent;
}


/**
 * Helper function for calculateReferenceForce(); computes the required force component to damp the swinging motion of the cargo
 *
 * @param	oscillationDampingConstant : damping for the oscillations
 * @param	massDrone : mass of the drone
 * @param	massCargo : mass of the cargo
 *
 * @return	A (double) which is the damping component in the force equation
 */
double DroneControllerForce::calculateCargoDampingForceComponent(double xDrone, double xCargo) {
	// Initialize variable
	double cargoDampingForceComponent{};

	// Compute
	cargoDampingForceComponent = getOscillationDampingConstant() * (xDrone - xCargo);

	// Return value
	return cargoDampingForceComponent;
}


/**
 * Helper function for calculateReferenceForce(); computes the force component to compensate for gravity
 *
 * @param	gravitationalConstant : gravitational constant of the environment
 * @param	massDrone : mass of the drone
 * @param	massCargo : mass of the cargo
 *
 * @return	A (double) which is the gravitational component in the force equation
 */
double DroneControllerForce::calculateGravitationalForceComponent(double gravitationalAcceleration, double massDrone, double massCargo) {
	// Initialize variable
	double gravityForceComponent{};

	// Compute
	gravityForceComponent = (massDrone + massCargo) * gravitationalAcceleration;

	// Return value
	return gravityForceComponent;
}