//==============================================================
// Filename : DroneControllerControlVector.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for computing the required control vector
//				 for feedback control of the simulator - source
//==============================================================

// Libraries
#include "DroneControllerControlVector.h"
#include <cmath>

// Constructor
DroneControllerControlVector::DroneControllerControlVector(double timeConstantX, double timeConstantY, double timeConstantTheta, double oscillationDampingConstant)
	: DroneControllerForce(timeConstantX, timeConstantY, timeConstantTheta, oscillationDampingConstant) {}


// Calculate (control vector: reference)
/**
 * Retrieves the reference control vector for the drone (+ cargo) system
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
 * @return	A (std::vector<double>) which represents the required control vector based on given parameters
 */
std::vector<double> DroneControllerControlVector::calculateReferenceControlVector(bool dynamicsType, double gravitationalConstant,
																				  double massDrone, double xDrone, double xDotDrone, double yDrone, double yDotDrone, double thetaDrone,
																				  double massCargo, double xCargo, double yCargo) 
{
	// Initialize vector
	std::vector<double> referenceControlVector{};
	
	// Compute reference control [torque] based on given properties and states of drone (+ cargo)
	double referenceTau = calculateReferenceTau(gravitationalConstant, dynamicsType,
												massDrone, xDrone, xDotDrone, yDrone, yDotDrone, thetaDrone,
												massCargo, xCargo, yCargo);

	// Compute reference control [rotational velocity] based on given properties and states of drone (+ cargo)
	double referenceOmega = calculateReferenceOmega(gravitationalConstant, dynamicsType,
													massDrone, xDrone, xDotDrone, yDrone, yDotDrone, thetaDrone,
													massCargo, xCargo, yCargo);

	// Construct vector
	referenceControlVector = { referenceTau, referenceOmega };

	// Return vector
	return referenceControlVector;
;}


/**
 * Computes and retrieves the required torque for the drone, specified in the control vector
 *
 * @param	gravitationalConstant : gravitational constant of the environment
 * @param	dynamicsType : specifies the force required for drone only (false), or drone and cargo (true)
 * @param	massDrone : mass of the drone
 * @param	xDrone : horizontal position of the drone
 * @param	xDotDrone : horizontal velocity of the drone
 * @param	yDrone : vertical position of the drone
 * @param	yDotDrone : vertical velocity of the drone
 * @param	thetaDrone : angle of the drone w.r.t. to horizontal line passing through centre of gravity of drone
 * @param	massCargo : mass of the cargo
 * @param	xCargo : horizontal position of the cargo
 * @param	yCargo : vertical position of the cargo
 * 
 * @return	A (double) which is the required reference torque for the control vector
 */
double DroneControllerControlVector::calculateReferenceTau(bool dynamicsType, double gravitationalConstant,
														   double massDrone, double xDrone, double xDotDrone, double yDrone, double yDotDrone, double thetaDrone,
														   double massCargo, double xCargo, double yCargo) 
{
	// Initialize variable
	double referenceTau{};

	// Retrieve reference force vector
	std::vector<double> referenceForceVector = calculateReferenceForceVector(gravitationalConstant, dynamicsType,
																			 massDrone, xDrone, xDotDrone, yDrone, yDotDrone,
																			 massCargo, xCargo, yCargo);

	// Compute (account for division by zero)
	if (cos(thetaDrone) == 0) {
		referenceTau = 0;
	}
	else {
		referenceTau = referenceForceVector[0] / cos(thetaDrone);
	}

	// Return value
	return referenceTau;
}


/**
 * Computes and retrieves the required angular velocity for the drone, specified in the control vector
 *
 * @param	gravitationalConstant : gravitational constant of the environment
 * @param	dynamicsType : specifies the force required for drone only (false), or drone and cargo (true)
 * @param	massDrone : mass of the drone
 * @param	xDrone : horizontal position of the drone
 * @param	xDotDrone : horizontal velocity of the drone
 * @param	yDrone : vertical position of the drone
 * @param	yDotDrone : vertical velocity of the drone
 * @param	thetaDrone : angle of the drone w.r.t. to horizontal line passing through centre of gravity of drone
 * @param	massCargo : mass of the cargo
 * @param	xCargo : horizontal position of the cargo
 * @param	yCargo : vertical position of the cargo
 * 
 * @return	A (double) which is the required reference torque for the control vector
 */
double DroneControllerControlVector::calculateReferenceOmega(bool dynamicsType, double gravitationalConstant,
															 double massDrone, double xDrone, double xDotDrone, double yDrone, double yDotDrone, double thetaDrone,
															 double massCargo, double xCargo, double yCargo) 
{
	// Initialize variables
	double referenceOmega{};
	double referenceTheta = calculateReferenceTheta(gravitationalConstant, dynamicsType, 
													massDrone, xDrone, xDotDrone, yDrone, yDotDrone, 
													massCargo, xCargo, yCargo);

	// Compute
	referenceOmega = (1 / getTimeConstantTheta()) * (referenceTheta - thetaDrone);

	// Return value
	return referenceOmega;
}


/**
 * A helper function for calculateReferenceOmega(), which computes the reference angle of the drone
 *
 * @param	gravitationalConstant : gravitational constant of the environment
 * @param	dynamicsType : specifies the force required for drone only (false), or drone and cargo (true)
 * @param	massDrone : mass of the drone
 * @param	xDrone : horizontal position of the drone
 * @param	xDotDrone : horizontal velocity of the drone
 * @param	yDrone : vertical position of the drone
 * @param	yDotDrone : vertical velocity of the drone
 * @param	thetaDrone : angle of the drone w.r.t. to horizontal line passing through centre of gravity of drone
 * @param	massCargo : mass of the cargo
 * @param	xCargo : horizontal position of the cargo
 * @param	yCargo : vertical position of the cargo
 * 
 * @return	A (double) which is the required reference angle 
 */
double DroneControllerControlVector::calculateReferenceTheta(bool dynamicsType, double gravitationalConstant,
															 double massDrone, double xDrone, double xDotDrone, double yDrone, double yDotDrone,
															 double massCargo, double xCargo, double yCargo)
{
	// Initialize variable
	double referenceTheta{};

	// Retrieve reference force vector
	std::vector<double> referenceForceVector = calculateReferenceForceVector(gravitationalConstant, dynamicsType,
																			 massDrone, xDrone, xDotDrone, yDrone, yDotDrone,
																			 massCargo, xCargo, yCargo);

	// Compute 
	referenceTheta = atan2(-referenceForceVector[0], referenceForceVector[1]);

	// Return value
	return referenceTheta;
}