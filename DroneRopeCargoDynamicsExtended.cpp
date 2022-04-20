//==//==============================================================
// Filename : DroneRopeCargoDynamicsExtended.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Extended class for dynamics of drone with   
//				 attached (slinging) cargo by rope - source
//==============================================================


// Libraries
#include "DroneRopeCargoDynamicsExtended.h"
#include <cmath>


// Constructor
DroneRopeCargoDynamicsExtended::DroneRopeCargoDynamicsExtended(double dynamicsType, double massDrone, double dragConstantDrone, double ropeLengthInitial, double ropeStiffness, double ropeDamping, double massCargo, double dragConstantCargo)
	: DroneRopeCargoDynamics(dynamicsType, massDrone, dragConstantDrone, ropeLengthInitial, ropeStiffness, ropeDamping, massCargo, dragConstantCargo) {}


// Getters (output vector: extended)
/**
 * Sets the extension of the output vector by first updating the current  
 * extension of the output vector and then returning it as a vector
 * 
 * @return A (std::vector<double>, attribute) of this class representing the extension of the output vector
 */
std::vector<double> DroneRopeCargoDynamicsExtended::getExtensionOutputVector() {
	// Update extension output vector
	setExtensionOutputVector();

	// Return vector
	return m_extensionOutputVector;
}


// Getters (output vector)
/**
 * Returns the output vector plus extension by retrieving both and concatenating them to one vector
 * 
 * @return	A (std::vector<double>) that is the output vector plus the made extension 
 */
std::vector<double> DroneRopeCargoDynamicsExtended::getOutputVector() {
	// Initialize total output vector
	std::vector<double> outputVector{};

	// Retrieve state vector from drone
	std::vector<double> baseOutputVector = DroneRopeCargoDynamics::getOutputVector();

	// Retrieve state vector from cargo
	std::vector<double> extendedOutputVector = getExtensionOutputVector();

	// Concatenate
	outputVector.reserve(outputVector.size() + baseOutputVector.size() + extendedOutputVector.size());  // Allocate memory for speed
	outputVector.insert(outputVector.end(), baseOutputVector.begin(), baseOutputVector.end());			// Rope length, rope rate of change --> total
	outputVector.insert(outputVector.end(), extendedOutputVector.begin(), extendedOutputVector.end());  // Rope angle  --> total

	// Return concatenated (total) state vector
	return outputVector;
}


// Setters (output vector)
/**
 * Sets the total output vector by first calculating these and then saving it to the object
 */
void DroneRopeCargoDynamicsExtended::setOutputVector() {
	// Calculate and save new [rope length] + [rope rate of change] to object
	DroneRopeCargoDynamics::setOutputVector();

	// Calculate and save new [rope angle] to object
	setExtensionOutputVector();
};

// Setters (output vector: extension)
/**
 * Sets the extension of the output vector by first calculating it and then saving it to the object
 */
void DroneRopeCargoDynamicsExtended::setExtensionOutputVector() {

	// Initialize vector
	std::vector<double> extensionOutputVector = calculateExtensionOutputVector();

	// Save rope angle to vector
	setRopeAngle(extensionOutputVector[0]);
}

void DroneRopeCargoDynamicsExtended::setRopeAngle(double ropeAngle) {
	m_extensionOutputVector[0] = ropeAngle;
}


// Calculate (output vector: extended)
/**
 * Returns the extension of the output vector
 *
 * @return	A type (std::vector<double>) which is the extension of the output vector
 */
std::vector<double> DroneRopeCargoDynamicsExtended::calculateExtensionOutputVector() {
	// Initialize vector
	std::vector<double> extensionOutputVector;

	// Calculate angle
	double ropeAngle = calculateRopeAngle();

	// Construct vector
	extensionOutputVector = { ropeAngle };

	// Return vector
	return extensionOutputVector;
}

// Calculate (rope)
/**
 * Computes the rope angle relative to the drone
 *
 * @return	A type (double) which is the computed result of the rope angle
 */
double DroneRopeCargoDynamicsExtended::calculateRopeAngle() {
	/* NOTATIONS:
		x1 : xDrone        
		x2 : yDrone          
		x6 : xCargo
		x7 : yCargo
	*/

	// Initialize variables
	double ropeAngle{};
	double ropeAnglePart1{}, ropeAnglePart2{};

	// Compute
	ropeAnglePart1 = getXCargo() - getXDrone();
	ropeAnglePart2 = getYCargo() - getYDrone();
	ropeAngle = atan2(ropeAnglePart1, ropeAnglePart2);

	// Return value
	return ropeAngle;
}