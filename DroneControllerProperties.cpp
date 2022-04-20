//==============================================================
// Filename : DroneControllerProperties.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone controller properties - source
//==============================================================

// Libraries
#include "DroneControllerProperties.h"

// Constructor (with arguments)
DroneControllerProperties::DroneControllerProperties(double timeConstantX, double timeConstantY, double timeConstantTheta) {
	// Set attributes
	setTimeConstantX(timeConstantX);
	setTimeConstantY(timeConstantY);
	setTimeConstantTheta(timeConstantTheta);
}

// Setters (time constants)
/**
 *	Sets the input time constants (for x and y) to the object 
 *
 *	@param	timeConstantX :  time constant for a horizontal component of drone describing how fast it reacts
 *	@param	timeConstantY :  time constant for a vertical component of drone describing how fast it reacts
 */
void DroneControllerProperties::setTimeConstants(double timeConstantX, double timeConstantY, double timeConstantTheta) {
	setTimeConstantX(timeConstantX);
	setTimeConstantY(timeConstantY);
	setTimeConstantTheta(timeConstantTheta);
}

void DroneControllerProperties::setTimeConstantX(double timeConstantX) {
	m_timeConstantX = timeConstantX;
}

void DroneControllerProperties::setTimeConstantY(double timeConstantY) {
	m_timeConstantY = timeConstantY;
}

void DroneControllerProperties::setTimeConstantTheta(double timeConstantTheta) {
	m_timeConstantTheta = timeConstantTheta;
}