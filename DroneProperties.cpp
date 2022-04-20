//==============================================================
// Filename : DroneProperties.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone properties - source
//==============================================================

// Libraries
#include "DroneProperties.h"


// Constructor
DroneProperties::DroneProperties(double massDrone, double dragConstantDrone) {
	// Set attributes
	setMassDrone(massDrone);
	setDragConstantDrone(dragConstantDrone);
}


// Setters (drone characteristics)
void DroneProperties::setConstantDroneParameters(double massDrone, double dragConstantDrone) { // Main
	setMassDrone(massDrone);
	setDragConstantDrone(dragConstantDrone);
}
void DroneProperties::setMassDrone(double massDrone) {
	m_massDrone = massDrone;
}

void DroneProperties::setDragConstantDrone(double dragConstantDrone) {
	m_dragConstantDrone = dragConstantDrone;
};