//==============================================================
// Filename : DroneDynamics.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone dynamics - source
//==============================================================

// Libraries
#include "DroneDynamics.h"

// Constructor
DroneDynamics::DroneDynamics(double massDrone, double dragConstantDrone)
	: DroneProperties(massDrone, dragConstantDrone) {}


// Setters (state vector)
void DroneDynamics::setDroneStateVector(std::vector<double> droneStateVector) {
	m_droneStateVector = droneStateVector;
}

void DroneDynamics::setXDrone(double xDrone) {
	m_droneStateVector[0] = xDrone;
};

void DroneDynamics::setYDrone(double yDrone) {
	m_droneStateVector[1] = yDrone;
};

void DroneDynamics::setThetaDrone(double thetaDrone) {
	m_droneStateVector[2] = thetaDrone;
};

void DroneDynamics::setXDotDrone(double xDotDrone) {
	m_droneStateVector[3] = xDotDrone;
};

void DroneDynamics::setYDotDrone(double xDotDrone) {
	m_droneStateVector[4] = xDotDrone;
};


// Setters (control vector)
void DroneDynamics::setDroneControlVector(std::vector<double> droneControlVector) {
	m_droneControlVector = droneControlVector;
}

void DroneDynamics::setTauDrone(double tauDrone) {
	m_droneControlVector[0] = tauDrone;
};

void DroneDynamics::setOmegaDrone(double omegaDrone) {
	m_droneControlVector[1] = omegaDrone;
};