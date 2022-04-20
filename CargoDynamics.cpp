//==============================================================
// Filename : CargoDynamics.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for cargo dynamics - source
//==============================================================

// Libraries 
#include "CargoDynamics.h"
#include <cmath>


// Constructor
CargoDynamics::CargoDynamics(double massCargo, double dragConstantCargo)
	: CargoProperties(massCargo, dragConstantCargo) {};


// Setters (state vector: cargo)
void CargoDynamics::setCargoStateVector(std::vector<double> cargoStateVector) { // Main
	m_cargoStateVector = cargoStateVector;
}

void CargoDynamics::setXCargo(double xCargo) {
	m_cargoStateVector[0] = xCargo;
};

void CargoDynamics::setYCargo(double yCargo) {
	m_cargoStateVector[1] = yCargo;
};

void CargoDynamics::setXDotCargo(double xDotCargo) {
	m_cargoStateVector[2] = xDotCargo;
};

void CargoDynamics::setYDotCargo(double yDotCargo) {
	m_cargoStateVector[3] = yDotCargo;
};