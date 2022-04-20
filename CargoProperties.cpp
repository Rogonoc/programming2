//==============================================================
// Filename : CargoProperties.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for cargo properties - source
//==============================================================

// Libraries
#include "CargoProperties.h"


// Constructor
CargoProperties::CargoProperties(double massCargo, double dragConstantCargo) {
	// Set attributes
	setMassCargo(massCargo);
	setDragConstantCargo(dragConstantCargo);
}


// Setters (cargo)
void CargoProperties::setConstantCargoParameters(double massCargo, double dragConstantCargo) { // Main
	setMassCargo(massCargo);
	setDragConstantCargo(dragConstantCargo);
}

void CargoProperties::setMassCargo(double massCargo) {
	m_massCargo = massCargo;
};

void CargoProperties::setDragConstantCargo(double dragConstantCargo) {
	m_dragConstantCargo = dragConstantCargo;
};