//==============================================================
// Filename : CargoProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for cargo properties - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef CARGOPROPERTIES_H
#define CARGOPROPERTIES_H

// CargoProperties-class
class CargoProperties {
public:
	// Constructor (default)
	CargoProperties() = default;

	// Constructor (with arguments)
	CargoProperties(double massCargo, double dragConstantCargo);

	// Getters
	double getMassCargo() const { return m_massCargo; }
	double getDragConstantCargo() const { return m_dragConstantCargo; }

	// Setters
	void setConstantCargoParameters(double, double); // Main
	void setMassCargo(double);
	void setDragConstantCargo(double);

private:
	// Attributes 
	double m_massCargo;
	double m_dragConstantCargo;
};


// [END]: Prevent multiple inclusions of header
# endif