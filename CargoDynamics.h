//==============================================================
// Filename : CargoDynamics.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for cargo dynamics - header
//==============================================================


// [BEGIN]: Prevent multiple inclusions of header
#ifndef CARGODYNAMICS_H
#define CARGODYNAMICS_H


// Libraries
#include "CargoProperties.h"
#include "GravitationalConstants.h"
#include <vector>

// CargoDynamics-class
class CargoDynamics : public CargoProperties {
public:
	// Constructor (default)
	CargoDynamics() = default;

	// Constructor (with arguments)
	CargoDynamics(double massCargo, double dragConstantCargo);


	// Getters
	std::vector<double> getCargoStateVector() const { return m_cargoStateVector; } // Main
	double getXCargo() const { return m_cargoStateVector[0]; }
	double getYCargo() const { return m_cargoStateVector[1]; }
	double getXDotCargo() const { return m_cargoStateVector[2]; }
	double getYDotCargo() const { return m_cargoStateVector[3]; }


	// Setters
	void setCargoStateVector(std::vector<double>); // Main
	void setXCargo(double);
	void setYCargo(double);
	void setXDotCargo(double);
	void setYDotCargo(double);

private:
	// Attributes
	std::vector<double> m_cargoStateVector = { 0, 0, 0, 0 }; // x6 - x9
};


// [END]: Prevent multiple inclusions of header
# endif