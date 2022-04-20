//==============================================================
// Filename : DroneDynamics.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone dynamics - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONEDYNAMICS_H
#define DRONEDYNAMICS_H

// Libraries
#include "DroneProperties.h"
#include "GravitationalConstants.h"
#include <vector>

// DroneDynamics-class
class DroneDynamics : public DroneProperties {
public:
	// Constructor (default)
	DroneDynamics() = default;

	// Constructor (with arguments)
	DroneDynamics(double massDrone, double dragConstantDrone);


	// Getters (state vector)
	std::vector<double> getDroneStateVector() const { return m_droneStateVector; } // Main
	double getXDrone() const { return m_droneStateVector[0]; }
	double getYDrone() const { return m_droneStateVector[1]; }
	double getThetaDrone() const { return m_droneStateVector[2]; }
	double getXDotDrone() const { return m_droneStateVector[3]; }
	double getYDotDrone() const { return m_droneStateVector[4]; }

	// Getters (control vector)
	std::vector<double> getDroneControlVector() { return m_droneControlVector; } // Main
	double getTauDrone() const { return m_droneControlVector[0]; }
	double getOmegaDrone() const { return m_droneControlVector[1]; }


	// Setters (state vector)
	void setDroneStateVector(std::vector<double>); // Main
	void setXDrone(double);
	void setYDrone(double);
	void setThetaDrone(double);
	void setXDotDrone(double);
	void setYDotDrone(double);

	// Setters (control vector)
	void setDroneControlVector(std::vector<double>); // Main
	void setTauDrone(double);
	void setOmegaDrone(double);

private:
	// Attributes (state vector)
	std::vector<double> m_droneStateVector = { 0, 0, 0, 0, 0 }; // x1 - x5

	// Attributes (control vector)
	std::vector<double> m_droneControlVector = { 0, 0 }; // u1 - u2
};


// [END]: Prevent multiple inclusions of header
# endif