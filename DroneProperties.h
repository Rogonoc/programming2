//==============================================================
// Filename : DroneProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone properties - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONEPROPERTIES_H
#define DRONEPROPERTIES_H


// DroneProperties-class
class DroneProperties {
public:
	// Constructor (default)
	DroneProperties() = default;

	// Constructor (with arguments)
	DroneProperties(double massDrone, double dragConstantDrone);

	// Getters (drone)
	double getMassDrone() const { return m_massDrone; }

	// Setters (drone)
	void setConstantDroneParameters(double, double); // Main
	void setMassDrone(double);
	void setDragConstantDrone(double);

private:
	// Attributes (drone)
	double m_massDrone;
	double m_dragConstantDrone;
};


// [END]: Prevent multiple inclusions of header
#endif