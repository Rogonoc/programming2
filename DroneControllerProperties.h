//==============================================================
// Filename : DroneControllerProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for drone controller properties - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONECONTROLLERPROPERTIES_H
#define DRONECONTROLLERPROPERTIES_H


// DroneControllerProperties-class
class DroneControllerProperties {
public:
	// Constructor (default)
	DroneControllerProperties() = default;

	// Constructor (with arguments)
	DroneControllerProperties(double timeConstantX, double timeConstantY, double timeConstantTheta);

	// Getters (time constants)
	double getTimeConstantX() const { return m_timeConstantX; }
	double getTimeConstantY() const { return m_timeConstantY; }
	double getTimeConstantTheta() const { return m_timeConstantTheta; }

	// Setters (time constants)
	void setTimeConstants(double, double, double);
	void setTimeConstantX(double);
	void setTimeConstantY(double);
	void setTimeConstantTheta(double);

private:
	// Attributes (time constants)
	double m_timeConstantX;
	double m_timeConstantY;
	double m_timeConstantTheta;
};



// [END]: Prevent multiple inclusions of header
# endif