//==============================================================
// Filename : RopeProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for rope properties - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef ROPEPROPERTIES_H
#define ROPEPROPERTIES_H


// Libraries
#include "RopeProperties.h"

// RopeProperties-class
class RopeProperties {
public:
	// Constructor (default)
	RopeProperties() = default;

	// Constructor (with arguments)
	RopeProperties(double ropeLength, double ropeStiffness, double ropeDamping);

	// Getters
	double getRopeLengthInitial() const { return m_ropeLengthInitial; }
	double getRopeStiffness() const { return m_ropeStiffness; }
	double getRopeDamping() const { return m_ropeDamping; }

	// Setters
	void setConstantRopeParameters(double, double, double); // Main
	void setRopeLengthInitial(double);
	void setRopeStiffness(double);
	void setRopeDamping(double);

private:
	// Attributes
	double m_ropeLengthInitial;
	double m_ropeStiffness;
	double m_ropeDamping;
};


// [END]: Prevent multiple inclusions of header
# endif