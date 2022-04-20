//==============================================================
// Filename : RopeProperties.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for rope properties - source
//==============================================================

// Libraries
#include "RopeProperties.h"

// Constructor
RopeProperties::RopeProperties(double ropeLengthInitial, double ropeStiffness, double ropeDamping) {
	// Set attributes
	setRopeLengthInitial(ropeLengthInitial);
	setRopeStiffness(ropeStiffness);
	setRopeDamping(ropeDamping);
}


// Setters
void RopeProperties::setConstantRopeParameters(double ropeLengthInitial, double ropeStiffness, double ropeDamping) { // Main
	setRopeLengthInitial(ropeLengthInitial);
	setRopeStiffness(ropeStiffness);
	setRopeDamping(ropeDamping);
}

void RopeProperties::setRopeLengthInitial(double ropeLengthInitial) {
	m_ropeLengthInitial = ropeLengthInitial;
};

void RopeProperties::setRopeStiffness(double ropeStiffness) {
	m_ropeStiffness = ropeStiffness;
};

void RopeProperties::setRopeDamping(double ropeDamping) {
	m_ropeDamping = ropeDamping;
};