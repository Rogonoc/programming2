//==============================================================
// Filename : DroneRopeCargoDynamicsExtended.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Extended class for dynamics of drone with   
//				 attached (slinging) cargo by rope - header
//==============================================================


// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONEROPECARGODYNAMICSEXTENDED_H
#define DRONEROPECARGODYNAMICSEXTENDED_H


// Libraries
#include "DroneRopeCargoDynamics.h"

// DroneRopeCargoDynamicsExtended-class
class DroneRopeCargoDynamicsExtended : public DroneRopeCargoDynamics {
public:
	// Constructor (default)
	DroneRopeCargoDynamicsExtended() = default;

	// Constructor (with arguments)
	DroneRopeCargoDynamicsExtended(double dynamicsType, double massDrone, double dragConstantDrone, double massCargo, double dragConstantCargo, double ropeLengthInitial, double ropeStiffness, double ropeDamping);

	// Destructor (virtual)
	virtual ~DroneRopeCargoDynamicsExtended() {}


	// Getters (output vector)
	virtual std::vector<double> getOutputVector();

	// Getters (output vector: extended)
	std::vector<double> getExtensionOutputVector();
	double getRopeAngle() const { return m_extensionOutputVector[0]; }


	// Setters (output vector)
	virtual void setOutputVector();

	// Setters (output vector: extended)
	void setExtensionOutputVector();
	void setRopeAngle(double);

private:
	// Attributes (output vector: extended)
	std::vector<double> m_extensionOutputVector = {0};

	// Calculate (output vector: extended)
	std::vector<double> calculateExtensionOutputVector();

	// Calculate (rope)
	double calculateRopeAngle();
};


// [END]: Prevent multiple inclusions of header
# endif