//==============================================================
// Filename : DroneRopeCargoSimulator.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class containing object to simulate the behavior 
//				 a drone with or without cargo, using different
//				 integration methods - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONEROPECARGOSIMULATOR_H
#define DRONEROPECARGOSIMULATOR_H


// Libraries
#include "DroneRopeCargoDynamicsExtended.h"
#include "NumericalIntegrationMethods.h"

// DroneDynamicsPlusIntegration-class
class DroneRopeCargoSimulator : public DroneRopeCargoDynamicsExtended, public NumericalIntegrationMethods {
public:
	// Constructor (default)
	DroneRopeCargoSimulator() = default;
	
	// Setters (implementation)
	void setImplementation(bool dynamicsType, bool integrationType);

	// Other 
	std::vector<double> simulationStep(std::vector<double>);

private:
	// Attributes (implementation)	
	int m_implementationType;
};


// [END]: Prevent multiple inclusions of header
#endif