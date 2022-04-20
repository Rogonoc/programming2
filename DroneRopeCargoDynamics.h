//==============================================================
// Filename : DroneRopeCargoDynamics.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for dynamics of drone with attached  
//				 (slinging) cargo by rope - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONEROPECARGODYNAMICS_H
#define DRONEROPECARGODYNAMICS_H


// Libraries
#include "DroneDynamics.h"
#include "RopeProperties.h"
#include "CargoDynamics.h"
#include "GravitationalConstants.h"

// DroneRopeCargoDynamics-class
class DroneRopeCargoDynamics : public DroneDynamics, public RopeProperties, public CargoDynamics, public GravitationalConstants {
public:
	// Constructor (default)
	DroneRopeCargoDynamics() = default;

	// Constructor (with arguments)
	DroneRopeCargoDynamics(double dynamicsType, double massDrone, double dragConstantDrone, double ropeLengthInitial, double ropeStiffness, double ropeDamping, double massCargo, double dragConstantCargo);

	// Destructor (virtual)
	virtual ~DroneRopeCargoDynamics() {}


	// Getters (dynamics type)
	bool getDynamicsType() const { return m_dynamicsType; }

	// Getters (state vector)
	std::vector<double> getStateVector();

	// Getters (output vector)
	virtual std::vector<double> getOutputVector(); // Main
	double getRopeLength();
	double getRopeRateOfChange();

	// Setters (dynamics type)
	void setDynamicsType(bool);

	// Setters (state vector)
	void setStateVector(std::vector<double>);

	// Setters (output vector)
	virtual void setOutputVector();
	void setRopeLength(double);
	void setRopeRateOfChange(double);

	// Calculate (state derivative)
	static std::vector<double> calculateDerivativeStateVector(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool);

private:
	// Attributes (dynamics type)
	bool m_dynamicsType;

	// Attributes (output vector)
	std::vector<double> m_outputVector = { 0, 0 };

	// Calculate (output vector)
	std::vector<double> calculateOutputVector();

	// Helper functions for calculateDerivativeStateVector()
	static double calculateThrustComponent(bool, double, double);
	static double calculateDragComponent(bool, double, double, double);
	static double calculateRopeForceComponent(double, double, double, double, double, double, double);
		static double calculateRopeForce(double, double, double, double, double); // Overall rope force
};


// [END]: Prevent multiple inclusions of header
# endif