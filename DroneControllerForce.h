//==============================================================
// Filename : DroneControllerForce.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for computing the required force components
//				 of the drone controller - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONECONTROLLERFORCE_H
#define DRONECONTROLLERFORCE_H


// Libraries
#include "DroneControllerProperties.h"
#include <vector>


// DroneControllerForce-class
class DroneControllerForce : public DroneControllerProperties {
public:
	// Constructor (default)
	DroneControllerForce() = default;

	// Constructor (with arguments)
	DroneControllerForce(double timeConstantX, double timeConstantY, double timeConstantTheta, double oscillationDampingConstant);


	// Getters (controller behavior)
	double getOscillationDampingConstant() const { return m_oscillationDampingConstant; }

	// Getters (velocity vector)
	std::vector<double> getVelocityVector() const { return m_velocityVector; }


	// Setters (controller behavior)
	void setOscillationDampingConstant(double);

	// Setters (velocity vector)
	void setVelocityVector(std::vector<double>);


	// Calculate (force vector)
	std::vector<double> calculateReferenceForceVector(bool, double, double, double, double, double, double, double, double, double);

private:
	// Attributes (controller behavior)
	double m_oscillationDampingConstant;

	// Attributes (velocity vector)
	std::vector<double> m_velocityVector = {0, 0};


	// Helper functions for calculateReferenceForceVector()
	double calculateDynamicsForceComponent(bool, double, double, double, double);
	double calculateCargoDampingForceComponent(double, double);
	double calculateGravitationalForceComponent(double, double, double);
};


// [END]: Prevent multiple inclusions of header
# endif