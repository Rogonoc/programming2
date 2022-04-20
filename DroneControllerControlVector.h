//==============================================================
// Filename : DroneControllerControlVector.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for computing the required control vector
//				 for feedback control of the simulator - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef DRONECONTROLLERCONTROLVECTOR_H
#define DRONECONTROLLERCONTROLVECTOR_H


// Libraries
#include "DroneControllerForce.h"

// DroneControllerControlVector - class
class DroneControllerControlVector : public DroneControllerForce {
public:
	// Constructor (default)
	DroneControllerControlVector() = default;

	// Constructor (with arguments)
	DroneControllerControlVector(double timeConstantX, double timeConstantY, double timeConstantTheta, double oscillationDampingConstant);

	// Calculate (control vector: reference)
	std::vector<double> calculateReferenceControlVector(bool, double, double, double, double, double, double, double, double, double, double);

private:
	// Helper functions for calculateReferenceControlVector()
	double calculateReferenceTau(bool, double, double, double, double, double, double, double, double, double, double);
	double calculateReferenceOmega(bool, double, double, double, double, double, double, double, double, double, double);
		double calculateReferenceTheta(bool, double, double, double, double, double, double, double, double, double);
};


// [END]: Prevent multiple inclusions of header
# endif