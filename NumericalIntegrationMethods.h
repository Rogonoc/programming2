//==============================================================
// Filename : NumericalIntegrationMethods.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class for combining and switching numerical 
//				 integration methods - header 
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef NUMERICALINTEGRATIONMETHODS_H
#define NUMERICALINTEGRATIONMETHODS_H


// Libraries
#include "RungeKuttaFourNumericalIntegration.h"
#include "EulerNumericalIntegration.h"

// NumericalIntegrationBase-class
class NumericalIntegrationMethods : public RungeKuttaFourNumericalIntegration,  public EulerNumericalIntegration {
public:
	// Constructor (default)
	NumericalIntegrationMethods() = default;

	// Constructor (with arguments)
	NumericalIntegrationMethods(double timeStep, double integrationType);

	// Getters
	bool getIntegrationType() { return m_integrationType; }

	// Setters
	void setIntegrationType(bool);

	// Calculate
	std::vector<double> calculateNextState(std::function<std::vector<double>(std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, bool)>, 
										   std::vector<double>, std::vector<double>, std::vector<double>, 
										   std::vector<double>, bool);

private:
	// Attributes
	bool m_integrationType;
};


// [END]: Prevent multiple inclusions of header
#endif