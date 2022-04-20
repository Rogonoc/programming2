
// Libraries
#include "DroneRopeCargoSimulator.h"

int main()
{
	/* ---------------------------------- OBJECT ---------------------------------- */

	// Initialize DroneRopeCargoSimulator-object
	DroneRopeCargoSimulator simulator;

	// Initialize drone parameters
	double massDrone = 3;			// in [kg]
	double dragConstantDrone = 0.1;	// in [N s^2 / m^2]
	simulator.setConstantDroneParameters(massDrone, dragConstantDrone);

	// Initialize rope parameters
	double ropeInitialLength = 1.5;	// in [m]
	double ropeDamping = 50;		// in [N s / m]
	double ropeStiffness = 40000;	// in [N / m]
	simulator.setConstantRopeParameters(ropeInitialLength, ropeDamping, ropeStiffness);

	// Initialize cargo parameters
	double cargoMass = 2;			// in [kg]
	double dragConstantCargo = 0.1;	// in [N s^2 / m^2]
	simulator.setConstantCargoParameters(cargoMass, dragConstantCargo);

	// Set implementation
	simulator.setImplementation(true, true);  // (1st true) --> With cargo  
											  // (2nd true) --> RK4

	/* ---------------------------------- ACTIONS ---------------------------------- */

	// Specify a control vector for the object
	std::vector<double> controlVector = { 5, 2 };

	// Initialize next state vector to save to
	std::vector<double> stateVector{};

	// Step with simulator object and return its "next-state" state vector
	stateVector = simulator.simulationStep(controlVector);

	// [DEBUG] Check value in debugger
	std::vector<double> dummy = stateVector;

	return 0;
}