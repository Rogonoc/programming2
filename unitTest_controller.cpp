// Libraries
#include "DroneControllerControlVector.h"

// Main function
int main()
{
	/* ----------------------------- OBJECT ----------------------------- */

	// Initialize object
	DroneControllerControlVector controller;

	// Set dynamics type
	bool dynamicsType = false;

	// Set time constants
	double timeConstantX = 0.2;		// in [s]
	double timeConstantY = 0.2;		// in [s]
	double timeConstantTheta = 0.1;	// in [s]

	// Set controller damping
	double oscillationDampingConstant = 50; // in [N / m]

	// Set gravitational force
	double gravitationalConstant = 9.81;	// in [m / s^2]

	// Set drone parameters
	double massDrone = 3;		// in [kg]
	double xDrone = 1;			// in [m]
	double xDotDrone = 0.2;		// in [m / s]
	double thetaDrone = 0.523;	// in [rad] --> given value equal to +30 degrees
	double yDrone = 1;			// in [m]
	double yDotDrone = 0.2;		// in [m / s]

	// Set cargo parameters
	double massCargo = 2;	// in [kg]
	double xCargo = 0;		// in [m]
	double yCargo = 0;		// in [m]

	/* ----------------------------- ACTIONS ----------------------------- */

	// Set time constants
	controller.setTimeConstants(timeConstantX, timeConstantY, timeConstantTheta);

	// Give controller (hypothetical) required reference velocities
	std::vector<double> velocityVector = { 4, 4 };

	// Initialize control vector to save to
	std::vector<double> referenceControlVector{};

	// Calculate reference control vector
	referenceControlVector = controller.calculateReferenceControlVector(dynamicsType, gravitationalConstant,
																		massDrone, xDrone, xDotDrone, yDrone, yDotDrone, thetaDrone,
																		massCargo, xCargo, yCargo);

	// [DEBUG] Check value in debugger
	std::vector<double> dummy = referenceControlVector;

	// Exit program
	return 0;
}