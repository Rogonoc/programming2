//==============================================================
// Filename : GravitationalConstants.cpp
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class that contains standard values for 
//				 gravitional constants; can be expanded for 
//				 multiple planets  - source
//==============================================================

// Libraries
#include "GravitationalConstants.h"


// Setters
double GravitationalConstants::getGravitationalConstant(std::string planetName) {
	// Initialize parameter for gravity value
	double gravitationalConstant;

	// Initialize iterator for map
	auto it = m_gravitationalConstants.find(planetName);
	
	// Find associated value with the user-specified planet
	if (it != m_gravitationalConstants.end()) { // Specific planet
		gravitationalConstant = it->second;
	}
	else { // Earth as default if planet not found
		gravitationalConstant = m_gravitationalConstants["Earth"];
	}

	// Return value
	return gravitationalConstant;
};