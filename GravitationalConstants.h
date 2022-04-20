//==============================================================
// Filename : GravitationalConstants.h
// Authors : Jesper Schrijver, Nick in het Veld
// Version : v1
// License : MIT License
// Description : Class that contains standard values for 
//				 gravitional constants; can be expanded for 
//				 multiple planets  - header
//==============================================================

// [BEGIN]: Prevent multiple inclusions of header
#ifndef GRAVITATIONALCONSTANTS_H
#define GRAVITATIONALCONSTANTS_H


// Libraries
#include <string>
#include <map>

// GravitationalConstants-class
class GravitationalConstants {
public:
	// Constructor (default)
	GravitationalConstants() = default;

	// Getters
	double getGravitationalConstant(std::string);

private:
	// Attributes
	std::map<std::string, double> m_gravitationalConstants{  // Given in [m/s2]; extend this list with {"KEY" : "value"} if new values are to be added
		{"Earth", 9.81}, 
		{"Venus", 8.87}, 
		{"Mars", 3.721}, };
};


// [END]: Prevent multiple inclusions of header
#endif