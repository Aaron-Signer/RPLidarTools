#include "WheeledRobot.hpp"

class TheDankMeme: WheeledRobot {
  TheDankMeme() : WheeledRobot(
			       77, // wheelRadiusMillimeters
			       165) // halfAxleLengthMillimeters
  {
  }
  Velocities computeVelocities(long * odometry, Velocities & velocities) {
    // ouch, not sure yet
  }
};
      
