/**
 * @file Meep.ino
 * @brief Arduino robot firmware
 * @author Vasanth Sarathy
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "adafruit_motor_driver.h"
#define LBWHEEL_INIT 1
#define RBWHEEL_INIT 2
#define RFWHEEL_INIT 3
#define LFWHEEL_INIT 4

namespace Meep
{
	class Robot
	{
	public:
		/*
		 * @brief Class constructor for Robot
		 */
		Robot()
			: lbWheel(LBWHEEL_INIT), rbWheel(RBWHEEL_INIT), 
			lfWheel(LFWHEEL_INIT), rfWheel(RFWHEEL_INIT)
		{
		}

		/*
		 * @brief Initialize Robot state
		 */
		 void initialize()
		 {
		 	// Initialize Motor Shield
		 	lbWheel.initializeDriver();

		 	// TEST CODE to test motors 
		 	/* 
		 		lbWheel.setSpeed(255);
              	rbWheel.setSpeed(-255);
             	lfWheel.setSpeed(-255);
             	rfWheel.setSpeed(255);
			*/

		 	// Initialize the robot state and record the current time.
		 	state = stateStopped;
		 	startTime = millis();
		 }

		 /*
		  * @brief update the state of the robot based on inputs from RasPi, sensors etc.
		  * Must be called repeatedly while the robot is in operation
		  */
		 void run()
		 {	 	
		 	unsigned long currentTime = millis();
		 	unsigned long elapsedTime = currentTime - startTime;
		 	switch (state){
		 	case stateStopped:
		 		if (elapsedTime >= 5000) {
		 			lbWheel.setSpeed(255);
		 			rbWheel.setSpeed(255);
		 			lfWheel.setSpeed(255);
		 			rfWheel.setSpeed(255);
		 			state = stateRunning;
		 			startTime = currentTime;
		 		}
		 		break;
		 	case stateRunning:
		 		if (elapsedTime >= 8000) {
		 			lbWheel.setSpeed(0);
		 			rbWheel.setSpeed(0);
		 			lfWheel.setSpeed(0);
		 			rfWheel.setSpeed(0);
		 			state = stateStopped;
		 			startTime = currentTime;
		 		}
		 		break;
		 	}
		 }

	private:
		/*
		 * Set the motor variables of class Motor
		 */
		Motor lbWheel;
        Motor rbWheel;
        Motor lfWheel;
        Motor rfWheel;

        /*
         * Enum creates a user defined enumerated type with two possible values
         */
        enum state_t { stateStopped, stateRunning };
        state_t state;

        /* 
         * Variable to count.
         * Time units returned by the function millis()
         */
        unsigned long startTime;

	};
};

Meep::Robot robot;

void setup()
{
	robot.initialize();
}

void loop()
{
	robot.run();
}


