// Meep
// Robot Vehicle firmware for the Arduino platform
//
// Copyright (c) 2015 by Vasanth Sarathy
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
// AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/**
 * @file Meep.ino
 * @brief Arduino robot firmware
 * @author Vasanth Sarathy
 */

//-------------------------------------------
// DEVICE DRIVERS
/// Enable one driver in each category

/// Motor Controllers
#define ENABLE_ADAFRUIT_MOTOR_DRIVER
//#define ....
//#define ....

/// Distance Sensor
#define ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
//#define ....
//#define ....

/// Constants
#define TOO_CLOSE 10
#define MAX_DISTANCE (TOO_CLOSE * 20)

#ifdef ENABLE_ADAFRUIT_MOTOR_DRIVER
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "adafruit_motor_driver.h"
#define LBWHEEL_INIT 1
#define RBWHEEL_INIT 2
#define RFWHEEL_INIT 3
#define LFWHEEL_INIT 4
#endif

#ifdef ENABLE_NEWPING_DISTANCE_SENSOR_DRIVER
#include <NewPing.h>
#include "newping_distance_sensor.h"
#define DISTANCE_SENSOR_INIT 14,15,MAX_DISTANCE
#endif
//---------------------------------------------

//-------------------------------------------
// DEBUGGING AND ERROR LOGGING
#define LOGGING
#include "logging.h"
//-------------------------------------------


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
			lfWheel(LFWHEEL_INIT), rfWheel(RFWHEEL_INIT),
			distanceSensor(DISTANCE_SENSOR_INIT)
		{
		}

		/*
		 * @brief Initialize Robot state
		 */
		 void initialize()
		 {
		 	// Initialize Motor Shield
		 	lbWheel.initializeDriver();

		 	// Start running the motors  
	 		lbWheel.setSpeed(255);
          	rbWheel.setSpeed(255);
         	lfWheel.setSpeed(255);
         	rfWheel.setSpeed(255);

		 	// Initialize the robot state .
		 	state = stateRunning;
		 }

		 /*
		  * @brief update the state of the robot based on inputs from RasPi, sensors etc.
		  * Must be called repeatedly while the robot is in operation
		  */
		 void run()
		 {	 	
		 	if (state == stateRunning) {
		 		
		 		//Log distance for debugging purposes
		 		unsigned int distance = distanceSensor.getDistance();
		 		log("distance: %u\n", distance);
		 		
		 		if (distanceSensor.getDistance() <= TOO_CLOSE) {
		 			state = stateStopped;
		 			lbWheel.setSpeed(0);
		          	rbWheel.setSpeed(0);
		         	lfWheel.setSpeed(0);
		         	rfWheel.setSpeed(0);
		 		}
		 	}

		 }

	private:

		//Set the motor variables of class Motor
		Motor lbWheel;
        Motor rbWheel;
        Motor lfWheel;
        Motor rfWheel;

        // Set the variable of the distance sensor 
        DistanceSensor distanceSensor;

        //Enum creates a user defined enumerated type with two possible values
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
	Serial.begin(9600);
	robot.initialize();
}

void loop()
{
	robot.run();
}


