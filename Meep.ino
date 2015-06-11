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
#define TOO_CLOSE 25
#define MAX_DISTANCE (TOO_CLOSE * 20)
#define RUN_TIME 30

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


//-------------------------------------------
// MOVING AVERAGE FILTER
#include "moving_average.h"
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
			distanceSensor(DISTANCE_SENSOR_INIT), distanceAverage(MAX_DISTANCE)
		{
		}

 //---------------------------------------------------------------------------------------
		/*
		 * @brief Initialize Robot state
		 */
		 void initialize()
		 {
		 	// Initialize Motor Shield
		 	lbWheel.initializeDriver();

		 	//Calculating time
		 	endTime = millis() + RUN_TIME * 1000;
		 	move();

		 }

 //---------------------------------------------------------------------------------------
		 /*
		  * @brief update the state of the robot based on inputs from RasPi, sensors etc.
		  * Must be called repeatedly while the robot is in operation
		  */
		 void run()
		 {	 	


		 	if (stopped())
		 		return;

		 	// More efficient to get these variables at the start of run()
		 	// and then later call it in the functions.
		 	unsigned long currentTime = millis();
		 	int distance = distanceAverage.add(distanceSensor.getDistance());
		 	log("state: %d, currentTime: %ul, distance: %u\n", state, currentTime, distance);


		 	if (doneRunning(currentTime))
		 		stop();
		 	else if (moving()) {
		 		if (obstacleAhead(distance))
		 			turn(currentTime);
		 	}
		 	else if (turning()) {
		 		if (doneTurning(currentTime, distance))
		 			move();
		 	}
		 }

//---------------------------------------------------------------------------------------
	protected: 
	
	/*
	 * @brief stopped() checks if robot's current state is stateStopped
	 * @return TRUE if state = stateStopped, FALSE otherwise 
	 */	 
	bool stopped()
	{
		return (state == stateStopped);
	}

	/*
	 * @brief doneRunning() checks if robot was running for a predetermined amount of time
	 * @param time limit (e.g., 30 seconds)
	 * @return TRUE if time limit has expired, and FALSE otherwise
	 */
	bool doneRunning(unsigned long currentTime)
	{
		return (currentTime >= endTime);
	}

	/*
	 * @brief stop() changes state to stateStopped
	 * 
	 */
	void stop()
	{
		lbWheel.setSpeed(0);
      	rbWheel.setSpeed(0);
     	lfWheel.setSpeed(0);
     	rfWheel.setSpeed(0);
		state = stateStopped;
	}


	/*
	 * @brief moving() checks if the robot's current state is stateMoving
	 * @return TRUE if state = stateMoving, FALSE otherwise
	 */
	bool moving()
 	{
 		return (state == stateMoving);
 	}

	/*
	 * @brief obstacleAhead() checks if an obstacle is in front
	 * @param distanceAverage 
	 * @return TRUE if distanceAverage < TOO_CLOSE, FALSE otherwise
	 */
	bool obstacleAhead(unsigned int distance)
	{
		return (distance <= TOO_CLOSE);
	}

	/*
	 * @brief turn() initiates turn and changes state to stateTurning
	 */
	 void turn(unsigned long currentTime)
	 {
	 	if (random(2) == 0) 
	 	{
	 		//Turn Left
	 		lbWheel.setSpeed(-255);
     		lfWheel.setSpeed(-255);

     		rbWheel.setSpeed(255);
     		rfWheel.setSpeed(255);
	 	}
	 	else
	 	{
	 		//Turn right
	 		lbWheel.setSpeed(255);
     		lfWheel.setSpeed(255);

          	rbWheel.setSpeed(-255);
     		rfWheel.setSpeed(-255);
	 	}
	 	state = stateTurning;
	 	endStateTime = currentTime + random(500,1000);
	 }

	/*
	 * @brief turning() checks if current state is stateTurning
	 * @return TRUE if state = stateTurning, FALSE otherwise.
	 */
	bool turning()
	{
		return (state == stateTurning);
	}

	/*
	 * @brief doneTurning() checks if turning process complete
	 * @return TRUE if maneuver is complete and FALSE if not.
	 */
	bool doneTurning(unsigned long currentTime, unsigned int distance)
	 {
	 	if (currentTime >= endStateTime)
	 		return (distance > TOO_CLOSE);
	 	return false;
	 }


	/*
	 * @brief move() changes the robot state to stateMoving
	 */
	void move()
	{
		lbWheel.setSpeed(255);
      	rbWheel.setSpeed(255);
     	lfWheel.setSpeed(255);
     	rfWheel.setSpeed(255);
		state = stateMoving;
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
        enum state_t { stateStopped, stateMoving, stateTurning };
        state_t state;

        /* 
         * Variable to count.
         * Time units returned by the function millis()
         */
        unsigned long startTime;

        //Declare MovingAverage object. Note the window size of 3
        MovingAverage<unsigned int, 3> distanceAverage;

        //Variable to track time. See constant RUN_TIME at the top 
        unsigned long endTime;
        unsigned long endStateTime;

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


