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
	robot.initialize();
}

void loop()
{
	robot.run();
}


