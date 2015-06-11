/**
 * @file adafruit_motor_driver.h
 * @brief Motor device driver for the Adafruit Motor Shield v2.0
 * Builds off of the motor_driver.h file
 * (1) Creates constructor for Motor class 
 * (2) Implements the Adafruit-specific setSpeed() method 
 * (3) Implements the Adafruit-specific getSpeed() method
 * Note: To stack motorshields, will need to write a separate driver.
 * @author Vasanth Sarathy
 */

// Including this to use the MotorDriver class
#include "motor_driver.h"

namespace Meep
{
	// Define Motor class inherits from MotorDriver
	class Motor : public MotorDriver
	{
	public:
		/*
	 	 * @brief Constructor method for the Motor class
	 	 * @param Number for DC motor port (from 1 to 4)
	 	 * To construct each motor, a motorshield object is to be created
	 	 */
		Motor(int number)
			: MotorDriver(), motor(), currentSpeed(0)
		{
			AFMS = Adafruit_MotorShield();
			motor = AFMS.getMotor(number);
		}

		void setSpeed(int speed)
		{
			currentSpeed = speed;
			if (speed >= 0) {
				motor->setSpeed(speed);
				motor->run(FORWARD);
			}
			else {
				motor->setSpeed(-speed);
				motor->run(BACKWARD);
			}
		}

		int getSpeed() const
		{
			return currentSpeed;
		}

		void initializeDriver()
		{
			// Adafruit MotorShield v2 requires initialization.
			// Consider moving this outside Motor class. 
			AFMS.begin();
		}

	private:
		Adafruit_MotorShield AFMS;
		Adafruit_DCMotor *motor;
		int currentSpeed;

	};
};



