/** 
 * @file motor_driver.h
 * @brief Motor device driver definition for the Meep robot.
 * The idea is to abstract motor driver properties to work with any motor driver. 
 * See adafruit_motor_driver.h for the specific implementation of using this file. 
 * @author Vasanth Sarathy
 */

namespace Meep
{
	class MotorDriver
	{
	public: 

		/*
		 * @brief Controls the speed of the motors
		 * @param The speed of the motor. 
		 *  Valid values are between -255 and 255.
		 *  Positive values to run motor forward
		 *  Negative values to run motor backward
		 *  Zero to stop the motor
		 */
		virtual void setSpeed(int speed) = 0;

		/*
		 * @brief Obtains the current speed of the motors
		 * @return The current speed of the motor in the range of -255 to 255.
		 */
		virtual int getSpeed() const = 0;

		/*
		 * @brief Initializes the Motor Controller
		 * Initialization is needed for certain motorshields like Adafruit v2
		 */
		virtual void initializeDriver() = 0;
	};

};

/*
 * Notes: Virtual means that the methods can be overwritten by a subclass
 * The "=0" at the end of the method declarations and so in combination with
 * the virtual keyword imply that the methods do not have an implementation. 
 */
