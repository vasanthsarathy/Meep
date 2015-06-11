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
		Robot()
			: lbWheel(LBWHEEL_INIT), rbWheel(RBWHEEL_INIT), 
			lfWheel(LFWHEEL_INIT), rfWheel(RFWHEEL_INIT)
		{
		}

		 void initialize()
		 {
		 	lbWheel.initializeDriver();
		 	lbWheel.setSpeed(255);
            rbWheel.setSpeed(-255);
            lfWheel.setSpeed(-255);
            rfWheel.setSpeed(255);
		 }

		 void run()
		 {	 	
		 }

	private:
		Motor lbWheel;
        Motor rbWheel;
        Motor lfWheel;
        Motor rfWheel;
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


