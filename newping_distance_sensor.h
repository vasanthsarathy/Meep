/**
 * @file newping_distance_sensor.h
 * @brief Driver for distance sensors supported by NewPing.h library
 * @author Vasanth Sarathy 
 */

 #include "distance_sensor.h"

 namespace Meep
 {
 	/*
 	 * @brief Constructor class for the Distance Sensor
 	 */
 	class DistanceSensor : public DistanceSensorDriver
 	{
 	public:
 		DistanceSensor(int triggerPin, int echoPin, unsigned int maxDistance)
 			: DistanceSensorDriver(maxDistance), sensor(triggerPin, echoPin, maxDistance)
 		{
 		}

	 	virtual	unsigned int getDistance()
 		{
 			int distance = sensor.ping_cm();
 			if (distance <= 0)
 				return maxDistance;
 			return distance;
 		}

 	private:
 		NewPing sensor;
 	};
 };