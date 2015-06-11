/**
 * @file distance_sensor.h
 * @brief Distance sensor driver 
 * Idea is to use this with any kind of distance sensor
 * Generic interface for Ultrasonic, IR, etc.
 * @author Vasanth Sarathy
 */

 namespace Meep
 {
 	class DistanceSensorDriver
 	{
 	public:
 		/*
 		 * @brief Constructor class for distance sensor
 		 * @param distance The maximum distance in centimeters that needs to be tracked
 		 */
 		DistanceSensorDriver(unsigned int distance) 
 			: maxDistance(distance)
 		{
 		}

 		/*
 		 * @brief Method for returning the distance to the nearest object in centimeters
 		 * @return the distance to the closest object in centimeters
 		 * or maxDistance if no object was detected
 		 */
 		virtual unsigned int getDistance() = 0;

 	// Variables private to the outside but accessible to subclasses
 	protected:
 		unsigned int maxDistance;
 	};

 };