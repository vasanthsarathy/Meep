/**
 * @file moving_averag.h 
 * @brief Helper class that maintains a moving average over a set of samples. 
 * Like a model for other classes to be constructed
 * The reason is because moving average can be calculated for many different types of samples.
 * @author Vasanth Sarathy
 */

// Note: N is the window size of the moving average filter. 
// V is template argument that represents the 'type' of the samples coming in (e.g., int, float, etc.)
 template <typename V, int N> class MovingAverage
 {
 public:
 	/*
 	 * @brief Class constructor
 	 * @param default value to initialize the average
 	 */
 	 MovingAverage(V def = 0) : sum(0), p(0)
 	 {
 	 	for (int i = 0; i < N; i++) {
			samples[i] = def;
			sum += samples[i];
		}
 	 }

 	 /* 
 	  * @brief Add new sample.
 	  * @param V is the type of input samples, and new_sample is the new sample to add to filter
 	  * @return the updated average
 	  */
 	 V add(V new_sample)
 	 {
 	 	sum = sum - samples[p] + new_sample;
 	 	samples[p++] = new_sample;
 	 	if (p >= N)
 	 		p = 0;
 	 	return sum/N;
 	 }



 private:
 	V samples[N];
 	V sum;
 	V p;
 };


 // Example usage to create an instance
 /*
	MovingAverage<unsigned int, 3> average;
 */


// Example usage in .ino files
/*

(1) #include "moving_average.h"

(2) declare MovingAverage object in private section of the main Robot class
	private:
		MovingAverage<unsigned int, 3> distanceAverage;

(3) Initialize it in the Robot constructor to not detect an obstacle at the start. 
	Robot() 
	:	bla, bla, bla, distanceAverage(MAX_DISTANCE)

(4) Change the run() function 
	int distance = distanceAverage.add(distanceSensor.getDistance());

The idea is to not simply compare raw distance measurement against "TOO_CLOSE", but
to compare "distanceAverage" with "TOO_CLOSE"

*/
