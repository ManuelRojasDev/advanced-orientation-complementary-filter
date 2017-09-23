#ifndef _advancedcomplimentary_H_
#define _advancedcomplimentary_H_
#endif

#include <Arduino.h>

class advancedcomplimentary{
	/// ofset is set by 
	public:
	
	advancedcomplimentary(float gain_sensitivity_, float max_gain_){
		gain_max[0] = max_gain_;
		gain_max[1] = max_gain_;
		gain_max[2] = max_gain_;
		gain_sensitivity[0] = gain_sensitivity_;
		gain_sensitivity[1]= gain_sensitivity_;
		gain_sensitivity[2] = gain_sensitivity_;
	}
	
	advancedcomplimentary(){
		
		
	}
	
    void update(float *accel_data, float *gyro_data, float gyro_dt);
	float getX();
	float getY();
	float getZ();
	float getheading();
	void ini(float *accel_data);
	float get_heading(float *mag_data);
	
	
	
	
	private:
	


 float gyro_rate;// degrees per second from gyro fusion
 float gain;// complimanary filter gain
 float gain_max[3] = {0.5f, 0.5f, 0.5f};// max vaule that gain can get to
 float gain_sensitivity[3] = {10.0f, 10.0f, 100.0f};// sensitivity of the difference between  the filtered and the unfiltered new angle
 float gain_displacement[3] = {( 1.0f / gain_max[0] ), ( 1.0f / gain_max[1] ), ( 1.0f / gain_max[2] )};/// used to move the value of the 0 position in the graph
 float X_angle = 1, Y_angle = 1 , Z_angle = 1;// storage for the filtered angle,  1 to avoid 0/0 at startup
 float gyroX_invert, gyroY_invert, gyroZ_invert;
 float gyroX_rate, gyroY_rate, gyroZ_rate;
 float accel_X;// x axis
 float accel_Y;// Y axis
 float accel_Z;// Z axis
 float heading, gyro_heading;
		



};