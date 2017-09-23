#include "advancedcomplimentary.h"



	
	
    void advancedcomplimentary::update(float *accel_data, float *gyro_data, float gyro_dt){	

		
		/// convert the onfiltered accel data to degrees
		accel_X = atan(accel_data[0] / sqrt( accel_data[2] * accel_data[2]  + accel_data[1] * accel_data[1] ) ) * RAD_TO_DEG;// x axis to degrees
		accel_Y = atan(accel_data[1] / sqrt( accel_data[2] * accel_data[2]  + accel_data[0] * accel_data[0] ) ) * RAD_TO_DEG;// Y axis to degrees
		accel_Z = atan(accel_data[2] / sqrt( accel_data[0] * accel_data[0]  + accel_data[1] * accel_data[1] ) ) * RAD_TO_DEG;// Z axis to degrees
		
		///calculate angle for X axis
		gyroZ_invert = abs(Y_angle) / Y_angle;/// inverts Z_gyro when Y_angle < 0;
		gyroY_invert = abs(Z_angle) / Z_angle;/// inverts Y_gyro when accel_Z < 0;
		
		gyroY_rate = gyroY_invert * pow( cos( accel_Y * PI / 180.0f), 2) * gyro_data[1];// calculate gyro rotation 
		gyroZ_rate = gyroZ_invert * pow( cos( ( accel_Z * PI / 180.0f) ), 2) * gyro_data[2];//
		gyro_rate  = gyroY_rate - gyroZ_rate;
		
	    gain = 1.0f / ( ( gain_sensitivity[0] * pow( accel_X - X_angle, 2 ) + gain_displacement[0]) );/// calculate the the gain using the differnce between the previous calculated angle and the new accel_angle, the bigger the difference, the lower the gain
	    X_angle = (1.0f - gain ) * ( X_angle  - gyro_rate * gyro_dt  ) + ( gain ) * accel_X; /// complimentary filter with the adjusted gain for each loop
	  
	 
	    ///calculate angle for Y axis
		gyroZ_invert = abs(X_angle) / X_angle;/// inverts X_gyro when Z_angle < 0;
		gyroX_invert = abs(Z_angle) / Z_angle;/// inverts Y_gyro when accel_Z < 0;
		
		gyroX_rate = gyroX_invert * abs( cos( accel_X * PI / 180.0f)) * gyro_data[0];////
		gyroZ_rate = gyroZ_invert * pow( cos( ( accel_Z * PI / 180.0f) ), 2 ) * gyro_data[2];////
		gyro_rate = gyroX_rate - gyroZ_rate;
	  	  
		gain = 1.0f / ( ( gain_sensitivity[1] * pow( accel_Y - Y_angle, 2 ) + gain_displacement[1]) );/// calculate the the gain using the differnce between the previous calculated angle and the new accel_angle, the bigger the difference, the lower the gain
		Y_angle = (1.0f - gain ) * ( Y_angle + gyro_rate * gyro_dt ) + ( gain ) * accel_Y; /// complimentary filter with the adjusted gain for each loop
	  
	  
		///calculate angle for Z axis
		gyroY_invert = abs(X_angle) / X_angle;//
		gyroX_invert = abs(Y_angle) / Y_angle;// inverts Y_gyro when accel_Z < 0;
		
		gyroX_rate = gyroX_invert * abs( cos( accel_X * PI / 180.0f) ) * gyro_data[0];////
		gyroY_rate = gyroY_invert * abs( cos( accel_Y * PI / 180.0f) ) * gyro_data[1];////
		gyro_rate =  gyroY_rate - gyroX_rate; 
	  
		gain = 1.0f / ( ( gain_sensitivity[2] * pow( accel_Z - Z_angle, 2 ) + gain_displacement[2]) );/// calculate the the gain using the differnce between the previous calculated angle and the new accel_angle, the bigger the difference, the lower the gain
		Z_angle = (1.0f - gain ) * ( Z_angle + gyro_rate * gyro_dt ) + ( gain ) * accel_Z; /// complimentary filter with the adjusted gain for each loop
	    
		//// hedaing
		///heading -= gyro_dt * ( gyro_data[2] * sin( Z_angle * PI / 180.0f) + gyro_data[1] * sin( Y_angle * PI / 180.0f) + gyro_data[0] * sin( X_angle * PI / 180.0f) ) * 0.9999f;
		//gyro_heading += gyro_dt * gyro_data[2];
		

		
		
		
	}
	
	float advancedcomplimentary::get_heading(float *mag_data){
		//float y = abs(sin( Y_angle * PI / 2.0f ));
		//float x = abs(sin( X_angle * PI / 2.0f ));
		static unsigned long loopt;
		
		if( millis() - loopt > 10){
			loopt = millis();
		heading = atan2(  -mag_data[1] , mag_data[0] ) * RAD_TO_DEG;
		
		if( heading < 0 ){ heading += 360.0f;}
		
		}

		return heading;

	}
	
	
	float advancedcomplimentary::getX(){
		///get individual axis
		return X_angle;
	}
	
	float advancedcomplimentary::getY(){
		///get individual axis
		return Y_angle;
	}
	
	float advancedcomplimentary::getZ(){
		///get individual axis
		return Z_angle;
	}
	
	float advancedcomplimentary::getheading(){
		///get heading
		return heading;
	}
	
	void advancedcomplimentary::ini(float *accel_data){
		X_angle = atan(accel_data[0] / sqrt( accel_data[2] * accel_data[2]  + accel_data[1] * accel_data[1] ) ) * RAD_TO_DEG;// x axis
		Y_angle = atan(accel_data[1] / sqrt( accel_data[2] * accel_data[2]  + accel_data[0] * accel_data[0] ) ) * RAD_TO_DEG;// Y axis
		Z_angle = atan(accel_data[2] / sqrt( accel_data[0] * accel_data[0]  + accel_data[1] * accel_data[1] ) ) * RAD_TO_DEG;// Z axis
		
	}

