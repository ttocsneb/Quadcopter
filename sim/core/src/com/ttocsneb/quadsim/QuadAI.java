package com.ttocsneb.quadsim;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input.Keys;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;

public class QuadAI {

	final float P_Pitch = 2.2f;
	final float I_Pitch = .125f;
	final float D_Pitch = 6f;
	final int MAX_Pitch = 400;
	
	float error_tmp;
	float i_mem_pitch, pitch_setpoint, gyro_pitch_input, output_pitch, last_pitch_d_error;
	double gyro_pitch;
	
	public QuadAI() {
		i_mem_pitch = 0;
		last_pitch_d_error = 0;
	}
	
	float throttle;
	
	float motor1, motor2;
	
	/**
	 * Update the drone flight controller.
	 * @param gyroData angular velocity
	 * @param accelData deviation from free fall (relative to the craft.).
	 * @return (Left, Right) power outputs (0-1000)
	 */
	public Vector2 update(float gyroData, Vector2 accelData) {
		
		float pitch = (float)Math.atan(accelData.x/accelData.y/*NOTE: if there is a third axis like in real life, find the total value between y/z*/)*MathUtils.radiansToDegrees;
		//Don't forget to add the roll value in a 3d object.
		
		Gdx.app.debug("Data", "G " + gyroData + " P " + pitch + " X " + accelData.x + " Y " + accelData.y);
		
		
		
		gyro_pitch_input = gyro_pitch_input *0.8f + ((gyroData*MathUtils.radiansToDegrees)*.2f);

		//Set the setpoint (degrees per second) by the left and right keys with a max of 164 dps
		pitch_setpoint = Gdx.input.isKeyPressed(Keys.LEFT) ? 164 : (Gdx.input.isKeyPressed(Keys.RIGHT) ? -164 : 0);
		
		calculatePID();
		
		//Set the throttle
		throttle = MathUtils.clamp(throttle + (Gdx.input.isKeyPressed(Keys.UP) ? 5 : (Gdx.input.isKeyPressed(Keys.DOWN) ? -5 : 0)), 0, 800);
		
		//Set each motors speed.
		motor1 = throttle + output_pitch;
		motor2 = throttle - output_pitch;
		
		return new Vector2(motor1, motor2);
	}
	
	private void calculatePID() {

		//Find the error from the wanted speed vs actual speed
		error_tmp = gyro_pitch_input - pitch_setpoint;
		//find the integral error.
		i_mem_pitch += I_Pitch * error_tmp;
		//Constrain the integral.
		if(i_mem_pitch > MAX_Pitch)i_mem_pitch = MAX_Pitch;
		else if(i_mem_pitch < -MAX_Pitch)i_mem_pitch = -MAX_Pitch;
		
		//Find the PID value
		output_pitch = P_Pitch * error_tmp + i_mem_pitch + D_Pitch * (error_tmp - last_pitch_d_error);
		//Constrain it if necessary
		if(output_pitch > MAX_Pitch) output_pitch = MAX_Pitch;
		else if(output_pitch < -MAX_Pitch) output_pitch = -MAX_Pitch;
		
		//Set the last error for the Derivative
		last_pitch_d_error = error_tmp;
	}
	
}
