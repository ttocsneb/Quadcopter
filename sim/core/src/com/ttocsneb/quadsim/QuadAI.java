package com.ttocsneb.quadsim;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input.Keys;
import com.badlogic.gdx.graphics.g2d.GlyphLayout;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;

public class QuadAI {

	final float P_Pitch = 0.3f;//2.2f;
	final float I_Pitch = 0.0f;//.125f;
	final float D_Pitch = 10.5f;//6f;
	final int MAX_Pitch = 400;

	float error_tmp;
	float i_mem_pitch, pitch_setpoint, output_pitch,
			last_pitch_d_error;
	float accel_pitch, accel_pitch_input;

	GlyphLayout gyro;
	GlyphLayout pitch;
	GlyphLayout accel;

	public QuadAI() {
		gyro = new GlyphLayout(Main.font, "Gyro: ");
		pitch = new GlyphLayout(Main.font, "Pitch: ");
		accel = new GlyphLayout(Main.font, "Accel: ");
		Main.addText(gyro, pitch, accel);
	}

	public void reset() {
		i_mem_pitch = 0;
		last_pitch_d_error = 0;
		throttle = 0;

	}       

	float throttle;

	float motor1, motor2;
	
	float gyroAngle;

	/**
	 * Update the drone flight controller.
	 * 
	 * @param gyroData
	 *            angular velocity
	 * @param accelData
	 *            deviation from free fall (relative to the craft.).
	 * @return (Left, Right) power outputs (0-1000)
	 */
	public Vector2 update(float gyroData, Vector2 accelData) {
		
		//Use an integral to calculate the angle from the gyroscope.
		gyroAngle += gyroData;

		//Find the pitch of the quad from the accelerometer.
		float accelPitch = MathUtils.atan2(accelData.x, accelData.y)*MathUtils.radiansToDegrees;
		
		//Note: Since the simulation is so accurate, we do not need to fuse the two sensors.
		//Use a complementary filter to fuse both the accelerometer pitch, and the gyro pitch.
		//accel_pitch_input = 0.98f*gyroAngle + 0.02f*accelPitch;
		accel_pitch_input = accelPitch;
		
		
		//Set the wanted orientation, the max is 45 deg in both directions.
		pitch_setpoint = Gdx.input.isKeyPressed(Keys.LEFT) ? 45 : Gdx.input.isKeyPressed(Keys.RIGHT) ? -45 : 0;
		
		//Run the PID calculations.
		calculatePID();

		// Set the throttle
		throttle = MathUtils.clamp(
				throttle
						+ (Gdx.input.isKeyPressed(Keys.UP) ? 5 : (Gdx.input
								.isKeyPressed(Keys.DOWN) ? -5 : 0)), 0, 800);

		// Set each motors speed from the PID data.
		motor1 = (float) (throttle + output_pitch);
		motor2 = (float) (throttle - output_pitch);
		

		//Write values to the debug menu.
		accel.setText(Main.font, "Accel: " + Math.round(accelData.x*10)/10f + "," + Math.round(accelData.y*10)/10f);
		pitch.setText(Main.font, "Pitch: " + Math.round(accel_pitch_input*10)/10f);
		gyro.setText(Main.font, "Gyro: " + Math.round(gyroData*10)/10f);

		return new Vector2(motor1, motor2);
	}

	private void calculatePID() {
		//Find the error from the setpoint to the input
		error_tmp = (accel_pitch_input - pitch_setpoint);

		//Find the integral of the error.
		i_mem_pitch += I_Pitch * error_tmp;

		//Constrain the integral to the Max Pitch value
		if (i_mem_pitch > MAX_Pitch)
			i_mem_pitch = MAX_Pitch;
		else if (i_mem_pitch < -MAX_Pitch)
			i_mem_pitch = -MAX_Pitch;

		// Calculate the PID 
		output_pitch = P_Pitch * error_tmp + i_mem_pitch + D_Pitch
				* (error_tmp - last_pitch_d_error);
		// Constrain it to the Max Pitch value
		if (output_pitch > MAX_Pitch)
			output_pitch = MAX_Pitch;
		else if (output_pitch < -MAX_Pitch)
			output_pitch = -MAX_Pitch;

		//Save the error for derivative.
		last_pitch_d_error = error_tmp;
	}

}
