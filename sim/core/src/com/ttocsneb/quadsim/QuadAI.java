package com.ttocsneb.quadsim;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input.Keys;
import com.badlogic.gdx.math.Vector2;

public class QuadAI {

	public QuadAI() {
		
	}
	
	/**
	 * Update the drone flight controller.
	 * @param gyroData angular velocity
	 * @param accelData deviation from free fall (relative to the craft.).
	 * @return (Left, Right) power outputs (0-1000)
	 */
	public Vector2 update(float gyroData, Vector2 accelData) {
		
		Gdx.app.debug("Data", "G " + gyroData + " X " + accelData.x + " Y " + accelData.y);
		
		
		return new Vector2(Gdx.input.isKeyPressed(Keys.LEFT) ? 250 : 0, Gdx.input.isKeyPressed(Keys.RIGHT) ? 250 : 0);
	}
	
}
