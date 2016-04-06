package com.ttocsneb.quadsim;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.math.Vector2;

public class QuadAI {

	public QuadAI() {
		
	}
	
	public Vector2 update(float gyroData, Vector2 accelData) {
		
		Gdx.app.debug("Data", "G " + gyroData + " X " + accelData.x + " Y " + accelData.y);
		
		return null;
	}
	
}
