package com.ttocsneb.quadsim;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.Box2DDebugRenderer;
import com.gushikustudios.rube.RubeScene;
import com.gushikustudios.rube.loader.RubeSceneLoader;

public class Main extends ApplicationAdapter {
	RubeScene scene;

	Box2DDebugRenderer renderer;

	OrthographicCamera cam;

	Body quad;

	QuadAI ai;

	@Override
	public void create() {
		Gdx.app.setLogLevel(Application.LOG_DEBUG);
		
		RubeSceneLoader loader = new RubeSceneLoader();
		scene = loader.loadScene(Gdx.files.internal("world.json"));

		quad = scene.getNamed(Body.class, "Drone").first();

		renderer = new Box2DDebugRenderer();// new Box2DDebugRenderer(true,
											// false, false, true, true, true);
		cam = new OrthographicCamera(5, 5);

		ai = new QuadAI();
		
		lastVel = new Vector2();

	}

	Vector2 lastVel;

	@Override
	public void render() {
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

		Vector2 vel = new Vector2(quad.getLinearVelocity());
		vel.sub(lastVel);
		vel.x *= scene.stepsPerSecond;
		vel.y *= scene.stepsPerSecond;
		vel.y -= scene.getWorld().getGravity().y;

		float totacc = (float) Math.sqrt(vel.x * vel.x + vel.y * vel.y);

		vel.x = Math.min(8, Math.max(-8, (MathUtils.sin(quad.getAngle()) * totacc)/-scene.getWorld().getGravity().y));
		vel.y = Math.min(8, Math.max(-8, (MathUtils.cos(quad.getAngle()) * totacc)/-scene.getWorld().getGravity().y));

		ai.update(quad.getAngularVelocity(), vel);
		lastVel = new Vector2(quad.getLinearVelocity());

		scene.step();

		renderer.render(scene.getWorld(), cam.combined);

	}
}
