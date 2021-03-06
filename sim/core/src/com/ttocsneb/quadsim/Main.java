package com.ttocsneb.quadsim;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input.Keys;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Texture.TextureFilter;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.GlyphLayout;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer.ShapeType;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.Box2DDebugRenderer;
import com.badlogic.gdx.utils.Array;
import com.gushikustudios.rube.RubeScene;
import com.gushikustudios.rube.loader.RubeSceneLoader;

public class Main extends ApplicationAdapter implements InputProcessor {
	private static final float MaxForce = 50;
	private static final int ppm = 50;
	private RubeScene scene;

	private Box2DDebugRenderer renderer;
	private ShapeRenderer shape;
	private OrthographicCamera cam;

	private Body quad;

	private QuadAI ai;

	private OrthographicCamera batchCam;
	public static BitmapFont font;
	private SpriteBatch batch;
	private static Array<GlyphLayout> text;

	private GlyphLayout position;
	private GlyphLayout orientation;
	private GlyphLayout velocity;
	
	private int windDir = 45;//TODO Add control to the wind speed/direction.
	private float windForce = 0f;

	/**
	 * Add a line to the display.
	 * 
	 * @param layout
	 */
	public static void addText(GlyphLayout... layout) {
		for (GlyphLayout l : layout) {
			text.add(l);
		}
	}

	@Override
	public void create() {
		Gdx.app.setLogLevel(Application.LOG_DEBUG);

		font = new BitmapFont(Gdx.files.internal("isocteur.fnt"));
		font.getRegion().getTexture()
				.setFilter(TextureFilter.Linear, TextureFilter.Linear);
		batch = new SpriteBatch();
		text = new Array<GlyphLayout>();

		position = new GlyphLayout(font, "Positon: ");
		velocity = new GlyphLayout(font, "Velocity: ");
		orientation = new GlyphLayout(font, "Orientation:");
		addText(position, velocity, orientation, new GlyphLayout(font,
				"--------------"));

		shape = new ShapeRenderer();
		renderer = new Box2DDebugRenderer();
		renderer.setDrawJoints(false);
		cam = new OrthographicCamera(Gdx.graphics.getWidth() / ppm,
				Gdx.graphics.getHeight() / ppm);
		batchCam = new OrthographicCamera();
		batchCam.setToOrtho(false, 1920, 1080);
		// cam.zoom = 10f;

		lastVel = new Vector2();

		Gdx.input.setInputProcessor(this);

		ai = new QuadAI();
		init();

	}

	private void init() {
		if (scene != null)
			scene.getWorld().dispose();
		RubeSceneLoader loader = new RubeSceneLoader();
		scene = loader.loadScene(Gdx.files.internal("world.json"));

		quad = scene.getNamed(Body.class, "Drone").first();
		ai.reset();

	}

	Vector2 lastVel;

	@Override
	public void render() {
		// Clear the screen to black
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

		for(Body b : scene.getBodies()) {
			b.applyForceToCenter(windForce * MathUtils.cosDeg(windDir), windForce * MathUtils.sinDeg(windDir), false);
		}
		
		// move the camera to the quad.
		cam.position.set(quad.getPosition().x, quad.getPosition().y, 0);
		cam.update();

		// Convert linear acceleration to an output similar to that of
		// an accelerometer.
		// First convert the velocity to acceleration.
		Vector2 acc = new Vector2(quad.getLinearVelocity());
		acc.sub(lastVel);
		acc.x *= scene.stepsPerSecond;
		acc.y *= scene.stepsPerSecond;
		// add gravity to the y axis
		acc.y -= scene.getWorld().getGravity().y;
		// Get the total acceleration in both axis.
		float totacc = (float) Math.sqrt(acc.x * acc.x + acc.y * acc.y);
		// calculate the acceleration in the plane of the quad.
		acc.x = Math.min(
				8,
				Math.max(-8, (MathUtils.sin(quad.getAngle()) * totacc)
						/ -scene.getWorld().getGravity().y));
		acc.y = Math.min(
				8,
				Math.max(-8, (MathUtils.cos(quad.getAngle()) * totacc)
						/ -scene.getWorld().getGravity().y));

		// Set the last velocity
		lastVel = new Vector2(quad.getLinearVelocity());

		// Update the ai with found acceleration, and angular velocity.
		Vector2 forces = ai.update(quad.getAngularVelocity(), acc);
		// Convert the forces value into actual forces using the MaxForce Final.
		forces.x *= MaxForce / 2000f;
		forces.y *= MaxForce / 2000f;
		forces.x = MathUtils.clamp(forces.x, 0, MaxForce / 2f);
		forces.y = MathUtils.clamp(forces.y, 0, MaxForce / 2f);

		// Set the position of the forces to variables (as it will be used
		// multiple times).
		Vector2 posleft = new Vector2(quad.getPosition()).add(
				0.1724f * MathUtils.cos(quad.getAngle() + 2.84489f),
				0.1724f * MathUtils.sin(quad.getAngle() + 2.84489f));
		Vector2 posright = new Vector2(quad.getPosition()).add(
				0.1724f * MathUtils.cos(quad.getAngle() + 0.296706f),
				0.1724f * MathUtils.sin(quad.getAngle() + 0.296706f));

		// apply the left/right forces to the corners of the quad.
		quad.applyForce(
				new Vector2(forces.x
						* MathUtils.cos(0.5f * MathUtils.PI + quad.getAngle()),
						forces.x
								* MathUtils.sin(0.5f * MathUtils.PI
										+ quad.getAngle())), posleft, true);
		quad.applyForce(
				new Vector2(forces.y
						* MathUtils.cos(0.5f * MathUtils.PI + quad.getAngle()),
						forces.y
								* MathUtils.sin(0.5f * MathUtils.PI
										+ quad.getAngle())), posright, true);

		// Render the world
		renderer.render(scene.getWorld(), cam.combined);

		// Draw the forces(the world renderer will not display individual forces
		// :( ).
		cam.update();
		shape.setProjectionMatrix(cam.combined);
		shape.begin(ShapeType.Line);
		shape.setColor(Color.YELLOW);
		// shape.circle(posleft.x, posleft.y, 0.01f, 10);
		// shape.circle(posright.x, posright.y, 0.01f, 10);

		shape.setColor(1, 1 - forces.x / MaxForce, 0, 1);
		shape.line(
				posleft,
				new Vector2(posleft.x + forces.x
						* MathUtils.cos(1.5f * MathUtils.PI + quad.getAngle())
						/ ppm, posleft.y + forces.x
						* MathUtils.sin(1.5f * MathUtils.PI + quad.getAngle())
						/ ppm));

		shape.setColor(1, 1 - forces.y / MaxForce, 0, 1);
		shape.line(
				posright,
				new Vector2(posright.x + forces.y
						* MathUtils.cos(1.5f * MathUtils.PI + quad.getAngle())
						/ ppm, posright.y + forces.y
						* MathUtils.sin(1.5f * MathUtils.PI + quad.getAngle())
						/ ppm));

		shape.end();

		orientation.setText(
				font,
				"Orientation: "
						+ MathUtils.round(quad.getAngle()
								* MathUtils.radiansToDegrees) + " deg");
		velocity.setText(
				font,
				"Velocity: "
						+ MathUtils.round(quad.getLinearVelocity().x * 100)
						/ 100f + ", "
						+ MathUtils.round(quad.getLinearVelocity().y * 100)
						/ 100f);
		position.setText(
				font,
				"Position: " + MathUtils.round(quad.getPosition().x * 100)
						/ 100f + ", "
						+ MathUtils.round(quad.getPosition().y * 100) / 100f);

		batchCam.update();
		batch.setProjectionMatrix(batchCam.combined);
		batch.begin();

		int position = 1070;
		for (GlyphLayout l : text) {
			font.draw(batch, l, 10, position);
			position -= l.height + 10;
		}

		batch.end();

		scene.step();

	}

	@Override
	public void resize(int width, int height) {
		cam.setToOrtho(false, width / ppm, height / ppm);
		batchCam.setToOrtho(false, (width / (float) height) * 1080, 1080);
	}

	@Override
	public void dispose() {
		shape.dispose();
		scene.getWorld().dispose();
		renderer.dispose();
		batch.dispose();
		font.dispose();
	}

	// ////////////////////////////////////////////////////////
	//
	// Input Processor
	//
	// ////////////////////////////////////////////////////////

	@Override
	public boolean keyDown(int keycode) {
		return false;
	}

	@Override
	public boolean keyUp(int keycode) {
		if (keycode == Keys.R) {
			init();
		}
		return false;
	}

	@Override
	public boolean keyTyped(char character) {
		return false;
	}

	@Override
	public boolean touchDown(int screenX, int screenY, int pointer, int button) {
		return false;
	}

	@Override
	public boolean touchUp(int screenX, int screenY, int pointer, int button) {
		return false;
	}

	@Override
	public boolean touchDragged(int screenX, int screenY, int pointer) {
		return false;
	}

	@Override
	public boolean mouseMoved(int screenX, int screenY) {
		return false;
	}

	@Override
	public boolean scrolled(int amount) {
		cam.zoom = MathUtils.clamp(cam.zoom + amount * 0.1f, 0.1f, 10);

		return false;
	}
}
