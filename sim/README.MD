Drone Simulator
--

This simulates a done (bi-copter) in a 2 dimensional space.  It is set up to give an Ai object all the information sensors could give it, ie. an accelerometer and gyroscope.  While using a 2D world to simulate a 3D object is not ideal, it makes it easy to test ideas on how to control a drone, without worrying about crashing.  

I have used the simulator to figure out how to control the quads orientation rather than its angular velocity, which has been a huge help.  I plan on using it in the future for other features that I may work on later, such as GPS tracking, and other stuff..

Download
--

You can run the program online [here][online], or download the program here [here][download].

Controls
--
**Arrow keys** control the throttle/pitch controls.

**R** resets the map.

**Scroll** to zoom.

Images
--

![sim-sit](https://raw.githubusercontent.com/ttocsneb/Quadcopter/master/sim/web-assets/sim-sit.png)
![sim-hover](https://raw.githubusercontent.com/ttocsneb/Quadcopter/master/sim/web-assets/sim-hover.png)
![sim-fast](https://raw.githubusercontent.com/ttocsneb/Quadcopter/master/sim/web-assets/sim-fast.png)

[download]: https://github.com/ttocsneb/Quadcopter/raw/master/sim/desktop/build/sim.jar
[online]: http://ttocsneb.github.io/Quadcopter/
