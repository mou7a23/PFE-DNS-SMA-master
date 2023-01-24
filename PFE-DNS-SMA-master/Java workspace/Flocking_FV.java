package tests;

import java.util.ArrayList;
import java.util.List;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;

public class Flocking_FV extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;

	private double detection_threshold=80;
	private double speed;
	
	private double attraction_coef=2;
	private double repulsion_coef=0.5;

	public Flocking_FV() {
		timeStep = 64;  // set the control time step


		// Sensors initialization 
		// IR distance sensors
		distanceSensor = new DistanceSensor[8];
		String[] sensorNames = {
				"ps0", "ps1", "ps2", "ps3",
				"ps4", "ps5", "ps6", "ps7"
		};

		for (int i = 0; i < 8; i++) {
			distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
			distanceSensor[i].enable(timeStep);
		}

		// Camera
		camera=this.getCamera("camera");
		camera.enable(timeStep);
		camera.recognitionEnable(timeStep);

		//  WiFi communication
		emitter=getEmitter("emitter");
		receiver=getReceiver("receiver");
		receiver.enable(timeStep);


		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);

		// LEDS
		leds = new LED[10];
		String[] ledsNames = {
				"led0", "led1", "led2", "led3",
				"led4", "led5", "led6", "led7",
				"led8", "led9"
		};
		for (int i = 0; i < 10; i++) {
			leds[i] = this.getLED(ledsNames[i]);
		}
	}

	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++)
			psValues[i] = distanceSensor[i].getValue();

		return psValues;
	}

	/**
	 * 
	 * @param left : a value between [-100;100]%
	 * @param right : a value between [-100;100]%
	 */
	protected void move(double left, double right) {
		double max=6.2;
		getMotor("left wheel motor").setVelocity(left * max / 100);
		getMotor("right wheel motor").setVelocity(right * max / 100);
	}

	/**
	 * Switch on / off a LED according to its num ([0;9])
	 * @param num
	 * @param on : true if the LED is to be switched on, 
	 * or false if the LED is to be switched off
	 */
	protected void setLED(int num, boolean on) {
		if(num < 10) {
			leds[num].set(on ? 1 : 0);
		}
	}

	/**
	 * 
	 * @return an empty list if nothing is detected by the camera, 
	 * a list of CameraRecognitionObject otherwise (see https://cyberbotics.com/doc/reference/camera#camera-recognition-object)
	 */
	protected List<CameraRecognitionObject> cameraDetection() {
		ArrayList<CameraRecognitionObject> detected=new ArrayList<>();
		int nb=camera.getRecognitionNumberOfObjects();
		if(nb >0) {
			CameraRecognitionObject[] objects=camera.getRecognitionObjects();
			for(int i=0;i<objects.length;i++) {
				detected.add(objects[i]);
			}
		}
		return detected;
	}

	/**
	 * Look in a List of camera detected objects if other robots are recognized 
	 * @param detected: a List of camera detected objects
	 * @return a List of CameraRecognitionObject representing the other robots
	 */
	protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
		ArrayList<CameraRecognitionObject> robots=new ArrayList<>();
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("e-puck") == 0)
				robots.add(ob);
		}
		return robots;		
	}

	/**
	 * 
	 * @param robot another robot detected by the Camera
	 * @return true if this robot has his LED "led8" on, false otherwise
	 */
	private boolean isLightON(CameraRecognitionObject robot) {
		int[] image=camera.getImage();
		boolean detected=false;

		int[] position=robot.getPositionOnImage();
		int width=robot.getSizeOnImage()[0];
		int height=robot.getSizeOnImage()[1];

		int startx=position[0] - (width + 1)/2;
		int starty=position[1] - (height + 1)/2;

		for (int i = 0; i < width; i++) {
			for(int j=0;j< height;j++) {
				int pixel=image[(startx+i)+(camera.getWidth() * (starty+j))];
				if(Camera.pixelGetRed(pixel) >= 254 && 	Camera.pixelGetGreen(pixel) >= 254 && Camera.pixelGetBlue(pixel) < 200) {
					if (detected) return true;
					else detected=true;					
				}
			}
		}
		return false;

	}

	/**
	 * Allows to send a message at all the other robots
	 * @param message
	 */
	protected void broadcastMessage(String message) {
		emitter.send(message.getBytes());
	}

	/**
	 * Check if a message has been received, and flush the pile
	 * @return null if there is no message, a String otherwise
	 */
	protected String checkMailBox() {
		while(receiver.getQueueLength() > 0) {
			byte[] message=receiver.getData();
			receiver.nextPacket();
			if(message != null) {
				return new String(message);	
			}
			else return null;
		}
		return null;
	}

	private void moveRandomly() {
		double left=50;
		double right=50;
		else move(left,right);
	}

	private boolean checkAndAvoidObstacle() {

		double[] ps_values=readDistanceSensorValues();

		// capteurs gauche
		boolean left_side = ps_values[5] > detection_threshold;
		boolean left_corner = ps_values[6] > detection_threshold;
		boolean left_front = ps_values[7] > detection_threshold;

		// capteurs droits
		boolean right_side = ps_values[2] > detection_threshold;
		boolean right_corner = ps_values[1] > detection_threshold;
		boolean right_front = ps_values[0] > detection_threshold;

		if (left_front)
			move(speed, speed * (Math.random() * -1));

		else if (right_front)
			move(speed * (Math.random() * -1), speed);

		else if (left_corner)
			move(speed, speed * (Math.random() *0.3));

		else if (right_corner)
			move(speed * (Math.random() *0.3), speed);

		else if (left_side)
			move(speed, speed * (Math.random() *0.8));

		else if (right_side)
			move(speed * (Math.random() *0.8), speed);

		else return false;

		return true;		
	}
	
	protected double distance(double x, double y) {
		return x*x+y*y;
	}
	
	protected Vector2d attraction(ArrayList<CameraRecognitionObject> robots) {
		Vector2d vector2d=new Vector2d(0, 0);
		for(CameraRecognitionObject other:robots) {
			vector2d.add(new Vector2d(other.getPosition()[0], other.getPosition()[1]));			
		}
		
		if(robots.size() > 0) {
			vector2d.divide(robots.size());
			vector2d.mult(attraction_coef);
		}
		return vector2d;
	}
	
	protected Vector2d repulsion(ArrayList<CameraRecognitionObject> robots) {
		Vector2d vector2d=new Vector2d(0, 0);
		for(CameraRecognitionObject other:robots) {
			Vector2d vother=new Vector2d(other.getPosition()[0], other.getPosition()[1]);
			vother.mult(-1);
			vector2d.add(vother.normalize());			
		}
		
		if(robots.size() > 0) {
			vector2d.divide(robots.size());
			vector2d.mult(repulsion_coef);
		}
		return vector2d;
	}
	

	/**
	 * The main method of the robot behaviour
	 */	
	public void run() {		
		// main control loop: perform simulation steps of timeStep milliseconds
		// and leave the loop when the simulation is over
		while (step(timeStep) != -1) {	
			speed=50;

			
			ArrayList<CameraRecognitionObject> detected=(ArrayList<CameraRecognitionObject>) cameraDetection();			
			ArrayList<CameraRecognitionObject> robots=(ArrayList<CameraRecognitionObject>) otherRobotsDetected(detected);

			boolean obstacle=checkAndAvoidObstacle();

			if(! obstacle) {

				if(robots.size()> 0) {
					
					Vector2d vector=attraction(robots);
					vector.add(repulsion(robots));
										
					if(vector.getY() > 0) {
						double angle=Math.acos(vector.normalize().getY() * -1) - Math.PI/2;
						move(speed*(1-angle/(Math.PI/2)), speed);
					}
					else if(vector.getY() < 0){
						double angle=Math.PI/2 - Math.acos(vector.normalize().getY() * -1);
						move(speed, speed*(1-angle/(Math.PI/2)));
						
					}
					else {
						move(speed, speed);
					}
					
				}
				else {
					moveRandomly();
				}
			}
		}
	}
	
	
	public static void main(String[] args) {
		Flocking_FV controller = new Flocking_FV();
		controller.run();
	}
	
	
	private class Vector2d {

	private double x;
	private double y;
	
	public Vector2d(double x, double y) {
		// TODO Auto-generated constructor stub
		this.x = x;
		this.y = y;
	}
	
	public void add(Vector2d vector) {
		this.x=x+vector.getX();
		this.y=y+vector.getY();
	}
	
	public Vector2d normalize() {
		double dist=length();
		return new Vector2d(x/dist, y/dist);
	}
	
	public void mult(double value) {
		this.x=x*value;
		this.y=y*value;
	}
	
	public void divide(double value) {
		this.x=x/value;
		this.y=y/value;
	}
	
	public double length() {
		return Math.sqrt(x*x+y*y);
	}
	
	public String toString() {
		return "{"+x+"+"+y+"}";
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}
	
	
}

	
}