
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.lang.model.util.ElementScanner6;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.PrintWriter;
import java.io.IOException;  // Import the IOException class to handle errors

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Emitter;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Receiver;
import com.cyberbotics.webots.controller.Robot;
import java.lang.Math;

public class PatternFormation extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private Camera camera;
	private Emitter emitter;
	private Receiver receiver;
	private LED[] leds;

	private double detection_threshold=80;
	private double speed = 50;
	private double rotate = 10;
	public int NB_EPUCK = 33;
	// private String robID;
    double alpha = Math.toRadians(90); // l'angle génératrice du vecteur F en radians
	double deadAheadWidth = Math.toRadians(18); // l'angle de dead ahead en radians; zone 'C' [0, 18]
	double r_avoid = 0.1; // le rayon de la zone A
	double epsilon = Math.PI/24;// paramètre à régler empiriquement.

	double x, y, theta;
	// List of neighbours
	private List<Voisin> voisins = new ArrayList<Voisin>();
           
	public PatternFormation() {
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
	
	protected void update(String message){
		if (message != null){
			String[] robots_data = message.split(";", NB_EPUCK); // NB_EPUCK = 10
			String[] robot_data;
			this.voisins.clear();
			for (int iter = 0; iter < robots_data.length - 1; iter++){
				robot_data = robots_data[iter].split(",", 4);  
				if (robot_data[0].equals(this.getName())){
					this.x = Double.parseDouble(robot_data[1]);
					this.y = Double.parseDouble(robot_data[2]);
					this.theta = Double.parseDouble(robot_data[3]);
				} else {
					if(!robot_data[0].equals("")){
						this.voisins.add(new Voisin(robot_data[0], Double.parseDouble(robot_data[1]), Double.parseDouble(robot_data[2]), Double.parseDouble(robot_data[3])));
					}
				}
			}
		}
	}

	
	protected void afficher_message(String message){
           if(message != null){
			System.out.println("{robot: " +this.getName()+", msg: "+message+"}");
           }
	}
	
	// this was included in update function above
	public void set_voisins(String message){
               if(message != null){
  	    String[] robots_data = message.split(";", 11); // NB_EPUCK = 10
  	    String[] robot_data;
  	    this.voisins.clear();
		for (String str_data:robots_data){
			robot_data = str_data.split(",", 4);
			if(!robot_data[0].equals("")){
					if (!robot_data[0].equals(this.getName())){
					this.voisins.add(new Voisin(robot_data[0], Double.parseDouble(robot_data[1]), Double.parseDouble(robot_data[2]), Double.parseDouble(robot_data[3])));
					}     
			}
		}
  	    }  
	}
	
	
	protected void afficher_voisins(){
  	    String[] robot_data;
  	    double dist;
  	    System.out.println("I am "+this.getName()+"("+this.x+", "+ this.y+")"+" Orientation: "+this.theta);
  	    if (this.voisins.isEmpty())
      	       System.out.println(ConsoleColors.RED+"I don't have neighbors!!"+ConsoleColors.RESET);
      	    else{
  	    for (Voisin voisin:this.voisins){
  	            dist = Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y));
                System.out.println("{Voisin: "+voisin.get_name()+", Coordinates: ("+voisin.get_x()+","+voisin.get_y()+"), Distance: "+dist+"}");
  	        }
  	    }
	}
           
         
    public Voisin get_closest_neighbor(){
		double dist_min = 10000.0, dist;
		Voisin closest_neighbor = new Voisin();
		for(Voisin voisin: voisins){
			dist = Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y));
			if(dist_min > dist){
				closest_neighbor = voisin;
				dist_min = dist;
			}  
		}
		if(dist_min == 10000.0)
			return null; // n'a pas de voisins
		return closest_neighbor;
    }

    
	
	private void moveRandomly() {
		double left=50;
		double right=50;
		move(left,right);
	}

	// Perception
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
	
	protected double distance(double x, double y, double x_v, double y_v) {
		return Math.sqrt((x_v-x)*(x_v-x)+(y_v-y)*(y_v-y));
	}

	public void to_action(){
		double f = this.alpha;
		double r_zone_A = this.r_avoid;
		double e = this.epsilon;
		double pi_2 = Math.PI/2;
		double deadAheadAngle = this.deadAheadWidth;
		double x_r = this.x, y_r = this.y, x_v, y_v;
		double dist;
		double ang;
		boolean activeA = false, activeB = false, activeC = false, activeD = false;
		for(Voisin voisin: this.voisins){
			x_v = voisin.get_x();
			y_v = voisin.get_y();
			dist = distance(x_r, y_r, x_v, y_v);
			ang = Math.atan2(x_v-x_r, y_v-y_r);
			if(dist < r_zone_A){
				if(!activeA)
					System.out.println(ConsoleColors.RED+"Zone A avtivée par: "+voisin.get_name()+"("+voisin.get_x()+", "+voisin.get_y()+")"+ConsoleColors.RESET);
				activeA = true;
			} else if(Math.abs(ang - f) < deadAheadAngle){
				if(!activeC)
					System.out.println(ConsoleColors.GREEN+"Zone C avtivée par: "+voisin.get_name()+"("+voisin.get_x()+", "+voisin.get_y()+")"+" ang = "+Math.abs(ang)+ConsoleColors.RESET);
				activeC = true;
			} else if(Math.cos(ang - f) > 0.05){ // 0.01 intervalle de tolérence pour permettre aux robots de s'arreter lorsqu'ils soient un à coté de l'autre sur la ligne
				if(!activeB)
					System.out.println(ConsoleColors.YELLOW+"Zone B avtivée par: "+voisin.get_name()+"("+voisin.get_x()+", "+voisin.get_y()+")"+" ang = "+Math.cos(ang - f)+ConsoleColors.RESET);
				activeB = true;
			} else if(Math.cos(ang - f) < -0.05){
				if(!activeD)
					System.out.println(ConsoleColors.BLUE+"Zone D avtivée par: "+voisin.get_name()+"("+voisin.get_x()+", "+voisin.get_y()+")"+ConsoleColors.RESET);
				activeD = true;
			}		
		}
		if(activeA){
			System.out.println(ConsoleColors.RED+"Decision: Avoid Collision Behaviour"+ConsoleColors.RESET);
			avoid_collision();
		} else if(activeC){
			System.out.println(ConsoleColors.GREEN+"Decision: Alter Course Behaviour"+ConsoleColors.RESET);
			alter_course();
		} else if(activeB){
			System.out.println(ConsoleColors.YELLOW+"Decision: Forward Behaviour"+ConsoleColors.RESET);
			forward();
		} else if(activeD){
			System.out.println(ConsoleColors.BLUE+"Decision: Backwards Behaviour"+ConsoleColors.RESET);
			backwards();
		} else{
			System.out.println(ConsoleColors.PURPLE_BOLD+"Decision: Stop Behaviour"+ConsoleColors.RESET);
			stop();
		}
	}

	//////////////////////// Actions ///////////////////////////////////:

	public void stop(){
		move(0.0, 0.0);	
	}
			
	public void avoid_collision(){ // Done: en utilisant la répulsion
		Voisin closest = this.get_closest_neighbor();
		double x_v = closest.get_x() - this.x;
		double y_v = closest.get_y() - this.y;
		double angle = Math.atan2(y_v, x_v);
		if(Math.abs(angle) > Math.PI/4){
			move(speed, speed);	
		} else if(angle < - 0.1){
			move(+rotate, -rotate);
		} else if(angle > 0.1){
			move(-rotate, +rotate);
		} else{
			move(-speed, -speed);
		}
	}
	
	public void alter_course(){
		double f = this.alpha - Math.PI/2; // alpha = f + PI/2
		if(Math.abs(this.theta-(f+Math.PI/2)) > + epsilon){
			System.out.println("Theta > Alpha: "+Math.abs(this.theta)+" > "+this.alpha+" + epsilon: "+this.epsilon);
			move(+rotate, -rotate);
		} else if(Math.abs(this.theta) < Math.abs(f+Math.PI/2) - epsilon){
			System.out.println("Theta < Alpha: "+Math.abs(this.theta)+" < "+this.alpha+" - epsilon: "+this.epsilon);
			move(-rotate, +rotate);
		} else {
		move(speed, speed);	
		}	
	}
	
	public void forward(){
		double f = this.alpha - Math.PI/2; // alpha = f + PI/2 ==> 
		double max_ps = -9999.0;
		double vitesse = 0;
		double produit_scalaire = 0;
		for(Voisin voisin: this.voisins){
			 produit_scalaire = (voisin.get_x()-this.x) * Math.cos(alpha) + (voisin.get_y()-this.y)*Math.sin(alpha);
			if(produit_scalaire > max_ps){
				max_ps = produit_scalaire;
				vitesse = produit_scalaire / distance(voisin.get_x()-this.x, voisin.get_y()-this.y, Math.cos(alpha), Math.sin(alpha));
			}
		}
		System.out.println("Speed: "+vitesse+" max_ps = "+max_ps);
		if(Math.abs(this.theta) > Math.abs(f) + epsilon){
			System.out.println("Theta > alpha+e: "+Math.abs(this.theta)+" > "+(f+this.epsilon));
			move(+rotate, -rotate);
		} else if(Math.abs(this.theta) < Math.abs(f) - epsilon){
			System.out.println("Theta < alpha-e: "+Math.abs(this.theta)+" < "+(f+this.epsilon));
			move(-rotate, +rotate);
		} else {
			System.out.println("Theta ~ alpha & Vitesse: "+(vitesse*100));
			move(100*vitesse, 100*vitesse);
		} 
		
	}
	
	public void backwards(){
		double f = this.alpha - Math.PI/2; // alpha = f + PI/2 ==> 
		double min_ps = 9999.0;
		double vitesse = 0;
		double produit_scalaire = 0;
		for(Voisin voisin: this.voisins){
			produit_scalaire = (voisin.get_x()-this.x) * Math.cos(alpha) + (voisin.get_y()-this.y)*Math.sin(alpha);
			if(produit_scalaire < min_ps){
				min_ps = produit_scalaire;
				vitesse = produit_scalaire / distance(voisin.get_x()-this.x, voisin.get_y()-this.y, Math.cos(alpha), Math.sin(alpha));	
			}
		}
		System.out.println("Speed: "+vitesse+" min_ps = "+min_ps);
		if(Math.abs(this.theta) > Math.abs(f) + epsilon){
			System.out.println("Theta > alpha+e: "+this.theta+" > "+(f+this.epsilon));
			move(+rotate, -rotate);
		}else if(Math.abs(this.theta) < Math.abs(f) - epsilon){
			System.out.println("Theta < alpha-e: "+this.theta+" < "+(f+this.epsilon));
			move(-rotate, +rotate);
		} else {
			System.out.println("Theta ~ alpha & Vitesse: "+(vitesse*100));
			move(100*vitesse, 100*vitesse);
		} 
	}
	

	////////////////////////////////////////////////////////////////////;

	// La fonction sert à enregistrer les données des robots dans un fichier texte.
	public void writeToFile(String message){
		if(message != null){
			try {
				PrintWriter myWriter = new PrintWriter(new FileWriter("filename.txt", true));
				myWriter.println(this.getName()+":{"+message+"}");
				myWriter.close();
				System.out.println("Successfully wrote to the file.");
			} catch (IOException e) {
				System.out.println("An error occurred.");
				e.printStackTrace();
			}	
		}
	}

	/**
	 * The main method of the robot behaviour
	 */	
	public void run() {
		// Affectation de groupes aux robots
		// boolean in = false;
		// String[] names = {"epuck0", "epuck1", "epuck2"};//, "epuck3", "epuck4", "epuck5", "epuck6", "epuck7", "epuck8", "epuck9"};
		// for(String name: names){
		// 		if(name.equals(this.getName()))
		// 			in = true;
		// }
		// if(in){
		// 	this.alpha = Math.toRadians(45);
		// } else {
		// 	this.alpha = Math.toRadians(-45);
		// }
		while (step(timeStep) != -1) {
          	System.out.println("Robot's name: " +this.getName()+"("+this.x+", "+this.y+")");
			String message;
			Boolean obstacle;
			obstacle = checkAndAvoidObstacle();
			message = this.checkMailBox();
			this.update(message);
			// this.afficher_message(message);
			// this.afficher_voisins();
			if(!obstacle){
				this.to_action();
			}
			
		}
	}


	public static void main(String[] args) {
		PatternFormation controller = new PatternFormation();
		controller.run();
	}
}