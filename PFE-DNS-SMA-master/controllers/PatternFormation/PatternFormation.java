
import java.util.ArrayList;
import java.util.List;

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
// import Voisin;

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
	private double speed;
	// private String robID;
           double alpha = 0; // l'angle génératrice du vecteur F en radians
           double omega = 1.9; // l'angle de dead ahead en radians; zone 'C'
           double rA = 0.3; // le rayon de la zone A
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
        	    String[] robots_data = message.split(";", 10); // NB_EPUCK = 10
        	    String[] robot_data;
        	    for (int iter = 0; iter < robots_data.length - 1; iter++){
            	robot_data = robots_data[iter].split(",", 4);  
      	           if (robot_data[0].equals(this.getName())){
          	              this.x = Double.parseDouble(robot_data[1]);
          	              this.y = Double.parseDouble(robot_data[2]);
          	              this.theta = Double.parseDouble(robot_data[3]);
      	            }
      	        }
      	    }
	}
	
	protected void afficher_message(String message){
           if(message != null){
  	  System.out.println("I am " +this.getName());
	  System.out.println(" Message reçu : "+message);
	  System.out.println("This is my Position ("+this.x+","+this.y+") and my Orientation: "+this.theta);
           }
	}
	
	public void set_voisins(String message){
               if(message != null){
  	    String[] robots_data = message.split(";", 11); // NB_EPUCK = 10
  	    String[] robot_data;
  	    this.voisins.clear();
	     for (String str_data:robots_data){
        	         robot_data = str_data.split(",", 4);
                    if(!robot_data[0].equals("")){
                    	 if (!robot_data[0].equals(this.getName())){
                            this.voisins.add(new Voisin(robot_data[0], Double.parseDouble(robot_data[1]), Double.parseDouble(robot_data[3]), Double.parseDouble(robot_data[2])));
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
      	        System.out.println(" ");
  	    for (Voisin voisin:this.voisins){
  	             dist = Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y));
                  	  System.out.print(voisin.get_name()+" Distance: "+dist);
  	        }
  	        System.out.println(" ");
  	    }
	}
           
           
    public void to_action(){
                 double angleF = Math.toRadians(this.alpha);
                 double angleC = Math.toRadians(this.omega);
                 boolean activeA = false, activeB = false, activeC = false, activeD = false;
                 double aux, aux_cos, aux_sin; 
                 // Decision  
                 for(Voisin voisin: this.voisins){
                     if(Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y)) < rA){
                         activeA = true;
                     } else {
                     if(voisin.cos_angle(this.x, this.y) > Math.cos(angleF+angleC)){
                        // cosX > cosY => X < Y
                        activeC = true;
                     } else {
                     aux_cos = voisin.cos_angle(this.x, this.y);
                     aux_sin = voisin.sin_angle(this.x, this.y);
                     aux = aux_cos * Math.cos(angleF) + aux_sin * Math.sin(angleF);
                     if(aux > 0){
                       activeB = true;
                       }
                       else {
                         activeD = true;
                         }
                       }
                     }
                 }
				// to Action
                if(activeA){
                   avoid_collision();
                } else {
                 if(activeC){
                   alter_course();
                 } else {
                       if(activeB){
                          forward();
                       } else {
                          if(activeD){
                               backwards();
                          } else {
                            stop();
                          }
                       }
                 }
              }   
           
           
    }
           

           public void stop(){
				System.out.println(ConsoleColors.PURPLE+"Decision: Stop"+ConsoleColors.RESET);
				move(0, 0);
				
           }
           

           public Voisin get_closest_neighbor(){
                 double dist_min = 10000, dist;
                 Voisin closest_neighbor = new Voisin();
                 for(Voisin voisin: voisins){
                     dist = Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y));
                     if(dist_min > dist){
                         closest_neighbor = voisin;
                         dist_min = dist;
                     }  
                 }
                 if(dist_min > 1000)
                    return null; // n'a pas de voisins
              return closest_neighbor;
           }
           
           
           public void avoid_collision(){
                System.out.println(ConsoleColors.RED+"Decision: Avoid Collision Behaviour"+ConsoleColors.RESET);
				double epsilon = 0.15, speed_reducer = 10; // paramètre à régler empiriquement.
				Voisin voisin = get_closest_neighbor();
				double r_x = voisin.get_x() - this.x;
				double r_y = voisin.get_y() - y;
				double vitesse = Math.sqrt(r_x*r_x+r_y*r_y);
				vitesse = vitesse * 100; // la distance obtenue est calculé en [m], on la transforme en [cm]
				if(Math.abs(this.theta - Math.atan2(r_y, r_x)) < epsilon){
					move(speed/speed_reducer, -speed/speed_reducer);
				} else {
					move(vitesse, vitesse);
					if(vitesse<20){
						move(speed, speed);
					}
				}
                
           }

           public void alter_course(){
			System.out.println(ConsoleColors.GREEN+"Decision: Alter Course Behaviour"+ConsoleColors.RESET);
			double epsilon = 0.15, speed_reducer = 5; // paramètre à régler empiriquement.
			if(Math.abs(this.theta - (alpha + Math.PI)) < epsilon){ // on cherche l'orientation du vecteur perpencutaire à F
					move(speed/speed_reducer, -speed/speed_reducer);
				}
			else {
				move(speed, speed);
			}
           }


           public void forward(){
			System.out.println(ConsoleColors.YELLOW+"Decision: Forward"+ConsoleColors.RESET);
			double max_ps = -9999.0;
			double epsilon = 0.15, speed_reducer = 5; // paramètre à régler empiriquement.
			for(Voisin voisin: this.voisins){
				double produit_scaler= (this.x-voisin.get_x())*Math.cos(alpha) + (this.y-voisin.get_y())*Math.sin(alpha);
				if(produit_scaler > max_ps){
					max_ps = produit_scaler;	
				}
			}
			if(Math.abs(this.theta - alpha) < epsilon){
					move(speed/speed_reducer, -speed/speed_reducer);
				}
			else {
				move(max_ps, max_ps);
				if(max_ps < 20){
					move(speed, speed);
				}
			}
           }


           public void backwards(){
            System.out.println(ConsoleColors.BLUE+"Decision: Backwards"+ConsoleColors.RESET);
			double min_ps = 9999.0;
			double epsilon = 0.15, speed_reducer = 5; // paramètre à régler empiriquement.
			for(Voisin voisin: this.voisins){
				double produit_scaler= (this.x-voisin.get_x())*Math.cos(alpha) + (this.y-voisin.get_y())*Math.sin(alpha);
				if(produit_scaler < min_ps){
					min_ps = produit_scaler;	
				}
			}
			if(Math.abs(this.theta - alpha) < epsilon){
					move(speed/speed_reducer, -speed/speed_reducer);
				}
			else {
				move(min_ps*100, min_ps*100);
				if(min_ps * 100 > -20){
					move(-speed, -speed);
				}
			}
           }
           
	private void moveRandomly() {
		double left=50;
		double right=50;
		move(left,right);
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
		return Math.sqrt(x*x+y*y);
	}


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
		// main control loop: perform simulation steps of timeStep milliseconds
		// and leave the loop when the simulation is over
		// int nb_co_swarm = 10;
		
		while (step(timeStep) != -1) {
			speed=50;
			// Perception
			String message;
			message = this.checkMailBox();
			this.update(message);
			this.set_voisins(message);
			
			/*double dist;
			Voisin voisin;
			voisin = this.get_closest_neighbor();
			if(voisin != null){
			dist = Math.sqrt((voisin.get_x()-this.x)*(voisin.get_x()-this.x)+(voisin.get_y()-this.y)*(voisin.get_y()-this.y));
			System.out.print(this.getName()+" <-- ");
			System.out.print(dist+" --> ");
			voisin.afficher();
			}*/
			
			// writeToFile(message); // permet d'enregistrer les messages reçus par les robots dans un fichier text
			// afficher_message(message); // Affiche le message reçu par le robot
			// afficher_voisins(); // affiche les voisins du robot
            
			// Decision & Action
			this.to_action();                     
			
			/*/boolean obstacle = checkAndAvoidObstacle();
			if(! obstacle) {		
			}*/
		}
	}



	public static void main(String[] args) {
		PatternFormation controller = new PatternFormation();
		controller.run();
	}
}