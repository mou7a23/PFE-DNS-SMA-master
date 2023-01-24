import SWIG_generated_files.Robot;

import java.util.Vector;

public class DNS extends Robot {

    private int timeStep = 32;
    private enum Mode {
        STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN, MOVE_BACKWARDS, ALTER_COURSE
    }




    protected void move(double left, double right) {
        double max=6.2;
        getMotor("left wheel motor").setVelocity(left * max / 100);
        getMotor("right wheel motor").setVelocity(right * max / 100);
    }

    protected void turn(double speed){
        move(speed, -speed);
    }

    public int scanSpace(float[] formationVector){
        int response = 0;
        boolean activeA = false;
        boolean activeB = false;
        boolean activeC = false;
        boolean activeD = false;



        return response;
    }

/*    if(activeA){ // 1, 3, 5, 7, 9, 11, 13, 15
        avoidObstacles();
    } else // 0, 2, 4, 6, 8, 10, 12, 14
            if(activeB){ // 2, 6, 12, 14
        if (activeC){ // 6, 14
            alter_course();
        } else{ // 2, 10
            move_forward();
        }
        move_forward();
    } else if (activeC){ // 4, 12
        alter_course();
    } else if(activeD){ // 8
        move_backwards();
    }
        else { // 0
        stop();
    }

 */
    private void alter_course() {
    }

    private void move_forward() {
    }

    private void move_backwards() {
    }

    public void avoidObstacles() {
    }
    public void stop(){

    }

    public void run() {

        // perform a simulation steps and leave the loop when the simulation is ended
        while (step(timeStep) != -1) {
            // Read sensors, particularly the order of the supervisor
            if (receiver.getQueueLength()>0){
                String message = new String(receiver.getData());
                receiver.nextPacket();
                System.out.println("I should "+AnsiCodes.RED_FOREGROUND+message+AnsiCodes.RESET+"!");
                if (message.equals("avoid obstacles"))
                    mode = Mode.AVOID_OBSTACLES;
                else if (message.equals("move forward"))
                    mode = Mode.MOVE_FORWARD;
                else if (message.equals("stop"))
                    mode = Mode.STOP;
                else if (message.equals("turn"))
                    mode = Mode.TURN;
            }
            double delta = distanceSensors[0].getValue()-distanceSensors[1].getValue();
            double[] speeds = {0.0, 0.0};

            // send actuators commands according to the mode
            switch (mode){
                case AVOID_OBSTACLES:
                    speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
                    speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
                    break;
                case MOVE_FORWARD:
                    speeds[0] = maxSpeed;
                    speeds[1] = maxSpeed;
                    break;
                case TURN:
                    speeds[0] =  maxSpeed / 2.0;
                    speeds[1] = -maxSpeed / 2.0;
                    break;
                default:
                    break;
            }
            motors[0].setVelocity(speeds[0]);
            motors[1].setVelocity(speeds[1]);
        }
    }

}
