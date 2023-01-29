/*
 * File:          supervisor_efficacite.c
 * Date: 25/05/2022
 * Description: This supervisor places all robots randomly. The robots must be in the world and
 *              have a DEF. If all robots have found the target, the supervisor pause 
 *              the simulation. It also calculate the efficiency of the program : sum of the distance at_the beginning of
 *              simulation of each robot and the target / sum of distance traveled by each robot.
 * Author: Nolwenn
 * Modifications: Le 19/01/2023
 * 
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>


#define TIME_STEP 256
#define NB_EPUCK 10
#define ARENA_SIZE 100
#define RANGE_DETECTION 0.36
#define RANDOM true
#define R_FoV 2.0
#define R_DEPLOY 1.0

#define MSG_SIZE 128

// Variable globale
WbNodeRef robots[NB_EPUCK];
WbFieldRef trans_field[NB_EPUCK];
WbFieldRef rttn_field[NB_EPUCK];
WbFieldRef controller_field[NB_EPUCK];
WbFieldRef window_field[NB_EPUCK];

WbFieldRef receiver_channel_field[NB_EPUCK];
WbFieldRef emitter_channel_field[NB_EPUCK];

WbFieldRef name_field[NB_EPUCK];
WbFieldRef trans_field_target;
WbFieldRef synch_field[NB_EPUCK];

/*
 * random number between min and max
 */
double random_between(int min, int max)
{
  int delta;
  double res;

  delta = max - min + 1;
  res = rand() % delta;
  res = (res + min) / 100;

  return res;
}

/*
 * Initialize robots[] and trans_field
 * function based on static void reset(void); in advanced_particle_swarm_optimization_supervisor.c
 */
void set_robots(void)
{
  // Name of the robot we are linking.
  char rob[7] = "epuck0";
  char def[30] = "DEF ";
  int i = 0;
   
  for (; i < NB_EPUCK; i++)
  {
      WbNodeRef root_node = wb_supervisor_node_get_root();
      WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
      strcpy(def,"DEF ");
      strcat(def,rob);
      strcat(def," E-puckCameraRecognition{ }");
      wb_supervisor_field_import_mf_node_from_string(root_children_field, 4, def);

    
    // Indicate where robot location has to be stored.
    robots[i] = wb_supervisor_node_get_from_def(rob);
    if (robots[i] == NULL)
    {
      fprintf(stderr, "No DEF node found in the current world file\n");
      exit(1);
    }

    trans_field[i] = wb_supervisor_node_get_field(robots[i], "translation");
    rttn_field[i] = wb_supervisor_node_get_field(robots[i], "rotation");
    window_field[i]= wb_supervisor_node_get_field(robots[i], "window");
    receiver_channel_field[i]= wb_supervisor_node_get_field(robots[i], "receiver_channel");
    controller_field[i] = wb_supervisor_node_get_field(robots[i], "controller");
    name_field[i]= wb_supervisor_node_get_field(robots[i], "name");
    synch_field[i]=wb_supervisor_node_get_field(robots[i], "synchronization");


    // Update robot name.
    rob[5]++;
  }

  return;
}

/*
 * Places each robot randomly in the arena
 * function from advanced_particle_swarm_optimization_supervisor.c
 */
void random_position(void)
{
  double p[3] = {0, 0, 0};
  double r[4] = {0, 0, 1, 0};
  bool synch=true;
  char rob[8] = "epuck0";
  
  if(! RANDOM){
  double x=0.12;
  double y=0.12;  
   
  int n = 0;
  for (n=0; n < NB_EPUCK; n++)
  {   
    p[0] = x; // x coordinate
    p[1] = y; // y coordinate

    x = x + 0.2;
    if(x > 0.8){
      x = 0.12;
      y = y - 0.2;
    }

    r[3] = random_between(-314, 314);

    wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
    wb_supervisor_field_set_sf_rotation(rttn_field[n], r);
    wb_supervisor_field_set_sf_string(window_field[n], "e-puck");
    wb_supervisor_field_set_sf_int32(receiver_channel_field[n], n+1);
    wb_supervisor_field_set_sf_string(controller_field[n], "PatternFormation");
    wb_supervisor_field_set_sf_string(name_field[n], rob);
    wb_supervisor_field_set_sf_bool(synch_field[n],synch);
    rob[5]++;

    printf("Robot: %s (x = %f, y = %f, theta = %f)\n", rob, p[0], p[1], r[3]);        
  }
  }
  else{
  
    int n = 0;
    double angle = random_between(-314, 314);
    for (; n < NB_EPUCK; n++){
      
      p[0] = R_DEPLOY * cos(angle + n * 2*M_PI / NB_EPUCK); ;// random_between(-ARENA_SIZE, ARENA_SIZE); // 
      p[1] = R_DEPLOY * sin(angle + n * 2*M_PI / NB_EPUCK); ;//random_between(-ARENA_SIZE, ARENA_SIZE); //

      r[3] = random_between(-314, 314);

      wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
      wb_supervisor_field_set_sf_rotation(rttn_field[n], r);
      wb_supervisor_field_set_sf_string(window_field[n], "e-puck");
      wb_supervisor_field_set_sf_int32(receiver_channel_field[n], n+1);
      wb_supervisor_field_set_sf_string(controller_field[n], "PatternFormation");
      wb_supervisor_field_set_sf_string(name_field[n], rob);
      wb_supervisor_field_set_sf_bool(synch_field[n],synch);
      rob[5]++;
      
      printf("Robot: %s (x = %f, y = %f, theta = %f)\n", rob, p[0], p[1], r[3]);        
    }
  
}
  printf("Random positioning done.\n");
  return;
}

/*
 * Return true if a robot is in the range
 */
bool in_the_range(const double *values_robots, const double *values_cible)
{
  bool x_range = values_robots[0] < values_cible[0] + RANGE_DETECTION && values_robots[0] > values_cible[0] - RANGE_DETECTION;
  bool y_range = values_robots[1] < values_cible[1] + RANGE_DETECTION && values_robots[1] > values_cible[1] - RANGE_DETECTION;

  return x_range && y_range;
}

/*
 * Return true if all robots have found the target
 */
bool simulation_finished(bool robots_stopped[NB_EPUCK])
{
  bool res = true;

  for (int i = 0; i < NB_EPUCK; i++)
    res = res && robots_stopped[i];

  return res;
}

/*
 * Return a distance between 2 points of the plan
 */
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}

char* doubleToString(double num) {
    char* str = (char*)malloc(100);
    sprintf(str, "%lf", num);
    return str;
}

/* create_message and send_message*/

int send_message(int robot_index){
      const double *values_robots[NB_EPUCK], *rotations[NB_EPUCK];
      double x, y;
      double theta;
      char  rob[7] = "epuck0", message[1024] = "";//, data[256] = "";
      char sep[2] = ",", end[2] = ";";
      int success = 0;
      int i = 0;
      const double *values_robot = wb_supervisor_field_get_sf_vec3f(trans_field[robot_index]);
      for(i = 0; i < NB_EPUCK; i++){
        values_robots[i] = wb_supervisor_field_get_sf_vec3f(trans_field[i]);
        rotations[i] = wb_supervisor_field_get_sf_rotation(rttn_field[i]);
        // données à envoyer:
        x = values_robots[i][0];
        y = values_robots[i][1];
        theta = rotations[i][3];
        double _distance = distance(values_robot[0], values_robot[1], x, y);

        if(_distance < R_FoV){
          strcat(message, rob);
          strcat(message, sep);
          strcat(message, doubleToString(x));
          strcat(message, sep);
          strcat(message, doubleToString(y));
          strcat(message, sep);
          strcat(message, doubleToString(theta));
          strcat(message, end);
        }
        rob[5]++;
      }
      WbDeviceTag tag = wb_robot_get_device("emitter");
      wb_emitter_set_channel(tag, robot_index + 1);
      success = wb_emitter_send(tag, message, strlen(message) * sizeof(char));     

    return success;
  }

/* fin creat_message and send message */

int main()
{ 
  wb_robot_init();
  srand(time(NULL));
  // setup
  set_robots();
  random_position();
  
  // update   
  while (wb_robot_step(TIME_STEP) != -1)
  {
     // envoi de message
     for(int k = 0; k < NB_EPUCK; k++){
       send_message(k);
     }
  }

  wb_robot_cleanup();
  return 0;
}