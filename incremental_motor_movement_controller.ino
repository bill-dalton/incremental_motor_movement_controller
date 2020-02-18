/* Purpose: 
 *    To better understand and control joint movements of a compliant cable driven robotic arm
 *    This accomodates the joint encoders that have been added (July 2019) to the joint itself for SL, UR & EL joints
 *    BR, LR, WL & WR joints already has such encoders
 * Runs on:
 *    Teensy, Arduino driving stepper motors
 *    For Dynamixels on OpenCM, use xl430_incremental_motor_movement_controller
 * Subscribes to ROS Topics:
 *    commanded_incremental_motor_movement
 *    minion_override_command
 * Publishes to ROS Topics:   
 *    BR_minion_state, SL_minion_state, etc
 * Action:   
 *    Moves motor to goal using RAMP algorithm
 *    Stops motion and motion timers if eother: endstop activated while moving, or lost comm, or estop command from master
 *    a valid new plan from master will cancel any endstop, lost comm, or emrg stop conditions
 *    lost comm time updating done in overrideCommandReceivedCallback(), lost comm monitoring done in loop()
 *    listening for estop command from master done in overrideCommandReceivedCallback()
 *    endstop monitoring done in stepOnce()
 * Modifications Dec 2019 to implement minion_state.operating_state concept
 *    convert operating_state and torque_state into an integer concept to publish as int minion_state.operating_state
 *    removed states string messages concept Dec 2019. This has been replaced by minion_state.operating_state and operating_state enum
 * Modifications Dec 2019 to implement minion_override_command concept
 *    implement allowed_to_move_cw & allowed_to_move_ccw concept
 * Modification to be non-blocking
 *    removed RAMP stuff
 *    added Timer1
 * Timers:
 *    Timer1 - fires to change stepper interval
 *    Timer3 - fires to pulse stepper
 *    millis() - used to periodically publish position and state info
 * To Do:
 *    
 */

#include <ros.h>
#include <ArduinoHardware.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <Ramp.h>
#include <my_robotic_arm/MinionState.h>
#include <my_robotic_arm/GetUpperarmOrientation.h>
#include <my_robotic_arm/GetLowerarmOrientation.h>
#include <geometry_msgs/Vector3.h>
//#include <Adafruit_MMA8451.h>
//#include <Adafruit_Sensor.h>

//states
enum {NORMAL=0,
      EMERG_STOPPED_BY_MASTER=-1,
      LOST_COMM_FROM_MASTER=-2,
      POSITIVE_ENDSTOP_ACTIVATED=-3,
      NEGATIVE_ENDSTOP_ACTIVATED=-4
      } operating_state;
//enum {GREEN, YELLOW, RED} torque_state;

/*//states string messages
static char states_msg[100];
static char empty_states_msg[100] = "States: ";
static char unpowered_msg[] = "unpowered ";
static char holding_position_msg[] = "holding_position ";
static char moving_msg[] = "moving ";
static char final_approach_msg[] = "final_approach ";
static char cw_endstop_activated_msg[] = "cw_endstop_activated ";
static char ccw_endstop_activated_msg[] = "ccw_endstop_activated ";
static char calibrating_msg[] = "calibrating ";
static char green_torque_msg[] = "green";
static char yellow_torque_msg[] = "yellow";
static char red_torque_msg[] = "red";
*/

//flags
volatile bool new_plan = false;
volatile bool allowed_to_move_positive = false;
volatile bool allowed_to_move_negative = false;

//debug messages
/*static char miscMsgs1[50];
static char miscMsgs2[50];
static char miscMsgs3[50];
static char miscMsgs4[50];
static char miscMsgs5[50];
*/
static char inbound_message[400];

//internal position and direction variables
static volatile long encoder_1_pos = 0;  //position of doe in encoder clicks
static volatile long encoder_2_pos = 0;  //position of stepper doe in encoder clicks
static volatile long stepper_counts = 0; //stepper position in stepper steps
static volatile bool encoder_direction_reversed = false;
static volatile bool series_elastic_direction_reversed = false;
static volatile bool stepper_direction_reversed = false;

//robot configuration constants
const bool BR_ENCODER_DIRECTION_REVERSED = false;
const bool SL_ENCODER_DIRECTION_REVERSED = false;
const bool UR_ENCODER_DIRECTION_REVERSED = true;
const bool EL_ENCODER_DIRECTION_REVERSED = false;
const bool LR_ENCODER_DIRECTION_REVERSED = false;
const bool BR_SE_DIRECTION_REVERSED = false;
const bool SL_SE_DIRECTION_REVERSED = false;
const bool UR_SE_DIRECTION_REVERSED = true;//changed to true 9/27/19
const bool EL_SE_DIRECTION_REVERSED = false;
const bool LR_SE_DIRECTION_REVERSED = false;
const bool BR_STEPPER_DIRECTION_REVERSED = true;
const bool SL_STEPPER_DIRECTION_REVERSED = true;
const bool UR_STEPPER_DIRECTION_REVERSED = true;//changed to true 9/27/19
const bool EL_STEPPER_DIRECTION_REVERSED = false;
const bool LR_STEPPER_DIRECTION_REVERSED = true;

//motion constants
static const float BR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.030000000; //July 2019 Encoder at joint  60:12 gears with 600ppr_quad/2400counts/rev
static const float SL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.0605769231; //Sept 2019 Encoder at joint 104:42 gears with 600ppr_quad/2400counts/rev
static const float UR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.050000000; //July 2019 Encoder at joint  87:29 gears with 600ppr_quad/2400counts/rev
static const float EL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.0518691589;//Sept 2019 Encoder at joint 107:37 gears with 600ppr_quad/2400counts/rev
static const float LR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.057142857; //July 2019 Encoder at joint  84:32 gears with 600ppr_quad/2400counts/rev
//static const float WL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.010141226; //July 2019 Encoder at joint  65:15 gears with 2048ppr_quad/8196counts/rev
//static const float WR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.010986328; //July 2019 Encoder at joint  64:16 gears with 2048ppr_quad/8196counts/rev

static const float BR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
static const float SL_SE_DEGREES_PER_ENCODER_COUNT = 0.014081124; //July 2019
static const float UR_SE_DEGREES_PER_ENCODER_COUNT = 0.018083128; //July 2019
static const float EL_SE_DEGREES_PER_ENCODER_COUNT = 0.0163024023; //Nov 2019 new 3P3G gear ratio, corrected pulley ratio to 2.00:2.00
static const float LR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
//static const float WL_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
//static const float WR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed

static const float BR_DEGREES_PER_MICROSTEP = 0.0150000000;  //15.000:1 gear ratio July 2019 arm configuration
static const float SL_DEGREES_PER_MICROSTEP = 0.0035202811;  //63.915:1 gear ratio July 2019 arm configuration
static const float UR_DEGREES_PER_MICROSTEP = 0.0045207820;  //49.770:1 gear ratio July 2019 arm configuration
static const float EL_DEGREES_PER_MICROSTEP = 0.0040756006;  //69.932:1 new 3P3G gear ratio Nov 2019 arm configuration, corrected pulley ratio to 2.00:2.00
static const float LR_DEGREES_PER_MICROSTEP = 0.0134927588;  //16.675:1 gear ratio July 2019 arm configuration
//static const float WL_DEGREES_PER_MICROSTEP = 0.0;  //xl430, not stepper, July 2019 arm configuration
//static const float WR_DEGREES_PER_MICROSTEP = 0.0;  //xl430, not stepper, July 2019 arm configuration

//motion variables
static volatile float required_duration;                //required duration in seconds for joint movement
static volatile float commanded_stepper_position = 0.0; //joint commanded_stepper_position in degrees
static volatile float commanded_incremental_movement = 0.0; //joint commanded_incremental_movement in degrees
static volatile bool  direction_positive = true;        //movement in positive joint direction is defined as CW
static volatile float current_pos = 0.0;                //joint position in degrees
static volatile float pos_error = 0.0;                  //difference in degrees between joint commanded_stepper_position and joint current position
static volatile long  steps_remaining = 0;              //stepper steps remaining to move. uses in final_approach and holding_position
static volatile long  steps_to_move = 0;                //for non-blocking motion algorithm
static volatile float joint_degrees_per_encoder_count;  //reset to particular joint in setup()
static volatile float SE_degrees_per_encoder_count;     //reset to particular joint in setup()
static volatile float degrees_per_microstep;            //reset to particular joint in setup()
static volatile long  peak_microsteps_per_second=0L;    //for non-blocking motion algorithm
static volatile long  current_microsteps_per_second=0L; //for non-blocking motion algorithm
static volatile long  change_in_microsteps_per_second_each_rate_update=0L;  //change occuring in one RATE_UPDATE_INTERVAL
static volatile long  current_microstep_period=0L;      //in microseconds, for non-blocking motion algorithm
static volatile unsigned long required_duration_until_peak_in_microseconds = 0L;//in milliseconds, for non-blocking motion algorithm
static volatile unsigned long movement_start_time;      //from millis() for non-blocking motion algorithm
static volatile unsigned long movement_end_time;        //from millis() for non-blocking motion algorithm. used to assure movement stops
static volatile float joint_velocity;                   //deg/sec     
static volatile float joint_acceleration;               //deg/sec/sec
static volatile float motor_velocity;                   //deg/sec
static volatile float motor_acceleration;               //deg/sec/sec
static volatile float previous_joint_position;          //degrees
static volatile float previous_joint_velocity;          //deg/sec
static volatile float previous_motor_position;          //degrees
static volatile float previous_motor_velocity;          //deg/sec

//loop timing variables
static volatile unsigned long next_publish_update     = 0L;   //used to time loop publish updates
static const unsigned long    PUBLISH_UPDATE_INTERVAL = 200L; //was 500L before 1/4/2020, in milliseconds, 200 is 5Hz, 500 is 2Hz, 50 is 20Hz, 10 is 100Hz
static const float            PUBLISH_UPDATE_INTERVAL_IN_SECONDS = (float) PUBLISH_UPDATE_INTERVAL / 1000.0;
static volatile unsigned long next_rate_update        = 0L;   //used to time loop publish updates
static const unsigned long    RATE_UPDATE_INTERVAL    = 50000L; //in microseconds, 500000 is 2Hz, 50000 is 20Hz, 100000 is 10Hz
static const unsigned long    SLOW_RATE_UPDATE_INTERVAL = 500000L; //in microseconds, 500000 is 2Hz, 50000 is 20Hz, 100000 is 10Hz
static volatile unsigned long update_interval;                //represents either RATE_UPDATE_INTERVAL (normal moves) or SLOW_RATE_UPDATE_INTERVAL (slow or short moves)
static bool                   slow_movement           = false;//determines whether RATE_UPDATE_INTERVAL or SLOW_RATE_UPDATE_INTERVAL is in use
static volatile unsigned long time_now;                       //used to store value from millis()
static const unsigned long    COMM_LOSS_TIMEOUT       = 2000L;//time interval in milliseconds to initiate lost comm shutdown if master is not heard from. 2000L = 2.0 seconds
static volatile unsigned long comm_loss_time_limit    = 0L;   //time at which to initiate lost comm shutdown
static volatile int           override_command_received = 0;  //stores override commands issued from master

//variables to be published
my_robotic_arm::MinionState minion_state;
std_msgs::String state; //may be in multiple states simultaneously
std_msgs::String state2; //may be in multiple states simultaneously
//trajectory_msgs::JointTrajectoryPoint jtp;

//endstop pins
const int POSITIVE_ENDSTOP_PIN = 2;  //pin # corrected 12/6/17
const int NEGATIVE_ENDSTOP_PIN = 3; //pin # corrected 12/6/17

//encoder pins
const int ENCODER_1_PIN_A = 20;
const int ENCODER_1_PIN_B = 21;
const int ENCODER_2_PIN_A = 18; //was pin 20 pre-June2019, changed to use I2C on pins 20/21 for 3-Axis accelerometer orientation
const int ENCODER_2_PIN_B = 19; //was pin 21 pre-June2019, changed to use I2C on pins 20/21 for 3-Axis accelerometer orientation

//stepper pins
const int STEP_PIN = A0;
const int DIR_PIN = A1;
const int ENABLE_PIN = A2;//note this enables both stepper 1 and stepper 2 on a joint
const int STEP2_PIN = A3; //for second stepper motor on same joint e.g. SL
const int DIR2_PIN = A4;  //Note DIR2 pin for SL must always be set opposite DIR_PIN as steppers motors installed opposite each other

//minion identification pins
const int IDENT_PIN_1 = 36;  //correct for new minion shield  V0.1 APr2016
const int IDENT_PIN_2 = 38;
const int IDENT_PIN_4 = 40;
const int IDENT_PIN_8 = 42;
int minion_ident = 0;//stores minion identification number
const int BR_IDENT = 14;
const int SL_IDENT = 13;
const int UR_IDENT = 12;
const int EL_IDENT = 11;
const int LR_IDENT = 10;
const int WL_IDENT = 9;
const int WR_IDENT = 8;
const int EE1_IDENT = 7;
const int EE2_IDENT = 6;

//ROS node handler
ros::NodeHandle_<ArduinoHardware, 2, 6, 512, 2560> nh;    //increased 04July2019

/* service call backs - currently unused
//void getUpperarmOrientationCallback(const my_robotic_arm::GetUpperarmOrientation::Request  &req, my_robotic_arm::GetUpperarmOrientation::Response &res){  
//  nh.loginfo("entering getOrientationCallback");
//  nh.spinOnce();
//  
//  Adafruit_MMA8451 mma = Adafruit_MMA8451();//defined here within function so it goes out of scope and stops at end of function
//
//  float _X = 0.0;
//  float _Y = 0.0;
//  float _Z = 0.0;
//  int num_points_to_smooth = 200; //was 200 pre-04Jul19
//  
//  //loop to start up MMA8451
//  unsigned long _MMA8451_start_time = millis();
//  unsigned long _MMA8451_end_time = _MMA8451_start_time + 3000L;
//  if (! mma.begin()) {
//    if ( millis() > _MMA8451_end_time ){
//      nh.loginfo("unable to start MMA8451");
//    }
//  }
//  
//  //init MMA8451
//  mma.setRange(MMA8451_RANGE_2_G);
//
//  //loop to read and smooth data
//  for (int i=0; i<num_points_to_smooth; i++){
//    //Get a new MMA8451 sensor event to read MMA8451
//    sensors_event_t event; 
//    mma.getEvent(&event);
//
//    //add porportion of new reading to previous cum
//    _X += event.acceleration.x / num_points_to_smooth; //note a float divided by and int should result in a float
//    _Y += event.acceleration.y / num_points_to_smooth; //note a float divided by and int should result in a float
//    _Z += event.acceleration.z / num_points_to_smooth; //note a float divided by and int should result in a float
//
//    nh.spinOnce();
//  }//end loop to read and smooth data
//
//  //put event readings in response res
//  res.upperarm_orientation.x = _X;
//  res.upperarm_orientation.y = _Y;
//  res.upperarm_orientation.z = _Z;
//
//  nh.spinOnce();
//
//  nh.loginfo("exiting getOrientationCallback");
//  nh.spinOnce();
//
//}// end getUpperarmOrientationCallback()
//
////service call backs
//void getLowerarmOrientationCallback(const my_robotic_arm::GetLowerarmOrientation::Request  &req, my_robotic_arm::GetLowerarmOrientation::Response &res){  
//  nh.loginfo("entering getOrientationCallback");
//  nh.spinOnce();
//
//  Adafruit_MMA8451 mma = Adafruit_MMA8451();//defined here within function so it goes out of scope and stops at end of function
//
//  float _X = 0.0;
//  float _Y = 0.0;
//  float _Z = 0.0;
//  int num_points_to_smooth = 200;
//  
//  //loop to start up MMA8451
//  unsigned long _MMA8451_start_time = millis();
//  unsigned long _MMA8451_end_time = _MMA8451_start_time + 3000L;
//  if (! mma.begin()) {
//    if ( millis() > _MMA8451_end_time ){
//      nh.loginfo("unable to start MMA8451");
//    }
//  }
//  
//  //init MMA8451
//  mma.setRange(MMA8451_RANGE_2_G);
//
//  //loop to read and smooth data
//  for (int i=0; i<num_points_to_smooth; i++){
//    //Get a new MMA8451 sensor event to read MMA8451
//    sensors_event_t event; 
//    mma.getEvent(&event);
//
//    //add porportion of new reading to previous cum
////    _X += event.acceleration.x / num_points_to_smooth; //note a float divided by and int should result in a float
////    _Y += event.acceleration.y / num_points_to_smooth; //note a float divided by and int should result in a float
////    _Z += event.acceleration.z / num_points_to_smooth; //note a float divided by and int should result in a float
//    _X += event.acceleration.x / 200.0; //note a float divided by and int should result in a float
//    _Y += event.acceleration.y / 200.0; //note a float divided by and int should result in a float
//    _Z += event.acceleration.z / 200.0; //note a float divided by and int should result in a float
//
//    nh.spinOnce();
//  }//end loop to read and smooth data
//
//  //put event readings in response res
//  res.lowerarm_orientation.x = _X;
//  res.lowerarm_orientation.y = _Y;
//  res.lowerarm_orientation.z = _Z;  
//
//  nh.spinOnce();
//
//  nh.loginfo("exiting getOrientationCallback");
//  nh.spinOnce();
//     
//}// end getLowerarmOrientationCallback()
 */

//subscriber call backs
void commandedIncrementalMotorMovementCallback(const std_msgs::String& the_command_msg_) {
  /*  action to be taken when msg received on topic commanded_incremental_motor_movement.
    First token in command message is a message number (int) to determine if command is duplicate that already been processed
    Second token is required duration of movement in (float) seconds
    Subsequent tokens represent commanded position in (float) degrees for each joint
      in order BR, SL, UR, EL, LR, WL, WR
  */
  int current_command_number_ = 0;
  static int previous_command_number_ = 0;//unique integer identify this command
  float commanded_incremental_movement_[7];//array holding req'd duration and commanded position in degrees in order: BR,SL,UR,EL,LR, WL

  //parse command
  inbound_message[0] = (char)0;  //"empties inbound_message by setting first char in string to 0
  strcat(inbound_message, the_command_msg_.data);//this finally works

  //debug only
  nh.loginfo("InbdMsg=");
  nh.loginfo(inbound_message);//this now works
  nh.spinOnce();

  //process first token in inbound_message which is unique command id number
  //http://jhaskellsblog.blogspot.com/2011/06/parsing-quick-guide-to-strtok-there-are.html
  //this whole test section works and returns torque of 5.230000
  //char test_message[] = "6.23,4.56,7.89";
  char* command;
  //  command = strtok(test_message, ",");//works
  command = strtok(inbound_message, ",");//works
  commanded_incremental_movement_[0] = atoi(command);//retrieves unique command message number

  //check this is not a repeated command
  current_command_number_ = commanded_incremental_movement_[0];
  if (current_command_number_ != previous_command_number_) {
    //this is a new command. Plan and execute motion
    //set previous commanded position
    previous_command_number_ = current_command_number_;
    //set flags
    new_plan = true;//new_plan flag is polled in loop() to initiate a new plan and motion
    operating_state = NORMAL;//a valid new plan will cancel any endstop, lost comm, or emrg stop conditions
  }
  else {
    //this is a duplicate plan, exit
    return;
  }

  //process remaining tokens in inbound_message
  int joint_index = 1;//note starts at 1 because required duration has already been parsed as joint_index=0
  while (command != 0)
  {
    command = strtok(0, ",");//note "0" i.e. null as input for subsequent calls to strtok()
    commanded_incremental_movement_[joint_index] = atof(command);
    joint_index++;
  }

  //apply to only this joint
  switch (minion_ident) {
    case BR_IDENT:
      joint_index = 2;
      break;
    case SL_IDENT:
      joint_index = 3;
      break;
    case UR_IDENT:
      joint_index = 4;
      break;
    case EL_IDENT:
      joint_index = 5;
      break;
    case LR_IDENT:
      joint_index = 6;
      break;
    case WL_IDENT:
      joint_index = 7;
    break;    default:
      break;
  }//end switch case

  //assign required duration and commanded_incremental_movement
  required_duration = commanded_incremental_movement_[1];
  commanded_incremental_movement = commanded_incremental_movement_[joint_index];

  //check for Command 1000 to set joint position variables
  if (commanded_incremental_movement > 500.0 && commanded_incremental_movement < 1500.0) {
    //Command 1000 - set joint position values to zero
    float new_joint_pos_;
    new_joint_pos_ = commanded_incremental_movement - 1000.0;
    
    if (minion_ident == LR_IDENT){
      //special cse July2019 for LR wiring which retains 3D accelerometer on I2C pins 20&21
      encoder_2_pos = (long) (new_joint_pos_ / joint_degrees_per_encoder_count);
    }
    else {
      encoder_1_pos = (long) (new_joint_pos_ / joint_degrees_per_encoder_count);
    }
    
    current_pos = new_joint_pos_;
    
    steps_remaining = 0;
    commanded_incremental_movement = 0.0;
    new_plan = false;

    //log
    nh.loginfo("cmd 1000: reset joint position variables");

    //exit without beginning any movement
    return;
  }// end of command 1000

  //check for Command 2000 to set SE position variables
  if (commanded_incremental_movement > 1500.0 && commanded_incremental_movement < 2500.0) {
    //Command 2000 - set SE position values to zero
    float new_SE_pos_;
    new_SE_pos_ = commanded_incremental_movement - 2000.0;
    
    if (minion_ident == LR_IDENT){
      //normally not used - no action taken as LR does not have SE encoder installed as of July 2019
    }
    else if (minion_ident == BR_IDENT){
      //normally not used - no action taken as BR does not have SE encoder installed as of July 2019
    }
    else {
      encoder_2_pos = (long) (new_SE_pos_ / SE_degrees_per_encoder_count);
    }

    //correct sign of encoder_2_pos if series_elastic_direction_reversed. Revised 10/4/19
    if (series_elastic_direction_reversed){
      encoder_2_pos = -encoder_2_pos;
    }
    
    steps_remaining = 0;
    commanded_incremental_movement = 0.0;
    new_plan = false;

    //log
    nh.loginfo("cmd 2000: reset SE position variables");

    //exit without beginning any movement
    return;
  }// end of command 2000
 
  //check for Command 3000 to set stepper position variables
  if (commanded_incremental_movement > 2500.0 && commanded_incremental_movement < 3500.0) {
    //Command 3000 - set stepper position values to zero
    float new_stepper_pos_;
    new_stepper_pos_ = commanded_incremental_movement - 3000.0;
    
    stepper_counts = (long) (new_stepper_pos_ / degrees_per_microstep);
    
    steps_remaining = 0;
    commanded_incremental_movement = 0.0;
    new_plan = false;

    //log
    nh.loginfo("cmd 3000: reset stepper position variables");

    //exit without beginning any movement
    return;
  }// end of command 3000
  
  //debug loginfo required_duration
  char result5[8]; // Buffer big enough for 7-character float
  dtostrf(required_duration, 6, 2, result5); // Leave room for too large numbers!
  nh.loginfo("required_duration=");//this works
  nh.loginfo(result5);//this works

  //debug loginfo commanded_incremental_movement
  char result[8]; // Buffer big enough for 7-character float
  dtostrf(commanded_incremental_movement, 6, 2, result); // Leave room for too large numbers!
  nh.loginfo("commanded_incremental_movement=");//this works
  nh.loginfo(result);//this works
}// end commandedIncrementalMovementCallback()
void overrideCommandReceivedCallback(const std_msgs::Int16& the_override_command_msg_){
  //action taken when message rec'd on rostopic: minion_override_command
  override_command_received = the_override_command_msg_.data;
  
  //renew time limit in which next override must be received in order to forestall lost comm timeout
  comm_loss_time_limit = millis() + COMM_LOSS_TIMEOUT;

  //handle any commands for non-normal operation
  if (override_command_received < 0){
    //negative values of override_command_ indicate a non-normal command has been received from the master
    switch (override_command_received) {
      case -999:
        //emerg stop command has been issued by master
        allowed_to_move_positive = false;
        allowed_to_move_negative = false;
        operating_state = EMERG_STOPPED_BY_MASTER;
        movement_end_time = millis();//this will stop movement clock
        break;
    }
  }
  
}// end overrideCommandReceivedCallback()

//subscribers
ros::Subscriber<std_msgs::String> commanded_incremental_motor_movement_sub("commanded_incremental_motor_movement", commandedIncrementalMotorMovementCallback);
ros::Subscriber<std_msgs::Int16> override_commands_sub("minion_override_command", overrideCommandReceivedCallback);

//publishers - need to create all possible publishers, but will only advertise ones called for by minion_ident
ros::Publisher BR_minion_state_pub("BR_minion_state", &minion_state);     //BR minion state
ros::Publisher SL_minion_state_pub("SL_minion_state", &minion_state);     //SL minion state
ros::Publisher UR_minion_state_pub("UR_minion_state", &minion_state);     //UR minion state
ros::Publisher EL_minion_state_pub("EL_minion_state", &minion_state);     //EL minion state
ros::Publisher LR_minion_state_pub("LR_minion_state", &minion_state);     //LR minion state

void doEncoder1A() {
  /*  encoder rotation causes encoder pins to change state. this monitors sequence of
      pins changes to determine direction and counts pulses to determine position change.
      A Series-Elastic joint and gear reduction connect the stepper motor to the encoder.
      THe difference between the encoder position and the stepper position is proportional
      to the torque across the Series-Elastic joint. Integer math can be used for torque
      measurement in real-time by selecting a gear reduction ratio to be an integer multiple
      of the ratio between encoder counts per revolution and stepper counts per revolution.
      At each encoder pulse, the encoder count is incremented by this integer multiple, thus
      at any time, the stepper position and the encoder position can be directly compared to determine torque.
      For example, in this incarnation:
      Stepper steps per revolution: 200
      ENcoder counts per revolution: 360
      Ration encoder counts to stepper steps: 360/200 = 1.8:1
      Planetary gear reduction ratio: 3.6:1
      Integer multiple: 3.6/1.8 = 2
      Increment/decrement encoder count by integer multiple of 2 at each encoder pulse
  */
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_1_PIN_A) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_B) == LOW) {
      //      encoder_1_pos += 1;         // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;         // CCW
      encoder_1_pos += 1;         // CW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_B) == HIGH) {
      //      encoder_1_pos += 1;          // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;          // CCW
      encoder_1_pos += 1;         // CW
    }
  }
} //end doEncoder1A()

void doEncoder1B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_1_PIN_B) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_A) == HIGH) {
      //      encoder_1_pos += 1;         // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;         // CCW
      encoder_1_pos += 1;         // CW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_1_PIN_A) == LOW) {
      //      encoder_1_pos += 1;          // CW
      encoder_1_pos -= 1;          // CCW
    }
    else {
      //      encoder_1_pos -= 1;          // CCW
      encoder_1_pos += 1;         // CW
    }
  }
} //end doEncoder1B()

void doEncoder2A() {
  /*  encoder rotation causes encoder pins to change state. this monitors sequence of
      pins changes to determine direction and counts pulses to determine position change.
      A Series-Elastic joint and gear reduction connect the stepper motor to the encoder.
      THe difference between the encoder position and the stepper position is proportional
      to the torque across the Series-Elastic joint. Integer math can be used for torque
      measurement in real-time by selecting a gear reduction ratio to be an integer multiple
      of the ratio between encoder counts per revolution and stepper counts per revolution.
      At each encoder pulse, the encoder count is incremented by this integer multiple, thus
      at any time, the stepper position and the encoder position can be directly compared to determine torque.
      For example, in this incarnation:
      Stepper steps per revolution: 200
      ENcoder counts per revolution: 360
      Ration encoder counts to stepper steps: 360/200 = 1.8:1
      Planetary gear reduction ratio: 3.6:1
      Integer multiple: 3.6/1.8 = 2
      Increment/decrement encoder count by integer multiple of 2 at each encoder pulse
  */
  // look for a low-to-high on channel A
  if (digitalRead(ENCODER_2_PIN_A) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_B) == LOW) {
      //      encoder_2_pos += 1;         // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;         // CCW
      encoder_2_pos += 1;         // CW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_B) == HIGH) {
      //      encoder_2_pos += 1;          // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;          // CCW
      encoder_2_pos += 1;         // CW
    }
  }
} //end doEncoder2A()

void doEncoder2B() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_2_PIN_B) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_A) == HIGH) {
      //      encoder_2_pos += 1;         // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;         // CCW
      encoder_2_pos += 1;         // CW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_2_PIN_A) == LOW) {
      //      encoder_2_pos += 1;          // CW
      encoder_2_pos -= 1;          // CCW
    }
    else {
      //      encoder_2_pos -= 1;          // CCW
      encoder_2_pos += 1;         // CW
    }
  }
} //end doEncoder2B()

void pulseStepper() {
  //pulses stepper during normal motion. called by stepOnce().
  digitalWrite(STEP_PIN, HIGH);
  digitalWrite(STEP2_PIN, HIGH);
  //    delayMicroseconds(5);  //increase this if pulses too fast for accurate step or jitters
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);
  //    delayMicroseconds(5);  //increase this if pulses too fast for accurate step or jitters
  if (direction_positive) {
    stepper_counts++;
  }
  else {
    stepper_counts--;
  }
} // end pulseStepper()

void stepOnce() {
  /* called periodically by Timer3
     determines, based on state, whether or not to pulse stepper, and how fast to pulse
  */
  //  nh.loginfo("stepOnce()");
  //  nh.spinOnce();

  //check for endstop activation - note endstop pins are set as INPUT_PULLUP so a LOW means switch has been activated
  if (digitalRead(POSITIVE_ENDSTOP_PIN) == LOW) {
    if (direction_positive) {
      nh.loginfo("Positive endstop + dir positive, stop!");
      //WORKING HERE 12/6/17
      //To Do - NEED TO CHECK DIRECTIONS ARE CORRECT
      operating_state = POSITIVE_ENDSTOP_ACTIVATED;
      allowed_to_move_positive = false;
      allowed_to_move_negative = true;
      movement_end_time = millis();//this will stop movement clock
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      //this else case should never happen if endstops are wired correctly
      nh.loginfo("Positive endstop + dir Negative");
      operating_state = POSITIVE_ENDSTOP_ACTIVATED;
      allowed_to_move_positive = false;
      allowed_to_move_negative = true;
      movement_end_time = millis();//this will stop movement clock
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }
  if (digitalRead(NEGATIVE_ENDSTOP_PIN) == LOW) {
    if (!direction_positive) {
      nh.loginfo("Negative endstop + dir Negative, stop!");
      operating_state = NEGATIVE_ENDSTOP_ACTIVATED;
      allowed_to_move_positive = true;
      allowed_to_move_negative = false;
      movement_end_time = millis();//this will stop movement clock
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      //this else case should never happen if endstops are wired correctly
      nh.loginfo("Negative endstop + dir Positive");
      operating_state = NEGATIVE_ENDSTOP_ACTIVATED;
      allowed_to_move_positive = true;
      allowed_to_move_negative = false;
      movement_end_time = millis();//this will stop movement clock
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }

  //pulse stepper if normal operation
  if (operating_state == NORMAL){
      //step and decrement
      pulseStepper();
      steps_remaining--;
  }

  /*switch (operating_state) {
    case UNPOWERED:
      //      nh.loginfo("UNP");
      break;
    case HOLDING_POSITION:
      //      nh.loginfo("HOLDING_POSITION");
      break;
    case MOVING:
      //      nh.loginfo("MOVING");
      //step and decrement
      pulseStepper();
      steps_remaining--;
      break;
    case EMERG_STOPPED:
      break;
    case CW_ENDSTOP_ACTIVATED:
      break;
    case CCW_ENDSTOP_ACTIVATED:
      break;
    case CALIBRATING:
      break;
    default:
      break;
  }//end switch operating_state
  */
}// end stepOnce()

long getPeakStepperVelocityNeeded(float the_required_total_duration_, float the_commanded_incremental_movement_){
  /* computes velocity needed to accel and decel stepper with RAMP profile to achieve desired incremental joint movement
   * initially assumes linear accel/decel profile
   * peak_velocity_ is used as input to myRamp.go() as target value to which to accelerate
   * Inputs:
   *  the_required_total_duration_ :: duration in float seconds. used as input to myRamp.go() after being converted to long or whichever units i instatiated myRamp as
   *  the_commanded_incremental_movement_ :: joint movement in degrees
   * Returns:
   *  peak_microsteps_per_second_ :: peak stepper velocity in stepper microsteps per second as long
   */

  //compute peak joint velocity in joint degrees per second assuming linear interpolation mode 
  //float average_joint_velocity_ = the_commanded_incremental_movement_ / the_required_total_duration_;
  //float peak_joint_velocity_ = avgerage_joint_velocity_ * 2.0;
  float peak_joint_velocity_ = 2.0 * (the_commanded_incremental_movement_ / the_required_total_duration_);//

  //convert peak joint velocity in joint degrees per second to peak stepper velocity in stepper microsteps per second
  long peak_microsteps_per_second_ = (long) abs(peak_joint_velocity_ / degrees_per_microstep);
  
  //KLUDGE FUDGE TO MAKE RAMP WORK CORRECTLY - TO DO FIX THIS!
//  peak_microsteps_per_second_ = 2 * peak_microsteps_per_second_;
  
  return peak_microsteps_per_second_;
}// end getPeakStepperVelocityNeeded()

void updateRate() {
  /*  changes stepper velocity by updating rate of pulses
   *  updates global variable current_microsteps_per_second to follow acceleration/deceleration curve
   *  called by Timer1.attachInterrupt(updateRate) at an interval of RATE_UPDATE_INTERVAL
   *  assumes a constant interval bewteen rate updates which is specified by RATE_UPDATE_INTERVAL
   *  Turns interrupts off then back on again per this: https://www.pjrc.com/teensy/td_libs_TimerOne.html
   */
//  nh.loginfo("enter updateRate()");
   
  long copy_steps_to_move = 0L;
  long copy_steps_remaining = 0L;
  long copy_current_microsteps_per_second = 0L;
  long copy_current_microstep_period = 0L;
  long copy_change_in_microsteps_per_second_each_rate_update = 0L;

  //stop interrupts to make a quick copy of globals, then restart -  Ref: https://www.pjrc.com/teensy/td_libs_TimerOne.html
  noInterrupts();
  copy_steps_to_move = steps_to_move;
  copy_steps_remaining = steps_remaining;
  copy_current_microsteps_per_second = current_microsteps_per_second;
  copy_change_in_microsteps_per_second_each_rate_update = change_in_microsteps_per_second_each_rate_update;
  interrupts();

  //calculate new stepper velocity
  if (copy_steps_remaining > (copy_steps_to_move / 2)) {
    //not yet at half way point - still accelerating
    copy_current_microsteps_per_second += copy_change_in_microsteps_per_second_each_rate_update;
  }
  else {
    //decelerating
    copy_current_microsteps_per_second -= copy_change_in_microsteps_per_second_each_rate_update;    
  }

  //calculate the microstep period and set Timer3 to new rate
  if ( copy_current_microsteps_per_second > 0 ) {
    //normal case
    copy_current_microstep_period = 1000000L / copy_current_microsteps_per_second;
  }
  else {
    //microsteps per second is zero or negative - avoid divide by zero problem, and prevent negative answer
    copy_current_microsteps_per_second = 0;
    copy_current_microstep_period = 2000000000L;  //set to near largest value of long. This will fire stepper every ~33 mintues
  }
  
  Timer3.setPeriod(copy_current_microstep_period);

  //stop interrupt to write new values to globals, then restart interrupts
  noInterrupts();
  current_microstep_period = copy_current_microstep_period;  //i don't think is used for anything outside this function?
  current_microsteps_per_second = copy_current_microsteps_per_second; //this IS used
  interrupts();

  /* debug messages
  sprintf(miscMsgs4, "new copy_current_microsteps_per_second is %ld", copy_current_microsteps_per_second);
  nh.loginfo(miscMsgs4);
  nh.spinOnce();
  sprintf(miscMsgs5, "new copy_current_microstep_period is %ld", copy_current_microstep_period);
  nh.loginfo(miscMsgs5);
  nh.spinOnce();
   */
   
} // end updateRate()

//NON-BLOCKING VERSION
void executeIncrementalMovement(float the_required_total_duration_, float the_commanded_incremental_movement_) {
  /* Result: pulses stepper a precise number of times to acheive commanded degrees of movement at joint
   * Note: This is the NON-BLOCKING method. ros pub stuff WILL operate while movement loop is in progress
   * Does NOT use Arduino RAMP library
   * Called by:
   *  if (new_plan) in loop()
   * Inputs:
   *  the_required_total_duration_ :: duration of complete movement in seconds
   *  the_commanded_incremental_movement_ :: commanded joint movement in degrees
   * Actions:
   *  calculates and sets global parameters
   *  restarts Timer1 and Timer3
   *  sets a millis() stop time to be monitored in loop() to stop timers and steppers
   * Note:
   *  For legacy reasons, the_required_total_duration_ is in float seconds, while rampLong myRamp' needs duration as milliseconds as an unsigned long  
   */
//  nh.loginfo("entering executeIncrementalMovement() - non-blocking version");
//  nh.spinOnce();

  //get steps to move and steps remaining
  steps_to_move = (long) abs(the_commanded_incremental_movement_ / degrees_per_microstep); //joint distance to travel in steps in absolute value as long - convert dtg_ in degrees to dtg_stepper_counts_ as a long
  steps_remaining = steps_to_move;
  
  //get peak_microsteps_per_second
  peak_microsteps_per_second = getPeakStepperVelocityNeeded(the_required_total_duration_, the_commanded_incremental_movement_);

  //initialize stepper velocity to zero
  current_microsteps_per_second = 0;
  current_microstep_period = 5000000; //set a 5 second period just to get things started

  //convert required duration in float seconds to required_duration_until_peak_in_microseconds in unsigned long. Notice this is only half the total duration as velocity ramps up to peak then down to zero
  required_duration_until_peak_in_microseconds = (unsigned long) abs(the_required_total_duration_ * 1000000.0 / 2.0);

  //calculate the rate of change in stepper velocity required in one RATE_UPDATE_INTERVAL
  long num_rate_updates_to_peak = required_duration_until_peak_in_microseconds / RATE_UPDATE_INTERVAL;
  change_in_microsteps_per_second_each_rate_update = (long) (peak_microsteps_per_second / num_rate_updates_to_peak);

  //decide of movement is normal or slow/short - added 9/25/19
  if (change_in_microsteps_per_second_each_rate_update > 4) {
    //this is a normal move
    update_interval = RATE_UPDATE_INTERVAL;
    Timer1.setPeriod(update_interval);
    slow_movement = false;
  }
  else {
    /* this is a slow or short move wherein small values of change_in_microsteps_per_second_each_rate_update 
     *  will be inaccurate because division of longs will truncate decimal portion. Particularly egregious when
     *  value is lees than one which truncates to zero. E.g. 0.662 truncates to zero */
    update_interval = SLOW_RATE_UPDATE_INTERVAL;
    Timer1.setPeriod(update_interval);    
    //recalculate for 10x slower updates
    change_in_microsteps_per_second_each_rate_update = (long) (10 * peak_microsteps_per_second / num_rate_updates_to_peak);
    slow_movement = true;
  }
  
  /* debug messages
  sprintf(miscMsgs1, "peak_microsteps_per_second %ld", peak_microsteps_per_second);
  nh.loginfo(miscMsgs1);
  nh.spinOnce();
  sprintf(miscMsgs2, "num_rate_updates_to_peak %ld", num_rate_updates_to_peak);
  nh.loginfo(miscMsgs2);
  nh.spinOnce();
  sprintf(miscMsgs3, "change_in_microsteps_per_second_each_rate_update %ld", change_in_microsteps_per_second_each_rate_update);
  nh.loginfo(miscMsgs3);
  nh.spinOnce();
  sprintf(miscMsgs1, (slow_movement ? "true" : "false"));
  nh.loginfo(miscMsgs1);
  nh.spinOnce();
   */
   
  //get direction of motion. positive distance defined as CCW, negative as CW
  if (the_commanded_incremental_movement_ > 0) {
      direction_positive = true;
//      nh.loginfo("in executeIncrementalMovement(), dir is CW");
//      nh.spinOnce();
      if (stepper_direction_reversed == true){
          digitalWrite(DIR_PIN, HIGH);
          digitalWrite(DIR2_PIN, LOW);
      }
      else{
          //this is reversed stepper direction
          digitalWrite(DIR_PIN, LOW);
          digitalWrite(DIR2_PIN, HIGH);        
      }
  } 
  else {
      direction_positive = false;
//      nh.loginfo("in executeIncrementalMovement(), dir is CCW");
//      nh.spinOnce();
      if (stepper_direction_reversed == true){
          digitalWrite(DIR_PIN, LOW);      
          digitalWrite(DIR2_PIN, HIGH);
      }
      else {
          //this is reversed stepper direction
          digitalWrite(DIR_PIN, HIGH);      
          digitalWrite(DIR2_PIN, LOW);
      }
  }

  //restart Timer1 and Timer3
  Timer3.setPeriod(current_microstep_period);
  Timer1.restart();
  Timer3.restart();

  movement_start_time = millis();
  movement_end_time = movement_start_time + 1000L + (long) 2 * the_required_total_duration_ * 1000L; //ends after duration plus 1 second (1000 milliseconds)

  /* debug messages
  sprintf(miscMsgs2, "change_in_microsteps_per_second_each_rate_update %ld", change_in_microsteps_per_second_each_rate_update);
  nh.loginfo(miscMsgs2);
  nh.spinOnce();
  nh.loginfo("exiting executeIncrementalMovement() - non-blocking version");
  nh.spinOnce();
   */

}// end executeIncrementalMovement() - NON-BLOCKING version

/*BLOCKING VERSION
void executeIncrementalMovement(float the_required_total_duration_, float the_commanded_incremental_movement_) {
  // Result: pulses stepper a precise number of times to acheive commanded degrees of movement at joint
  // Note: This is a blocking method. ros pub stuff will not operate while movement loop is in progress
  // Called by:
  //  if (new_plan) in loop()
  // Inputs:
  //  the_required_total_duration_ :: duration of complete movement in seconds
  //  the_commanded_incremental_movement_ :: commanded joint movement in degrees
  // Note:
  //  For legacy reasons, the_required_total_duration_ is in float seconds, while rampLong myRamp' needs duration as milliseconds as an unsigned long  
  //
  nh.loginfo("entering executeIncrementalMovement()");
  nh.spinOnce();

  rampLong myRamp;      // new ramp object (ramp<unsigned char> by default if not otherwise spec'd)

  //volatile long current_microsteps_per_second_ = 0L;  //this line causes service callback getUpperarmOrientationCallback() to fail and lock up node running 100+%
  long current_microsteps_per_second_ = 0L;
  long current_microstep_period_ = 0L;  //period in microseconds
  unsigned long required_duration_until_peak_in_milliseconds_ = 0L;

  //get steps to move
  steps_remaining = (long) abs(the_commanded_incremental_movement_ / degrees_per_microstep); //joint distance to travel in steps in absolute value as long - convert dtg_ in degrees to dtg_stepper_counts_ as a long

  //get peak_microsteps_per_second_
  long peak_microsteps_per_second_ = getPeakStepperVelocityNeeded(the_required_total_duration_, the_commanded_incremental_movement_);

  //convert required duration in float seconds to required_duration_until_peak_in_milliseconds_ in unsigned long. Notice this is only half the total duration as velocity ramps up to peak then down to zero
  required_duration_until_peak_in_milliseconds_ = (unsigned long) abs(the_required_total_duration_ * 1000.0 / 2.0);
    
//    sprintf(miscMsgs9, "long peak_microsteps_per_second_= %ld", peak_microsteps_per_second_);
//    nh.loginfo(miscMsgs9);

  //get direction of motion. positive distance defined as CCW, negative as CW
  if (the_commanded_incremental_movement_ > 0) {
      direction_positive = true;
//      nh.loginfo("in executeIncrementalMovement(), dir is CW");
//      nh.spinOnce();
      if (stepper_direction_reversed == true){
          digitalWrite(DIR_PIN, HIGH);
          digitalWrite(DIR2_PIN, LOW);
      }
      else{
          //this is reversed stepper direction
          digitalWrite(DIR_PIN, LOW);
          digitalWrite(DIR2_PIN, HIGH);        
      }
  } 
  else {
      direction_positive = false;
//      nh.loginfo("in executeIncrementalMovement(), dir is CCW");
//      nh.spinOnce();
      if (stepper_direction_reversed == true){
          digitalWrite(DIR_PIN, LOW);      
          digitalWrite(DIR2_PIN, HIGH);
      }
      else {
          //this is reversed stepper direction
          digitalWrite(DIR_PIN, HIGH);      
          digitalWrite(DIR2_PIN, LOW);
      }
  }
  
  //init Timer3
  Timer3.initialize(5000000);  // initialize Timer3, and set a 5 second period
  Timer3.attachInterrupt(stepOnce);

//    nh.loginfo("in executeIncrementalMovement(), 101");
//    nh.spinOnce();

  //reset microsteps_per_seconds to zero as start point of ramp. In a more sophisticated movement, this could be set to current running velocity
  myRamp.go(0L);
  
  //execute movement
  myRamp.go(peak_microsteps_per_second_, required_duration_until_peak_in_milliseconds_, LINEAR, FORTHANDBACK); // start interpolation (value to go to, duration, ramp_mode, loop_mode)
//  myRamp.go(peak_microsteps_per_second_, required_duration_until_peak_in_milliseconds_, QUADRATIC_INOUT, FORTHANDBACK); // start interpolation (value to go to, duration, ramp_mode, loop_mode)
//  myRamp.go(peak_microsteps_per_second_, required_duration_until_peak_in_milliseconds_, CUBIC_INOUT, FORTHANDBACK); // start interpolation (value to go to, duration, ramp_mode, loop_mode)

//    nh.loginfo("in executeIncrementalMovement(), 102");
//    nh.spinOnce();

  unsigned long start_time = millis();
  unsigned long end_time = start_time + 1000L + (long) 2 * the_required_total_duration_ * 1000L; //ends after duration plus 1 second (1000 milliseconds)

//    sprintf(miscMsgs7, "long start_time=%ld", start_time);
//    nh.loginfo(miscMsgs7);
//    sprintf(miscMsgs8, "long end_time=%ld", end_time);
//    nh.loginfo(miscMsgs8);
//    nh.loginfo("in executeIncrementalMovement(), 103");
//    nh.spinOnce();

  while( (steps_remaining > 0) && (millis() < end_time) ){
    //get current microsteps per second
    current_microsteps_per_second_ = myRamp.update();

//      sprintf(miscMsgs10, "long current_microsteps_per_second_=%ld", current_microsteps_per_second_);
//      nh.loginfo(miscMsgs10);
//      sprintf(miscMsgs11, "long steps_remaining=%ld", steps_remaining);
//      nh.loginfo(miscMsgs11);
//      //nh.loginfo("in while millis < end_time, 201");
//      nh.spinOnce();

    //convert to a time interval between stepper pulses
    current_microstep_period_ = 1000000L / current_microsteps_per_second_;
    
    //set boundry for allowable current_microstep_period limits
    if (current_microstep_period_ < 1) { current_microstep_period_ = 1000000L;};
    
    //set time interval
    Timer3.setPeriod(current_microstep_period_);//this line works!

    //update minion_state message. Velocity is in degrees/second for now
    minion_state.motor_velocity = degrees_per_microstep * current_microsteps_per_second_;

    //this is needed to prevent 'lost sync' during long movements
    nh.spinOnce();

    delay(1);//delays one millisecond

//      sprintf(miscMsgs7, "millis()%ld", millis());
//      nh.loginfo(miscMsgs7);
//      sprintf(miscMsgs8, "long end_time=%ld", end_time);
//      nh.loginfo(miscMsgs8);
//      nh.loginfo("in while millis < end_time, 205");
//      nh.spinOnce();

  }// end while millis < end_time
  
  //stop Timer3 from firing pulses when movement is complete
  Timer3.stop();

  nh.loginfo("exiting executeIncrementalMovement(), 105");
  nh.spinOnce();

}// end executeIncrementalMovement() - blocking version
*/

void readSensors() {
  /*//report state as a char message - not used since Dec 2019
  strcpy(states_msg, empty_states_msg);      //overwrites previous states_msg with empty msg
  if (operating_state == UNPOWERED) strcat(states_msg, unpowered_msg);
  if (operating_state == HOLDING_POSITION) strcat(states_msg, holding_position_msg);
  if (operating_state == MOVING) strcat(states_msg, moving_msg);
  if (operating_state == FINAL_APPROACH) strcat(states_msg, final_approach_msg);
  if (operating_state == CW_ENDSTOP_ACTIVATED) strcat(states_msg, cw_endstop_activated_msg);
  if (operating_state == CCW_ENDSTOP_ACTIVATED) strcat(states_msg, ccw_endstop_activated_msg);
  if (operating_state == CALIBRATING) strcat(states_msg, calibrating_msg);
  if (torque_state == GREEN) strcat(states_msg, green_torque_msg);
  if (torque_state == YELLOW) strcat(states_msg, yellow_torque_msg);
  if (torque_state == RED) strcat(states_msg, red_torque_msg);
  */

  //calculate the minion_state.joint_position
  if (minion_ident == LR_IDENT){
    //special case 19July19 for 3d accelerometer still wired to pins 20&21 I2C. LR has no SE encoder installed.
    if (encoder_direction_reversed == true) {
      minion_state.joint_position = -1.0 * ((float) encoder_2_pos) * joint_degrees_per_encoder_count;
    }
    else {
      minion_state.joint_position = ((float) encoder_2_pos) * joint_degrees_per_encoder_count;
    }         
    minion_state.SE_position = minion_state.joint_position;
   }
  else if (minion_ident == BR_IDENT){
    //special case 19July19 for BR. BR has no separate SE encoder so joint_position is used instead
    if (encoder_direction_reversed == true) {
      minion_state.joint_position = -1.0 * ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    }
    else {
      minion_state.joint_position = ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    }
    minion_state.SE_position = minion_state.joint_position;
  }
  else {
    //normal case for all other joints
    if (encoder_direction_reversed == true) {
      minion_state.joint_position = -1.0 * ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    }
    else {
      minion_state.joint_position = ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    }
    if (series_elastic_direction_reversed == true) {
      minion_state.SE_position = -1.0 * ((float) encoder_2_pos) * SE_degrees_per_encoder_count;
    }
    else {
      minion_state.SE_position = ((float) encoder_2_pos) * SE_degrees_per_encoder_count;     
    }
  }

  //calculate the minion_state.motor_position
  minion_state.motor_position = ((float) stepper_counts) * degrees_per_microstep;  

  //calculate the velocities and accelerations
  minion_state.joint_velocity     = (minion_state.joint_position - previous_joint_position) / PUBLISH_UPDATE_INTERVAL_IN_SECONDS;
  minion_state.joint_acceleration = (minion_state.joint_velocity - previous_joint_velocity) / PUBLISH_UPDATE_INTERVAL_IN_SECONDS;
  minion_state.motor_velocity     = (minion_state.motor_position - previous_motor_position) / PUBLISH_UPDATE_INTERVAL_IN_SECONDS;
  minion_state.motor_acceleration = (minion_state.motor_velocity - previous_motor_velocity) / PUBLISH_UPDATE_INTERVAL_IN_SECONDS;

  //store the previous values for the next round
  previous_joint_position = minion_state.joint_position;
  previous_joint_velocity = minion_state.joint_velocity;
  previous_motor_position = minion_state.motor_position;
  previous_motor_velocity = minion_state.motor_velocity;
  
  //populate the operating_state portion of the minion_state
  if (operating_state == NORMAL){
    //parrots back override_command_received if everything is normal
    //add if to prevent timing glitch
    if(override_command_received == -999){
      minion_state.operating_state = EMERG_STOPPED_BY_MASTER;
    }
    else{
      minion_state.operating_state = override_command_received; 
    }
  }
  else{
    //reports non-normal state e.g. endstop detected etc.
    minion_state.operating_state = operating_state;
  }
}//end readSensors()

void publishAll() {
  //publish stuff for this joint

  /*//populate data
  state.data = states_msg;
  */

  //publish minion-appropriate data
  switch (minion_ident) {
    case BR_IDENT:
      BR_minion_state_pub.publish( &minion_state );
      nh.spinOnce();
      break;
    case SL_IDENT:
      SL_minion_state_pub.publish( &minion_state );
      nh.spinOnce();
      break;
    case UR_IDENT:
      UR_minion_state_pub.publish( &minion_state );
      nh.spinOnce();
      break;
    case EL_IDENT:
      EL_minion_state_pub.publish( &minion_state );
      nh.spinOnce();
      break;
    case LR_IDENT:
      LR_minion_state_pub.publish( &minion_state );
      nh.spinOnce();
      break;
    default:
      break;
  }//end switch case
}//end publishAll()

void setup() {
  // setup, then read, minion identification pins
  pinMode(IDENT_PIN_1, INPUT_PULLUP);
  pinMode(IDENT_PIN_2, INPUT_PULLUP);
  pinMode(IDENT_PIN_4, INPUT_PULLUP);
  pinMode(IDENT_PIN_8, INPUT_PULLUP);
  bitWrite(minion_ident, 0, digitalRead(IDENT_PIN_1));
  bitWrite(minion_ident, 1, digitalRead(IDENT_PIN_2));
  bitWrite(minion_ident, 2, digitalRead(IDENT_PIN_4));
  bitWrite(minion_ident, 3, digitalRead(IDENT_PIN_8));

  //init encoder pins
  pinMode(ENCODER_1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_2_PIN_B, INPUT_PULLUP);

  //init endstop pins
  pinMode(POSITIVE_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(NEGATIVE_ENDSTOP_PIN, INPUT_PULLUP);

  //init digital encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_A), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_B), doEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_A), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_B), doEncoder2B, CHANGE);

  //init stepper driver pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  //enable stepper by default
  digitalWrite(ENABLE_PIN, LOW);

  //setup publishers and subscribers
  //  nh.getHardware()->setBaud(230400);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
//  nh.getHardware()->setBaud(115200);//06Jan2018 lowered to 115200 for compatiblity with simultaneous running of Arduino Due //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.getHardware()->setBaud(57600);  //reset to this 1/22/19 for sevon-minio comaptibility - baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  //    nh.getHardware()->setBaud(9600);  //baud rate for this rosserail_arduino node must match rate for rosserial_python node running in terminal window on laptop
  nh.initNode();
  nh.subscribe(commanded_incremental_motor_movement_sub);
  nh.subscribe(override_commands_sub);

  /* setup encoders, pubs and subs needed for this minion's joint
    use switch to setup each joint's needs:
      joint_degrees_per_encoder_count
      degrees per microstep
      encoders, pubs and subs
  */

  if (minion_ident == BR_IDENT) {
      // BR joint
      joint_degrees_per_encoder_count = BR_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = BR_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = BR_DEGREES_PER_MICROSTEP;
      encoder_direction_reversed = BR_ENCODER_DIRECTION_REVERSED;
      series_elastic_direction_reversed = BR_SE_DIRECTION_REVERSED;
      stepper_direction_reversed = BR_STEPPER_DIRECTION_REVERSED;
      nh.advertise(BR_minion_state_pub);
      nh.loginfo("SetupBR");
  }
  else if (minion_ident == SL_IDENT) {
      // SL joint
      joint_degrees_per_encoder_count = SL_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = SL_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = SL_DEGREES_PER_MICROSTEP;
      encoder_direction_reversed = SL_ENCODER_DIRECTION_REVERSED;
      series_elastic_direction_reversed = SL_SE_DIRECTION_REVERSED;
      stepper_direction_reversed = SL_STEPPER_DIRECTION_REVERSED;
      nh.advertise(SL_minion_state_pub);
      nh.loginfo("SetupSL");
  }
  else if (minion_ident == UR_IDENT) {
      // UR joint
      joint_degrees_per_encoder_count = UR_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = UR_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = UR_DEGREES_PER_MICROSTEP;
      encoder_direction_reversed = UR_ENCODER_DIRECTION_REVERSED;
      series_elastic_direction_reversed = UR_SE_DIRECTION_REVERSED;
      stepper_direction_reversed = UR_STEPPER_DIRECTION_REVERSED;
      nh.advertise(UR_minion_state_pub);
//      ros::ServiceServer<my_robotic_arm::GetUpperarmOrientation::Request, my_robotic_arm::GetUpperarmOrientation::Response> get_upperarm_orientation_service("get_upperarm_orientation_service",&getUpperarmOrientationCallback);
//      nh.advertiseService(get_upperarm_orientation_service);
      nh.loginfo("SetupUR");
  }
  else if (minion_ident == EL_IDENT) {
      // EL joint
      joint_degrees_per_encoder_count = EL_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = EL_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = EL_DEGREES_PER_MICROSTEP;
      encoder_direction_reversed = EL_ENCODER_DIRECTION_REVERSED;
      series_elastic_direction_reversed = EL_SE_DIRECTION_REVERSED;
      stepper_direction_reversed = EL_STEPPER_DIRECTION_REVERSED;
      nh.advertise(EL_minion_state_pub);
      nh.loginfo("SetupEL");    
  }
  else if (minion_ident == LR_IDENT) {
      // LR joint
      joint_degrees_per_encoder_count = LR_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = LR_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = LR_DEGREES_PER_MICROSTEP;
      encoder_direction_reversed = LR_ENCODER_DIRECTION_REVERSED;
      series_elastic_direction_reversed = LR_SE_DIRECTION_REVERSED;
      stepper_direction_reversed = LR_STEPPER_DIRECTION_REVERSED;
      nh.advertise(LR_minion_state_pub);
//      ros::ServiceServer<my_robotic_arm::GetLowerarmOrientation::Request, my_robotic_arm::GetLowerarmOrientation::Response> get_lowerarm_orientation_service("get_lowerarm_orientation_service",&getLowerarmOrientationCallback);
//      nh.advertiseService(get_lowerarm_orientation_service);
      nh.loginfo("SetupLR");
      nh.spinOnce();
  }
  
  nh.spinOnce();

  //setup timers - ref: https://www.pjrc.com/teensy/td_libs_TimerOne.html 
  Timer1.initialize(RATE_UPDATE_INTERVAL);  //Timer1 fires to change i.e. update stepper velocity
//  Timer1.initialize(500000);  //Timer1 fires to change i.e. update stepper velocity
  Timer1.attachInterrupt(updateRate);
  Timer1.start();
  Timer1.stop();
  Timer3.initialize(2000000000);  //Timer1 fires to pulse stepper. just set to once every ~33 minutes for now. will not matter becasue it will be turned off
  Timer3.attachInterrupt(stepOnce);
  Timer3.start();
  Timer3.stop();

  //set first comm loss timeout limit to be time now plus 10 seconds to allow everything to come up to speed
  comm_loss_time_limit = millis() + 10000L;

  //set operating state to normal for startup
  operating_state = NORMAL;

  nh.loginfo("incremental_motor_movement_controller non-blocking version V0.0.04 12/11/2019");
  nh.spinOnce();
}// end setup()

void loop() {
  //wait until you are actually connected. Ref: http://wiki.ros.org/rosserial_arduino/Tutorials/Logging
  while (!nh.connected())
  {
    nh.spinOnce();
  }

  //  determine motion needed and begin moving
  if (new_plan)     {
    nh.loginfo("entering if new_plan 0");
    nh.spinOnce();
      
    //enable movment
    if (direction_positive){
      allowed_to_move_positive = true;
      allowed_to_move_negative = false;
    }
    else{
      allowed_to_move_positive = false;
      allowed_to_move_negative = true;
    }
    //operating_state = MOVING; //this is deprecated as of Dec 2019

    executeIncrementalMovement(required_duration, commanded_incremental_movement);
    new_plan = false;
  }//end if new_plan

  //log updates periodically
  time_now = millis();
  if (time_now > next_publish_update) {
    //read sensors
    readSensors();
    //check for lost comm
    if (time_now > comm_loss_time_limit){
      //initiate lost comm procedure
      allowed_to_move_positive = false;
      allowed_to_move_negative = false;
      operating_state = LOST_COMM_FROM_MASTER;
      movement_end_time = millis();//this will stop movement clock
    }
    //publish
    publishAll();
    next_publish_update = time_now + PUBLISH_UPDATE_INTERVAL;
  }//end if next_publish_update

  //shutdown movement when time is up
  if (time_now > movement_end_time) {
    //movement should be finished. stop stepper motions and Timers
    Timer1.stop();
    Timer3.stop();
    allowed_to_move_positive = false;
    allowed_to_move_negative = false;
  }  

  nh.spinOnce();
}// end loop()
