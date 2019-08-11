/* Purpose: 
 *    To better understand and control joint movements of a compliant cable driven robotic arm
 *    This accomodates the joint encoders that have been added (July 2019) to the joint itself for SL, UR & EL joints
 *    BR, LR, WL & WR joints already has such encoders
 * Runs on:
 *    Teensy, Arduino driving stepper motors
 *    For Dynamixels on OpenCM, use old xl430_incremental_movement_controller
 * Subscribes to ROS Topics:
 *    commanded_incremental_motor_movement
 * Publishes to ROS Topics:   
 *    BR_minion_state, SL_minion_state, etc
 * Action:   
 *    Moves motor to goal using RAMP algorithm
 * To Do:
 *    convert running_state and torque_state into an integer concept to publish as int minion_state.operating_state
 *    
 */

#include <ros.h>
#include <ArduinoHardware.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <Ramp.h>
#include <my_robotic_arm/MinionState.h>
#include <my_robotic_arm/GetUpperarmOrientation.h>
#include <my_robotic_arm/GetLowerarmOrientation.h>
#include <geometry_msgs/Vector3.h>
//#include <Adafruit_MMA8451.h>
//#include <Adafruit_Sensor.h>

//states
enum {UNPOWERED, HOLDING_POSITION, MOVING, FINAL_APPROACH, CW_ENDSTOP_ACTIVATED, CCW_ENDSTOP_ACTIVATED, CALIBRATING } running_state;
enum {GREEN, YELLOW, RED} torque_state;

//states string messages
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

//flags
volatile bool new_plan = false;

//debug messages
//static char miscMsgs[20];
static char inbound_message[40];

//internal position and direction variables
static volatile long encoder_1_pos = 0;  //position of doe in encoder clicks
static volatile long encoder_2_pos = 0;  //position of stepper doe in encoder clicks
static volatile long stepper_counts = 0; //stepper position in stepper steps
//static volatile bool encoder_direction_reversed = false;
static volatile bool stepper_direction_reversed = false;

//robot configuration constants
//const bool BR_ENCODER_DIRECTION_REVERSED = false;
//const bool SL_ENCODER_DIRECTION_REVERSED = false;
//const bool UR_ENCODER_DIRECTION_REVERSED = false;
//const bool EL_ENCODER_DIRECTION_REVERSED = false;
//const bool LR_ENCODER_DIRECTION_REVERSED = false;
const bool BR_STEPPER_DIRECTION_REVERSED = true;
const bool SL_STEPPER_DIRECTION_REVERSED = true;
const bool UR_STEPPER_DIRECTION_REVERSED = false;
const bool EL_STEPPER_DIRECTION_REVERSED = false;
const bool LR_STEPPER_DIRECTION_REVERSED = true;

//motion constants
static const float BR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.030000000; //July 2019 Encoder at joint  60:12 gears with 600ppr_quad/2400counts/rev
static const float SL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.053773585; //July 2019 Encoder at joint 106:38 gears with 600ppr_quad/2400counts/rev
static const float UR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.050000000; //July 2019 Encoder at joint  87:29 gears with 600ppr_quad/2400counts/rev
static const float EL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.058878505; //July 2019 Encoder at joint 107:42 gears with 600ppr_quad/2400counts/rev
static const float LR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.057142857; //July 2019 Encoder at joint  84:32 gears with 600ppr_quad/2400counts/rev
//static const float WL_JOINT_DEGREES_PER_ENCODER_COUNT = 0.010141226; //July 2019 Encoder at joint  65:15 gears with 2048ppr_quad/8196counts/rev
//static const float WR_JOINT_DEGREES_PER_ENCODER_COUNT = 0.010986328; //July 2019 Encoder at joint  64:16 gears with 2048ppr_quad/8196counts/rev

static const float BR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
static const float SL_SE_DEGREES_PER_ENCODER_COUNT = 0.014081124; //July 2019
static const float UR_SE_DEGREES_PER_ENCODER_COUNT = 0.018083128; //July 2019
static const float EL_SE_DEGREES_PER_ENCODER_COUNT = 0.004985766; //July 2019
static const float LR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
//static const float WL_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed
//static const float WR_SE_DEGREES_PER_ENCODER_COUNT = 0.0; //July 2019 - not installed

static const float BR_DEGREES_PER_MICROSTEP = 0.0150000000;  //15.000:1 gear ratio July 2019 arm configuration
static const float SL_DEGREES_PER_MICROSTEP = 0.0035202811;  //63.915:1 gear ratio July 2019 arm configuration
static const float UR_DEGREES_PER_MICROSTEP = 0.0045207820;  //49.770:1 gear ratio July 2019 arm configuration
static const float EL_DEGREES_PER_MICROSTEP = 0.0012464415; //180.51:1 gear ratio July 2019 arm configuration
static const float LR_DEGREES_PER_MICROSTEP = 0.0134927588;  //16.675:1 gear ratio July 2019 arm configuration
//static const float WL_DEGREES_PER_MICROSTEP = 0.0;  //xl430, not stepper, July 2019 arm configuration
//static const float WR_DEGREES_PER_MICROSTEP = 0.0;  //xl430, not stepper, July 2019 arm configuration

//motion variables
static volatile float required_duration;                //required duration in seconds for joint movement
static volatile float commanded_stepper_position = 0.0; //joint commanded_stepper_position in degrees
static volatile float commanded_incremental_movement = 0.0; //joint commanded_incremental_movement in degrees
static volatile bool  direction_CW = true;              //movement in positive joint direction is defined as CW
static volatile float current_pos = 0.0;                //joint position in degrees
static volatile float pos_error = 0.0;                  //difference in degrees between joint commanded_stepper_position and joint current position
static volatile long  steps_remaining = 0;              //stepper steps remaining to move. uses in final_approach and holding_position
static volatile float joint_degrees_per_encoder_count;  //reset to particular joint in setup()
static volatile float SE_degrees_per_encoder_count;     //reset to particular joint in setup()
static volatile float degrees_per_microstep;            //reset to particular joint in setup()

//variables to be published
my_robotic_arm::MinionState minion_state;
std_msgs::String state; //may be in multiple states simultaneously
std_msgs::String state2; //may be in multiple states simultaneously
//trajectory_msgs::JointTrajectoryPoint jtp;

//endstop pins
const int CW_ENDSTOP_PIN = 2;  //pin # corrected 12/6/17
const int CCW_ENDSTOP_PIN = 3; //pin # corrected 12/6/17

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

//loop timing variables
static volatile unsigned long next_update = 0L;//used to time loop updates
static const unsigned long UPDATE_INTERVAL = 500L;//was 500L before 7/4/19, in milliseconds, 500 is 2Hz, 50 is 20Hz, 10 is 100Hz

////service call backs
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
    //set flag
    new_plan = true;//new_plan flag is polled in loop() to initiate a new plan and motion
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

//subscribers
ros::Subscriber<std_msgs::String> commanded_incremental_motor_movement_sub("commanded_incremental_motor_movement", commandedIncrementalMotorMovementCallback);

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
  if (direction_CW) {
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
  if (digitalRead(CW_ENDSTOP_PIN) == LOW) {
    if (direction_CW) {
      nh.loginfo("CW endstop + dir CW, stop!");
      //WORKING HERE 12/6/17
      //NEED TO CHECK DIRECTIONS ARE CORRECT
      //NEED TO UNCOMMENT NEXT LINE
      running_state = CW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      nh.loginfo("CW endstop + dir CCW");
      running_state = CW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }
  if (digitalRead(CCW_ENDSTOP_PIN) == LOW) {
    if (!direction_CW) {
      nh.loginfo("CCW endstop + dir CCW, stop!");
      running_state = CCW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
    else {
      nh.loginfo("CCW endstop + dir CW");
      running_state = CCW_ENDSTOP_ACTIVATED;
      return; //exits stepOnce() if endstop has been activated in this direction
    }
  }

  switch (running_state) {
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
    case CW_ENDSTOP_ACTIVATED:
      break;
    case CCW_ENDSTOP_ACTIVATED:
      break;
    case CALIBRATING:
      break;
    default:
      break;
  }//end switch running_state
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

void executeIncrementalMovement(float the_required_total_duration_, float the_commanded_incremental_movement_) {
  /* Result: pulses stepper a precise number of times to acheive commanded degrees of movement at joint
   * Note: This is a blocking method. ros pub stuff will not operate while movement loop is in progress
   * Inputs:
   *  the_required_total_duration_ :: duration of complete movement in seconds
   *  the_commanded_incremental_movement_ :: commanded joint movement in degrees
   * Note:
   *  For legacy reasons, the_required_total_duration_ is in float seconds, while rampLong myRamp' needs duration as milliseconds as an unsigned long  
   */
  nh.loginfo("entering executeIncrementalMovement()");
  nh.spinOnce();

  rampLong myRamp;      // new ramp object (ramp<unsigned char> by default if not otherwise spec'd)

  //volatile long current_microsteps_per_second_ = 0L;  //this line causes service callback getUpperarmOrientationCallback() to fail and lock up node running 100+%
  long current_microsteps_per_second_ = 0L;
  long current_microstep_period_ = 0L;  //period in microseconds
  unsigned long required_duration_until_peak_in_milliseconds_ = 0L;

  steps_remaining = (long) abs(the_commanded_incremental_movement_ / degrees_per_microstep); //joint distance to travel in steps in absolute value as long - convert dtg_ in degrees to dtg_stepper_counts_ as a long

  //get peak_microsteps_per_second_
  long peak_microsteps_per_second_ = getPeakStepperVelocityNeeded(the_required_total_duration_, the_commanded_incremental_movement_);

  //convert required duration in float seconds to required_duration_until_peak_in_milliseconds_ in unsigned long. Notice this is only half the total duration as velocity ramps up to peak then down to zero
  required_duration_until_peak_in_milliseconds_ = (unsigned long) abs(the_required_total_duration_ * 1000.0 / 2.0);
    
//    sprintf(miscMsgs9, "long peak_microsteps_per_second_= %ld", peak_microsteps_per_second_);
//    nh.loginfo(miscMsgs9);

  //get direction of motion. positive distance defined as CCW, negative as CW
  if (the_commanded_incremental_movement_ > 0) {
      direction_CW = true;
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
      direction_CW = false;
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

}// end executeIncrementalMovement()

void readSensors() {
  //report state
  strcpy(states_msg, empty_states_msg);      //overwrites previous states_msg with empty msg
  if (running_state == UNPOWERED) strcat(states_msg, unpowered_msg);
  if (running_state == HOLDING_POSITION) strcat(states_msg, holding_position_msg);
  if (running_state == MOVING) strcat(states_msg, moving_msg);
  if (running_state == FINAL_APPROACH) strcat(states_msg, final_approach_msg);
  if (running_state == CW_ENDSTOP_ACTIVATED) strcat(states_msg, cw_endstop_activated_msg);
  if (running_state == CCW_ENDSTOP_ACTIVATED) strcat(states_msg, ccw_endstop_activated_msg);
  if (running_state == CALIBRATING) strcat(states_msg, calibrating_msg);
  if (torque_state == GREEN) strcat(states_msg, green_torque_msg);
  if (torque_state == YELLOW) strcat(states_msg, yellow_torque_msg);
  if (torque_state == RED) strcat(states_msg, red_torque_msg);

  //populate MinionState message
  if (minion_ident == LR_IDENT){
    //special case 19July19 for 3d accelerometer still wired to pins 20&21 I2C. LR has no SE encoder installed.
    minion_state.joint_position = ((float) encoder_2_pos) * joint_degrees_per_encoder_count;   
    minion_state.SE_position = minion_state.joint_position;
   }
  else if (minion_ident == BR_IDENT){
    //special case 19July19 for BR. BR has no separate SE encoder so joint_position is used instead
    minion_state.joint_position = ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    minion_state.SE_position = minion_state.joint_position;
  }
  else {
    //normal case for all other joints
    minion_state.joint_position = ((float) encoder_1_pos) * joint_degrees_per_encoder_count;
    minion_state.SE_position = ((float) encoder_2_pos) * SE_degrees_per_encoder_count;
  }

  minion_state.motor_position = ((float) stepper_counts) * degrees_per_microstep;
  //minion_state.motor_velocity = //this is populated in executeIncrementalMovement()
  minion_state.motor_acceleration = 0.0099; //this can be added later
  minion_state.operating_state = 99;  //To do - add this concept

}//end readSensors()

void publishAll() {
  //publish stuff for this joint

  //populate data
  state.data = states_msg;

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
  pinMode(CW_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(CCW_ENDSTOP_PIN, INPUT_PULLUP);

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
      //encoder_direction_reversed = BR_ENCODER_DIRECTION_REVERSED;
      stepper_direction_reversed = BR_STEPPER_DIRECTION_REVERSED;
      nh.advertise(BR_minion_state_pub);
      nh.loginfo("SetupBR");
  }
  else if (minion_ident == SL_IDENT) {
      // SL joint
      joint_degrees_per_encoder_count = SL_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = SL_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = SL_DEGREES_PER_MICROSTEP;
      //encoder_direction_reversed = SL_ENCODER_DIRECTION_REVERSED;
      stepper_direction_reversed = SL_STEPPER_DIRECTION_REVERSED;
      nh.advertise(SL_minion_state_pub);
      nh.loginfo("SetupSL");
  }
  else if (minion_ident == UR_IDENT) {
      // UR joint
      joint_degrees_per_encoder_count = UR_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = UR_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = UR_DEGREES_PER_MICROSTEP;
      //encoder_direction_reversed = UR_ENCODER_DIRECTION_REVERSED;
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
      //encoder_direction_reversed = EL_ENCODER_DIRECTION_REVERSED;
      stepper_direction_reversed = EL_STEPPER_DIRECTION_REVERSED;
      nh.advertise(EL_minion_state_pub);
      nh.loginfo("SetupEL");    
  }
  else if (minion_ident == LR_IDENT) {
      // LR joint
      joint_degrees_per_encoder_count = LR_JOINT_DEGREES_PER_ENCODER_COUNT;
      SE_degrees_per_encoder_count = LR_SE_DEGREES_PER_ENCODER_COUNT;
      degrees_per_microstep = LR_DEGREES_PER_MICROSTEP;
      //encoder_direction_reversed = LR_ENCODER_DIRECTION_REVERSED;
      stepper_direction_reversed = LR_STEPPER_DIRECTION_REVERSED;
      nh.advertise(LR_minion_state_pub);
//      ros::ServiceServer<my_robotic_arm::GetLowerarmOrientation::Request, my_robotic_arm::GetLowerarmOrientation::Response> get_lowerarm_orientation_service("get_lowerarm_orientation_service",&getLowerarmOrientationCallback);
//      nh.advertiseService(get_lowerarm_orientation_service);
      nh.loginfo("SetupLR");
      nh.spinOnce();
  }
  
  nh.spinOnce();

  //setup timer
  //  Timer7.attachInterrupt(stepOnce).setPeriod(0).start(); //delay(50);      //fires steppers at freqs specified in queue
  //ref: https://playground.arduino.cc/Code/Timer3//

  //Timer3 now setup in executeIncrementalMovement() 6/4/2019
//  Timer3.initialize(5000000);  // initialize Timer3, and set a 1/2 second period
//  Timer3.attachInterrupt(stepOnce);

  nh.loginfo("incremental_motor_movement_controller V0.0.01 7/15/2019");
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
      
    running_state = MOVING;

    executeIncrementalMovement(required_duration, commanded_incremental_movement);
//      nh.loginfo("finished if new_plan 1");
//      nh.spinOnce();
    new_plan = false;
//      nh.loginfo("finished if new_plan 2");
//      nh.spinOnce();
    running_state = HOLDING_POSITION;
//      nh.loginfo("finished if new_plan 3");
//      nh.spinOnce();
 }//end if new_plan

  //log updates
  if (millis() > next_update) {
    readSensors();
    publishAll();
    //    noInterrupts();
    next_update = millis() + UPDATE_INTERVAL;
    //    interrupts();
  }//end if next_update

  nh.spinOnce();
}// end loop()
