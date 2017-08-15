/**
 * MOVR Arduino Driver
 * 
 * This Arduino sketch subscribes to a "drive command" topic.
 * The messages received are used to control the vehicle's (front) steering motor and (rear) drive-motor.
 * The sketch also creates a ROS publisher to publish odometry data generated from the wheel encoders (and IMU).
 * 
 * @author Irvin S Cardenas 
 *    Email:  irvin@irvincardenas.com 
 *    GitHub: @kPatch
 *    
 * @author Jong-Hoon Kim 
 *    Email:  jkim72@kent.edu
 *    GitHub: @hoonywiz
 * 
 * 
 * To run:
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyACM1
 * // launch "steering_cmd" ROS publisher node
 * 
 * *****************************************
 * ABBREVIATIONS
 * *****************************************
 * steer      = steering
 * enc        = encoder
 * potmtr     = potentiometer
 * l          = left
 * r          = right
 * msg        = message
 * cmd        = command
 * cb         = callback
 * *****************************************
 * 
 * *****************************************
 * PLANNING
 * *****************************************
 * 
 * http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
 * http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots#Command_interfaces
 * 
 * https://answers.ros.org/question/260935/twist-message-working-ackermann-type-conversion/
 */

/* *****************************************
 * ROBOT Drive Motor Definitions
 * 
 * http://henrysbench.capnfatz.com/henrys-bench/arduino-voltage-measurements/arduino-pcf8591-digital-to-analog-tutorial/
 * *****************************************/
#include "Wire.h"
#define PCF8591 (0x90 >> 1)
int volatile out = 120;

/* *****************************************
 * Robot Odometry Definitions
 * 
 * Calibration_Factor = Distance_Traveled_per_Click = (PI * Wheel_Diameter) / Number_of_Clicks_per_Revolution 
 * Distance_Traveled_by_a_Wheel = Calibration_Factor * Number_of_Clicks_Traveled_by_Wheel
 * Average_Motion_of_Both_Wheels = (Distance_Traveled_by_Left_Wheel + Distance_Traveled_by_Right_Wheel) / Wheel_Baseline
 * 
 * Spefications:
 * >  600 pulses/revolution for a single phase.
 *    Therefore, two-phase output leads to 24000
 * >  Maximum mechanical speed: 5000 Revolutions / minute
 * >  Response Frequence: 0 - 20KHz
 * 
 * Wire Mappings:
 * Wire Color     Connections
 * RED            5 - 24V DC
 * BLACK          Ground
 * GREEN          A Phase
 * WHITE          B Phase
 * 
 * LPD3806 G5 24C rotary encoder produces 2400 pulses for 1 complete revolution
 * 1 complete revolution equal 360 degrees
 * 
 * encLAngle = Pulse Count * (360/2400)
 * encLAngle = Pulse Count * (3/20)
 * 
 * https://www.youtube.com/watch?v=qsdiIZncgqo
 * https://www.youtube.com/watch?v=0QLZCfqUeg4
 * http://ttuadvancedrobotics.wikidot.com/odometry#toc9
 * https://toriilab.blogspot.com/2016/09/a-rotary-encoder-is-used-to-measure.html
 * http://correll.cs.colorado.edu/?p=1869
 * http://planning.cs.uiuc.edu/node658.html
 * *****************************************/
#define WHEEL_L_DIAMETER      8.50        // The diameter of the left rear wheel    8.5 mm
#define WHEEL_R_DIAMETER      8.50        // The diameter of the right rear wheel   8.5 mm
#define WHEELS_BASELINE       43.25       // The distance between the left wheel and right wheel contact point, 43.25 MM
#define CLICKS_PER_REVOLUTION 2400
#define CALIBRATION_FACTOR    0.01112083  // (M_PI * WHEEL_DIAMETER) / CLICKS_PER_REVOLUTION; (M_PI * 8.5) / 2400

// LEFT REAR ENCODER Variables
#define                 ENC_L_PIN_A   21    // Left encoder pin A
#define                 ENC_L_PIN_B   20    // Left encoder pin B
float                   encLAngle   = 0.0;  // Left encoder rotation angle
volatile unsigned int   encLPos     = 0;    // Left encoder ticks (position)
volatile unsigned int   encLRevs    = 0;    // Left encoder number of revolutions

// RIGHT REAR ENCODER Variables
#define                 ENC_R_PIN_A   19    // Right encoder pin A
#define                 ENC_R_PIN_B   18    // Right encoder pin B
float                   encRAngle   = 0.0;  // Right encoder rotation angle
volatile unsigned int   encRPos     = 0;    // Right encoder tick (position)
volatile unsigned int   encRevs     = 0;    // Right encoder number of revolutions

volatile unsigned int   vehicleVelocity = 0;

/* *****************************************
 * STEERING PID Definitions
 * *****************************************/
#include <PID_v1.h>
#define MOTOR1_A_PIN      6
#define MOTOR1_B_PIN      7
#define MOTOR1_PWM_PIN    5
#define MOTOR1_POT_PIN    A7
#define INIT_SET_POINT    512

//Define Variables we'll be connecting to
double motor1Setpoint, motor1Input, motor1Output;
double motor1Kp = 2;
double motor1Ki = 5;
double motor1Kd = 0.12;

// Specify the links and initial tuning parameters
// P_ON_M specifies that Proportional on Measurement be used
// P_ON_E (Proportional on Error) is the default behavior
PID myPID(&motor1Input, &motor1Output, &motor1Setpoint, motor1Kp, motor1Ki, motor1Kd, P_ON_M, DIRECT);
 
/********************************************
 *  ROS Imports
 ********************************************/
#include <ArduinoHardware.h>
#include <ros.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <movr_steering/AckermannDrive.h>

/********************************************
 * ROS Node(s) Definitions
 ********************************************/
ros::NodeHandle nh;              // Node Handler
std_msgs::String steerAngleMsg;    // Steering odometry message
//char steerAngleReading[10] = "TEST_ANGLE";

/**
 * ROS movr command subscriber - callback function
 * Receives AckermannDrive message
 * Then updates PID "motor1Setpoint"
 * 
 * http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html
 */
void movrCmdCb(const movr_steering::AckermannDrive& steerCmd) {
  // SET Angula
  motor1Setpoint = transformSteerAngleToPotmtr(steerCmd.steering_angle);

  // SET Linear Velocity According to AckermannDrive Command's Linear Velocity and Odometry Velocity Calculations
  out = steerCmd.speed;
}

ros::Subscriber<movr_steering::AckermannDrive> movrCmdSub("movr_cmd", &movrCmdCb);  // Subscriber receives AckermannDrive messages
ros::Publisher steerAnglePub("steer_angle_pub", &steerAngleMsg);                    // 

void setup() {
  Wire.begin();                     // Required for A to DA converter
  Serial.begin(57600);
  
  pinMode(MOTOR1_POT_PIN, INPUT);
  pinMode(MOTOR1_A_PIN, OUTPUT);
  pinMode(MOTOR1_B_PIN, OUTPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  initPID();

  // Setup for LEFT REAR ENCODER
  pinMode(ENC_L_PIN_A, INPUT);
  pinMode(ENC_L_PIN_B, INPUT);
  pinMode(ENC_L_PIN_A, INPUT_PULLUP);
  pinMode(ENC_L_PIN_B, INPUT_PULLUP);
  void readEncLA();
  void readEncLB();
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN_A), readEncLA, CHANGE);   // encoder pin on interrupt 0 (pin 21)
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN_B), readEncLB, CHANGE);   // encoder pin on interrupt 1 (pin 20)

  // Setup for RIGHT REAR ENCODER
//  pinMode(ENC_R_PIN_A, INPUT);
//  pinMode(ENC_R_PIN_B, INPUT);
//  pinMode(ENC_R_PIN_A, INPUT_PULLUP);
//  pinMode(ENC_R_PIN_B, INPUT_PULLUP);
//  void readEncRA();
//  void readEncRB();
//  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN_A), readEncRA, CHANGE);   // encoder pin on interrupt 0 (pin 21)
//  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN_B), readEncRB, CHANGE);   // encoder pin on interrupt 1 (pin 20)


  nh.initNode();
  nh.subscribe(movrCmdSub);
}

void loop() {
  updatePIDInput();
  if(myPID.Compute() == true) {
    updatePIDOutput();
  }

  encLAngle = encLPos * (3.0 / 20.0);     // LEFT Encoder Angle
  
  Wire.beginTransmission(PCF8591);
  Wire.write(0x40);                 // Sets the PCF8591 into DA mode
  Wire.write(out);                  // Sets the output
  Wire.endTransmission();
  
  nh.spinOnce();
  // checkMessage(); // This will not be need as we will get the message from the ROS callback function
}

void initPID() {
  motor1Input = analogRead(MOTOR1_POT_PIN);
  motor1Setpoint = INIT_SET_POINT;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
}

void updatePIDInput() {
  motor1Input = analogRead(MOTOR1_POT_PIN);
}

void updatePIDOutput() {
  if(motor1Output > 0) {
    digitalWrite(MOTOR1_A_PIN, 0);
    digitalWrite(MOTOR1_B_PIN, 1);
  }
  else {
    digitalWrite(MOTOR1_A_PIN, 1);
    digitalWrite(MOTOR1_B_PIN, 0);
  }

  analogWrite(MOTOR1_PWM_PIN, abs(motor1Output));   // Move the steering motor
}

/**
 * Transform steering angle to potentiometer analogue
 * @param angle - low: -45, high:45
 * fromLow: min left turning angle
 * fromHigh: max right turning angle
 * toLow: min potentiometer reading identified to map to 45 degrees LEFT
 * toHigh: max potentiometer reading identified to map to 45 degrees RIGHT
 */
double transformSteerAngleToPotmtr(double angle) {
  //return map(angle, -45, 45, 256, 768);
  return map(angle, -45, 45, 266, 758);
}

void readEncLA() {
  if(digitalRead(ENC_L_PIN_A) == HIGH) {   // Look for a low-to-high on channel A
    if(digitalRead(ENC_L_PIN_B) == LOW) {  // Check channel B to see which way encoder is turning
      encLPos = encLPos + 1;            // CW
    } else {
      encLPos = encLPos - 1;            // CCW
    }
  } else {                              // Must be a high-to-low edge on channel A
    if(digitalRead(ENC_L_PIN_B) == HIGH) {
      encLPos = encLPos + 1;            // CW
    } else {
      encLPos = encLPos - 1;            // CCW
    }
  }
  
  if(encLPos > 2400) {
    encLPos = 0;
  }
}

void readEncLB() {
  if(digitalRead(ENC_L_PIN_B) == HIGH) {   // Look for a low-to-high on channel B
    if(digitalRead(ENC_L_PIN_A) == HIGH) { // Check channel A to see which way encoder is turning
      encLPos = encLPos + 1;            // CW
    } else {
      encLPos = encLPos - 1;            // CCW
    }
  } else {                              // Look for a high-to-low on channel B
    if(digitalRead(ENC_L_PIN_A) == LOW) {  // Check channel B to see which way encoder is turning
      encLPos = encLPos + 1;            // CW
    } else {
      encLPos = encLPos - 1;            // CCW
    }
  }
   
  if(encLPos > 2400) {
    encLPos = 0;
  }
}

//double calibrationFactor(double wheelDiameter, int nClicks) {
//  return (M_PI * wheelDiameter) / nClicks;
//}

/**
 * Calculates the distance traveled for a single wheel
 */
double distanceTraveled(int nClicks) {
  return CALIBRATION_FACTOR * nClicks;
}

double totalDistanceTraveled(double leftWheelDistance, double rightWheelDistance) {
  return (leftWheelDistance + rightWheelDistance) / 2;
}

