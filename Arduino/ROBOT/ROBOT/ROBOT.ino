// Include libraries
#include <MatrixMath.h>
#include <Mecanum.h>
#include <DC_motor.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Pose.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MFRC522.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif
#include <KalmanFus.h>
#include <discreteKalman.h>
#include <PinAllocation.h>

const float pi = 3.14159265359;

// Dimensions of Robot
float l_x = 225.5;
float l_y = 217.5;
float R = 30;

// Dimenstions of operation area:
float area_x = 1250;
float area_y = 1250;
float safe_dis = 325; // safety distance to the edge of the stage

// translation of gearbox
const float translation = 74.831;
// Encoder sample time in milliseconds (is equal to odometry -> and fusion with IMU)
const int encoder_frequency = 50;
const int IMU_frequency = 100;
// Encoder counts per round
const float counts_per_round = 3591.84;
// PID sample time
const int PID_sample_time = 50;

// Steps, Step times and step delays for
const int steps_Wings = 50;
const int steps_Seats = 50;
const int stepTime_Wings = 1500; //Microseconds
const int stepDelay_Wings = 100; //Milliseconds
const int stepTime_Seats = 8000; // Microseconds
const int stepDelay_Seats = 1; // Milliseconds

// Create MPU object
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw_prev;         // Previos yaw angle

// Create RFID reader object
MFRC522 mfrc522(SDA_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::StatusCode status; //variable to get card status

// RFID variables
byte buffer[18];  //data transfer buffer (16+2 bytes data+CRC)
byte buffer_size;
uint8_t pageAddr = 0x06; //read 16 bytes (Page 6,7,8,9)

// Create mecanum wheel object
Mecanum SRTMecanum(l_x, l_y, R);

// Create global Pose
Pose Robot_Pose;

// Create Kalman filter to fuse IMU and odometry data
KalmanFus KalmanOdoIMU;

// Create discrete Kalman filter for NFC tags
discreteKalman KalmanNFC;

// Create DC-motors
DC_motor DC_1(DC_M1_Dir, DC_M1_Speed, translation);
DC_motor DC_2(DC_M2_Dir, DC_M2_Speed, translation);
DC_motor DC_3(DC_M3_Dir, DC_M3_Speed, translation);
DC_motor DC_4(DC_M4_Dir, DC_M4_Speed, translation);

bool DC_STOP;
bool Stepper_STOP; 

// Create Encoders
Encoder Encoder_1(counts_per_round);
Encoder Encoder_2(counts_per_round);
Encoder Encoder_3(counts_per_round);
Encoder Encoder_4(counts_per_round);

// Encoder timing
unsigned long time;
unsigned long time_prev = 0;
int time_diff;
int time_diff_IMU;
unsigned long time_prev_IMU;


// Speeds
double v[3]; // desired speed in local frame
double w[4]; // desired rotational wheel speed WITH SIGN
double w_should[4]; // Desired rotational wheel speeds WITHOUT SIGN
double w_is[4]; // measured rotational wheel speeds WITHOUT SIGN
double w_is_sign[4]; // measured rotational wheel speeds WITH SIGN

// Create PID for DC motors // Output limit is 0-255 by default
double PID_Out_DC[4]; //Output of PID controler
double Kp_DC = 10, Ki_DC = 90, Kd_DC = 0;
PID PID_DC1(&w_is[0], &PID_Out_DC[0], &w_should[0], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC2(&w_is[1], &PID_Out_DC[1], &w_should[1], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC3(&w_is[2], &PID_Out_DC[2], &w_should[2], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC4(&w_is[3], &PID_Out_DC[3], &w_should[3], Kp_DC, Ki_DC, Kd_DC, DIRECT);

// Create PID for position Control
double Kp_Pose_xy = 2.5, Ki_Pose_xy = 0.5, Kd_Pose_xy = 0;
PID PID_x(&Robot_Pose.globalPose[0], &v[0], &Robot_Pose.globalPose_should[0], Kp_Pose_xy, Ki_Pose_xy, Kd_Pose_xy, DIRECT);
PID PID_y(&Robot_Pose.globalPose[1], &v[1], &Robot_Pose.globalPose_should[1], Kp_DC, Ki_DC, Kd_DC, DIRECT);
double Kp_Pose_th = 1, Ki_Pose_th = 1, Kd_Pose_th = 0;
PID PID_theta(&Robot_Pose.globalPose[2], &v[2], &Robot_Pose.globalPose_should[2], Kp_DC, Ki_DC, Kd_DC, DIRECT);

// Maximal Speeds in mm/s and rad/s
const double v_max_x = 200;
const double v_max_y = 200;
const double v_max_theta = 1;
// Arguments from remote control can either be in local frame or global frame:
bool remote_local = true;
// Distances/angle at which PID takes over
const double PID_dis_x = 100;
const double PID_dis_y = 100;
const double PID_dis_theta = 1;
// Allowed Pose Error:
const float Pose_error_xy = 5;
const float Pose_error_theta = 0.04;

bool print_to_COM = true; //Print data da serial port (computer)
bool print_commands = true;

// States of the robot
int state;
/* switch(state){
 *  case(0): standby -> no mode selected, no movement, INITIAL Pose can be set
 *  case(1): driving -> Robot expects commands to move with velocity vector for a certain time --> int drive_time
 *  case(2): folding -> Robot expects commands to fold
 *  case(3): analog remote controll
 *  case(4): not implemented: predefined configurations
 *  case(5): not implemented: PID position control
 */
 int drive_time = 3000; // drive for 3000 milliseconds

// layouts / configurations
 int layout;
 /* switch(layout){
  *  case(0): private/moving
  *  case(1): peak hour lean
  *  case(2): peak hour seat
 */

 // localization method
 int loc_method;
 /* switch(loc_method){
  *  case(0): odometry = is default method
  *  case(1): odometry + IMU
  *  case(2): odometry + IMU + RFID
 */

#include <Folding_Functions.h>
void setup() {
  // Set states, layout and localization method
  change_state(0);
  layout = 0;
  loc_method = 0;
  // Will be set to true when entering state 0 (STANDBY)
  DC_STOP = false;
  Stepper_STOP = false;
  
  // Set speed in local frame
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  // Set global initial pose:
  Robot_Pose.setglobalPose(0,0,0);
  Robot_Pose.setglobalPose_should(0,0,0);

  // Initialize IMU and RFID reader
  Init_IMU_RFID();

  // Initialize Bluetooth
  BT.begin(9600);
  // Initialize serial communications with Computer
  Serial.begin(9600);
  
  attachInterrupt(digitalPinToInterrupt(Encoder_1_pin), doEncoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_2_pin), doEncoder_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_3_pin), doEncoder_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_4_pin), doEncoder_4, CHANGE);
 
  // Pins for DC-motors
  pinMode(DC_M1_Dir, OUTPUT);
  pinMode(DC_M2_Dir, OUTPUT);
  pinMode(DC_M3_Dir, OUTPUT);
  pinMode(DC_M4_Dir, OUTPUT);
  pinMode(DC_M1_Speed, OUTPUT);
  pinMode(DC_M2_Speed, OUTPUT);
  pinMode(DC_M3_Speed, OUTPUT);
  pinMode(DC_M4_Speed, OUTPUT);
  DC_STOP = allWheelsSTOP();
  
  //Magnet
  pinMode(magnets, OUTPUT);
  
  // Pins for Stepper motors
  pinMode(Wing_R_dir,OUTPUT); 
  pinMode(Wing_L_dir,OUTPUT);
  pinMode(Seat_R_dir,OUTPUT); 
  pinMode(Seat_M_dir,OUTPUT);
  pinMode(Seat_L_dir,OUTPUT); 
  pinMode(Wing_R_step,OUTPUT); digitalWrite(Wing_R_step, LOW);
  pinMode(Wing_L_step,OUTPUT); digitalWrite(Wing_L_step, LOW);
  pinMode(Seat_R_step,OUTPUT); digitalWrite(Seat_R_step, LOW);
  pinMode(Seat_M_step,OUTPUT); digitalWrite(Seat_M_step, LOW);
  pinMode(Seat_L_step,OUTPUT); digitalWrite(Seat_L_step, LOW);
  pinMode(sleep_Stepper,OUTPUT); digitalWrite(sleep_Stepper, HIGH);
  Stepper_STOP = allSteppersSTOP();
  
  // Pins for Encoders
  pinMode(Encoder_1_pin, INPUT);
  digitalWrite(Encoder_1_pin, HIGH);
  pinMode(Encoder_2_pin, INPUT);
  digitalWrite(Encoder_2_pin, HIGH);
  pinMode(Encoder_3_pin, INPUT);
  digitalWrite(Encoder_3_pin, HIGH);
  pinMode(Encoder_4_pin, INPUT);
  digitalWrite(Encoder_4_pin, HIGH);

  // PID settings
  PID_DC1.SetMode(AUTOMATIC);
  PID_DC1.SetSampleTime(PID_sample_time);
  PID_DC2.SetMode(AUTOMATIC);
  PID_DC2.SetSampleTime(PID_sample_time);
  PID_DC3.SetMode(AUTOMATIC);
  PID_DC3.SetSampleTime(PID_sample_time);
  PID_DC4.SetMode(AUTOMATIC);
  PID_DC4.SetSampleTime(PID_sample_time);

  PID_x.SetMode(AUTOMATIC);
  PID_x.SetSampleTime(PID_sample_time);
  PID_y.SetMode(AUTOMATIC);
  PID_y.SetSampleTime(PID_sample_time);
  PID_theta.SetMode(AUTOMATIC);
  PID_theta.SetSampleTime(PID_sample_time);
  PID_x.SetOutputLimit(v_max_x, v_max_x);
  PID_y.SetOutputLimit(-v_max_y, v_max_y);
  PID_theta.SetOutputLimit(-v_max_theta, v_max_theta);
}

void loop() {  
  //Command Variables
  char  command_buffer[20]; // stores entire command
  char command = 'd'; //'d' = default
  int arg1 = 0;
  int arg2 = 0;
  int arg3 = 0;

  switch(state){
    // STANDBY /////////////////////////////////////////////////
    case 0:{
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          // Change and initialize states
          change_state(arg1);
          break;
        }
        case 'P': // Set global pose according to arguments
        {
          Robot_Pose.setglobalPose(arg1,arg2,arg3);
          print_globalPose();
          break;
        }
        default: {state = 0; break;}// Stay in STANDBY 
      }
      break;
    }
    ////////////////////////////////////////////////////////////
    // Driving /////////////////////////////////////////////////
    case 1:{
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          change_state(arg1);
          break;
        }
        case 'L': // Change localization method (first argument)
        {
          change_loc_method(arg1);
          break;
        }
		case 'P': // Set global pose according to arguments
        {
          Robot_Pose.setglobalPose(arg1,arg2,arg3);
          print_globalPose();
          break;
        }
        case 'M': //Moving with velocities defined by arg1-3 for a certain time
        {
          DC_STOP = false;
          if(layout!=0){
            if(print_to_COM){
              Serial.println("Bevore moving change into privacy/moving configuration!!");
              break;
            }
          }
          // Calculations of wheel speeds
          v[0] = arg1;
          v[1] = arg2;
          v[2] = arg3;
          if(v[0]==0&&v[1]==0&&v[2]==0){
            DC_STOP = allWheelsSTOP();
          }
          // Calculate desired wheels speeds WITH SIGN
          SRTMecanum.CalcWheelSpeeds((float*)v, (float*)w);
          // Desired wheel speeds WITHOUT SIGN (needed for PID controller
          SRTMecanum.WheelSpeeds_NoSign((float*)w, (float*)w_should);
          
          // Print header for localization results
          if(print_to_COM){
            switch(loc_method){
              case 0: {Serial.println("Pose odometry:   x \t y \t theta"); break;}
              case 1: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta"); break;}
              case 2: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta    Pose Kalman RFID    x \t y \t theta"); break;}    
            }
          }

          // Reset Encoders and Reset FiFo buffer of IMU and get current yaw angle from IMU
          getEnconderSpeeds(50);
          mpu.resetFIFO();
          get_IMU_yaw();
          yaw_prev = ypr[0];
          // Initialize times
          time = millis();
          //time_prev NEEDED for localization: Before starting localization always ste time_prev to millis!!
          time_prev = millis();
          //driving for drive_time milliseconds
          localize_Robot();
          while(millis()-time<drive_time){
            localize_Robot();          
            if(!DC_STOP){// Compute PID results for DC motors and run DC motors
              run_DC();
            }
          }
          DC_STOP = allWheelsSTOP();
          localize_Robot();
          break;
        }
        
        default: {break;} 
      }
      break;      
    }
    ////////////////////////////////////////////////////////////
    // FOLDING /////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    case 2: 
    {
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          change_state(arg1);
          break;
        }
        case 'W': //'W' is fold wings first argument: both(2) or left(1) or right(0) wing, second argument open(1) or close(0)
        {
          do_Wings(arg1, arg2);
          break;
        }
        case 'C': //'C' for chair(seat): first argument Seat (all left middle right) -> (3 2 1 0) & second argument direction (up down): (1 0)
        {
          do_Seats(arg1, arg2);
          break;
        }
        
      }
      break;
    }
    ////////////////////////////////////////////////////////////
    // REMOTE CONTROL //////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    case 3:
    {
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          change_state(arg1);
          break;
        }
        case 'L': // Change localization method (first argument)
        {
          change_loc_method(arg1);
          break;
        }
        case 'P': // Set global pose according to arguments
        {
          Robot_Pose.setglobalPose(arg1,arg2,arg3);
          print_globalPose();
          break;
        }
        case 'M': //Moving with velocities defined by arg1-3 for a certain time
        {
          DC_STOP = false;
          print_commands = false;
          if(layout!=0){
            if(print_to_COM){
              Serial.println("Bevore moving change into privacy/moving configuration!!");
              break;
            }
          }
          // Print header for localization results
          if(print_to_COM){
            switch(loc_method){
              case 0: {Serial.println("Pose odometry:   x \t y \t theta"); break;}
              case 1: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta"); break;}
              case 2: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta    Pose Kalman RFID    x \t y \t theta"); break;}    
            }
          }
           // Reset Encoders and Reset FiFo buffer of IMU and get current yaw angle from IMU
          getEnconderSpeeds(50);
          mpu.resetFIFO();
          get_IMU_yaw();
          yaw_prev = ypr[0];
          //time_prev NEEDED for localization: Before starting localization always ste time_prev to millis!!
          time_prev = millis();
		      time_prev_IMU = millis();
          localize_Robot();
          while(command=='M'){
            read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
            if(remote_local){  
              // Calculations of wheel speeds (speeds are local)
              v[0] = v_max_x*arg1/100;
              v[1] = v_max_y*arg2/100;
              v[2] = v_max_theta*arg3/100;
            }
            else{ // if speeds are from a global perspective
              Robot_Pose.speed_to_local_v((float*)v, (float)arg1/100, (float)arg2/100, (float)arg3/100);
            }            
            arg1 = 0;
            arg2 = 0; 
            arg3 = 0;
            if(v[0]==0&&v[1]==0&&v[2]==0){
              DC_STOP = allWheelsSTOP();
              break;
            }
            // Calculate desired wheels speeds WITH SIGN
            SRTMecanum.CalcWheelSpeeds((float*)v, (float*)w);
            // Desired wheel speeds WITHOUT SIGN (needed for PID controller
            SRTMecanum.WheelSpeeds_NoSign((float*)w, (float*)w_should);   
            Serial.println(w_should[0]);         
            localize_Robot();  
            if(!DC_STOP){// Compute PID results for DC motors and run DC motors
              run_DC();
            }         
          }
          localize_Robot();         
          break;
        }
        default: {break;} 
      }
      break;
    }
    ////////////////////////////////////////////////////////////
    // Layouts //////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    case 4:
    {
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          change_state(arg1);
          break;
        } 
      }
      break;
    }
    ////////////////////////////////////////////////////////////
    // PID POSITION CONTROL ////////////////////////////////////
    ////////////////////////////////////////////////////////////
    case 5:
    {
      // Waiting for commands
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //First character of command
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          change_state(arg1);
          break;
        }
        case 'L': // Change localization method (first argument)
        {
          change_loc_method(arg1);
          break;
        }
        case 'P': // Set global pose according to arguments
        {
          Robot_Pose.setglobalPose(arg1,arg2,arg3);
          print_globalPose();
          break;
        }
        case 'T': //'T': target position
        {
          // Arguments contain the desired position
          Robot_Pose.globalPose_should[0] = arg1;
          Robot_Pose.globalPose_should[1] = arg2;
          Robot_Pose.globalPose_should[2] = arg3;
          if(print_to_COM){
            Serial.print("Heading to x: ");
            Serial.print(arg1);
            Serial.print("\t y: ");
            Serial.print(arg2);
            Serial.print("\t theta: ");
            Serial.println(arg3);
          }
          DC_STOP = false;
          // Print header for localization results
          if(print_to_COM){
            switch(loc_method){
              case 0: {Serial.println("Pose odometry:   x \t y \t theta"); break;}
              case 1: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta"); break;}
              case 2: {Serial.println("Pose odometry:   x \t y \t theta    Angle IMU:   theta    Pose Kalman:    x \t y \t theta    Pose Kalman RFID    x \t y \t theta"); break;}    
            }
          }
          // Reset Encoders and Reset FiFo buffer of IMU and get current yaw angle from IMU
          getEnconderSpeeds(50);
          mpu.resetFIFO();
          get_IMU_yaw();
          yaw_prev = ypr[0];

          // Calculated distance to go in local system         
          Robot_Pose.calctoGo_local();
          while(abs(Robot_Pose.toGo_local[0])>Pose_error_xy || abs(Robot_Pose.toGo_local[1])>Pose_error_xy || abs(Robot_Pose.toGo_local[2])>Pose_error_theta){
            if(abs(Robot_Pose.toGo_local[0])<PID_dis_x){
              PID_x.Compute();
            }
            else{
              v[0] = sgn(Robot_Pose.toGo_local[0])*v_max_x;
            }
            if(abs(Robot_Pose.toGo_local[1])<PID_dis_y){
              PID_y.Compute();
            }
            else{
              v[1] = sgn(Robot_Pose.toGo_local[1])*v_max_y;
            }
            if(abs(Robot_Pose.toGo_local[2])<PID_dis_theta){
              PID_theta.Compute();
              
            }
            else{
              v[2] = sgn(Robot_Pose.toGo_local[2])*v_max_theta;
            }
            
            if(v[0]==0&&v[1]==0&&v[2]==0){
              DC_STOP = allWheelsSTOP();
            }
            // Calculate desired wheels speeds WITH SIGN
            SRTMecanum.CalcWheelSpeeds((float*)v, (float*)w);
            // Desired wheel speeds WITHOUT SIGN (needed for PID controller
            SRTMecanum.WheelSpeeds_NoSign((float*)w, (float*)w_should);
                     
            if(!DC_STOP){
              run_DC();
            }
            localize_Robot();
            
            Robot_Pose.calctoGo_local();
            Serial.println(v[0]);
            Serial.println(v[1]);

          }
          DC_STOP = allWheelsSTOP();
          if(print_to_COM){
            Serial.println("Position reached!!");
          }
        
          
          break;
        }
        default: {break;} 
      }
      break;
    }
    // Default case
    default: 
    {
      if(print_to_COM){
        Serial.print("ERROR: No state selected. Variable state = ");
        Serial.println(state);
      }
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      break;
    }
  }
}


/*////////////////////////////////////////////////////////////////////////////////
////////////////FUNCTIONS/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////*/
// Read Bluetooth Commands
void read_BT_command(char* command_buffer, char *command, int *arg1, int *arg2, int *arg3){
  int input;
  int i = 0;
  while(1){
     /* //This is just to determine the standard deviation of the IMU
      get_IMU_yaw();
      Serial.print(ypr[0],DEC);
      Serial.print("\t\t\t");
      Serial.println(millis());*/
    if ( BT.available() ) {
        input = (BT.read());
        command_buffer[i] = input;
        i++;
        if (input == '!') {
            i = 2;
            break;
        }
        delay(1);
    } else {
      if(print_to_COM)
      {
        //Serial.println("Bluetooth not available");
      }      
    }
  }    
  *command = command_buffer[0];
  //Converts subsequent characters in the array into integer as function argument
  int sign = 1;
  while (command_buffer[i] != '|') {
      if(command_buffer[i] == '-'){
        sign = -1;
      }
      else{
        *arg1 *= 10;
        *arg1 = *arg1 + (command_buffer[i] - 48);
      }
      i++;
  }
  *arg1 *= sign;
  i++;
  sign = 1;
  while (command_buffer[i] != '|') {
      if(sign == 1 && command_buffer[i] == '-'){
        sign = -1;
      }
      else{
        *arg2 *= 10;
        *arg2 = *arg2 + (command_buffer[i] - 48);
      }
      i++;
  }
  *arg2 *= sign;
  i++;
  sign = 1;
  while (command_buffer[i] != '!') {
      if(sign == 1 && command_buffer[i] == '-'){
        sign = -1;
      }
      else{
      *arg3 *= 10;
      *arg3 = *arg3 + (command_buffer[i] - 48);
      }
      i++;
  }
  *arg3 *= sign;
    //Check/print command:
  if(print_commands){
    Serial.print("Command Buffer: ");
    Serial.println(command_buffer);
    Serial.print("Command: ");
    Serial.println(*command);
    Serial.print("arg1: ");
    Serial.println(*arg1);
    Serial.print("arg2: ");
    Serial.println(*arg2);
    Serial.print("arg3: ");
    Serial.println(*arg3);   
  }
  memset(command_buffer, 0, sizeof(command_buffer));
} 

///////////////////////////////////////////////////
void print_globalPose(){
  if(print_to_COM){
    Serial.println("The (new) pose is: ");
    Serial.print(Robot_Pose.globalPose[0]);
    Serial.print("\t");
    Serial.print(Robot_Pose.globalPose[1]);
    Serial.print("\t");
    Serial.println(Robot_Pose.globalPose[2]);
  }
}

///////////////////////////////////////////////////
void change_loc_method(int arg1){
  loc_method = arg1;
  switch(arg1){
    case 0: {break;}
    case 1:
    case 2: 
    {
        // Check IMU
      if(!dmpReady){
        if(print_to_COM){
        Serial.println("IMU not ready");
        }
        change_loc_method(0);//change to odometry only 
        return;
      }
      // Set detected tag to current position
      KalmanNFC.tag_det[0] = Robot_Pose.globalPose[0];
      KalmanNFC.tag_det[1] = Robot_Pose.globalPose[1]; 
      //Get current yaw angle
      mpu.resetFIFO();
      // read a packet from FIFO of IMU
      get_IMU_yaw();
      yaw_prev = ypr[0];
      break;
      }
  }
  if(print_to_COM){
    switch(arg1){     
      case 0: 
      {
        Serial.println("Localization method changed to: ODOMETRY"); 
        break;
      }      
      case 1: 
      {
        Serial.println("Localization method changed to: ODOMETRY + IMU"); 
        Serial.print("Current yaw-angle: ");
        Serial.println(yaw_prev);
        break;
      }      
      case 2: 
      {
        Serial.println("Localization method changed to: ODOMETRY + IMU + RFID");
        Serial.print("Current yaw-angle: ");
        Serial.println(yaw_prev); 
        break;
      }            
    }
  }
}
void change_state(int arg1){
  // Stop all motors
  if(!DC_STOP){
    DC_STOP = allWheelsSTOP();
  }
  if(!Stepper_STOP){
    Stepper_STOP = allSteppersSTOP();
  }
  delay(500);
  state = arg1;
  switch(arg1){    
    case 0:
    {
      break; 
    }
    case 1: 
    {
      change_loc_method(0);
      break;
    }
    case 2:
    {
      break;
    }
    case 3: 
    {
      change_loc_method(0);
      break;
    }      
    case 4: 
    {
      change_loc_method(2);
      break;
    }  
    case 5: 
    {
      change_loc_method(0);
      break;
    }          
  }
  if(print_to_COM){
    switch(arg1){
      case 0: 
      {
        Serial.println("State changed to STANDBY");
        break;
      }      
      case 1: 
      {
        Serial.println("State changed to DRIVING");
        Serial.print("Each movement is: ");
        Serial.print(drive_time);
        Serial.println(" milliseconds long");
        Serial.println("Default localization method: odometry");
        break;
      }    
      case 2: 
      {
        Serial.println("State changed to FOLDING");
        break;
      }    
      case 3: 
      {
        Serial.println("State changed to REMOTE CONTROL");
        Serial.println("Default localization method: odometry");
        break;
      }      
      case 4: 
      {
        Serial.println("State changed to LAYOUT");
        Serial.println("Default localization method: odometry + IMU + RFID");
        break;
      }  
      case 5: 
      {
        Serial.println("State changed to PID POSITION CONTROL");
        Serial.println("Default localization method: odometry + IMU + RFID");
         break;
      } 
      default: {Serial.println("Not a valid Mode!!");break;}         
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
void doEncoder_1() {
  Encoder_1.update();
}
void doEncoder_2() {
  Encoder_2.update();
}
void doEncoder_3() {
  Encoder_3.update();
}
void doEncoder_4() {
  Encoder_4.update();
}

//////////////////////////////////////////////////////////////////////////////////////
static inline int8_t sgn(float val){
  if (val<0) return -1;
  return 1;
}
//////////////////////////////////////////////////////////////////////////////////////
// calculate wheel speeds form encoders
void getEnconderSpeeds(int time_diff){
  // Get speed in rad/sec from encoders!! (WITHOUT SIGN)
  w_is[0] = Encoder_1.calcSpeed(time_diff);
  Encoder_1.count = 0;
  w_is[1] = Encoder_2.calcSpeed(time_diff);
  Encoder_2.count = 0;
  w_is[2] = Encoder_3.calcSpeed(time_diff);
  Encoder_3.count = 0;
  w_is[3] = Encoder_4.calcSpeed(time_diff);
  Encoder_4.count = 0;
  //Add Sign assuming all wheels rotate with the correct direction
  for(int i=0; i<4; i++){
    if(w[i]<0){
      w_is_sign[i] = -w_is[i];
    }
    else {
      w_is_sign[i] = w_is[i];
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Run DC motors
void run_DC(){
  PID_DC1.Compute();
  PID_DC2.Compute();
  PID_DC3.Compute();
  PID_DC4.Compute();
  // Map wheel speeds to value between 0 and 255 to run DC-motors
  DC_1.map_wheelspeed(w[0],PID_Out_DC[0]);
  DC_2.map_wheelspeed(w[1],PID_Out_DC[1]);
  DC_3.map_wheelspeed(w[2],PID_Out_DC[2]);
  DC_4.map_wheelspeed(w[3],PID_Out_DC[3]);

  // Run DC-motors
  digitalWrite(DC_1.pin_dir, DC_1.dir);
  analogWrite(DC_1.pin_speed, DC_1.mapped_speed);
  digitalWrite(DC_2.pin_dir, DC_2.dir);
  analogWrite(DC_2.pin_speed, DC_2.mapped_speed);
  digitalWrite(DC_3.pin_dir, DC_3.dir);
  analogWrite(DC_3.pin_speed, DC_3.mapped_speed);
  digitalWrite(DC_4.pin_dir, DC_4.dir);
  analogWrite(DC_4.pin_speed, DC_4.mapped_speed);
}
              
//////////////////////////////////////////////////////////////////////////////////////
bool allWheelsSTOP(){
  digitalWrite(DC_1.pin_dir, 0);
  analogWrite(DC_1.pin_speed, 0);
  digitalWrite(DC_2.pin_dir, 0);
  analogWrite(DC_2.pin_speed, 0);
  digitalWrite(DC_3.pin_dir, 0);
  analogWrite(DC_3.pin_speed, 0);
  digitalWrite(DC_4.pin_dir, 0);
  analogWrite(DC_4.pin_speed, 0);
  delay(50);
  getEnconderSpeeds(50);
  delay(50);
  getEnconderSpeeds(50);
  v[0]=0;v[1]=0;v[2]=0;
  SRTMecanum.CalcWheelSpeeds((float*)v, (float*)w);
  SRTMecanum.WheelSpeeds_NoSign((float*)w, (float*)w_should);
  PID_Out_DC[0] = 0;
  PID_Out_DC[1] = 0;
  PID_Out_DC[2] = 0;
  PID_Out_DC[3] = 0;
  DC_1.map_wheelspeed(w[0],PID_Out_DC[0]);
  DC_2.map_wheelspeed(w[1],PID_Out_DC[1]);
  DC_3.map_wheelspeed(w[2],PID_Out_DC[2]);
  DC_4.map_wheelspeed(w[3],PID_Out_DC[3]);
  PID_DC1.Initialize();
  PID_DC2.Initialize();
  PID_DC3.Initialize();
  PID_DC4.Initialize();
  return true;
  }

//////////////////////////////////////////////////////////////////////////////////////
bool allSteppersSTOP(){
  digitalWrite(Wing_R_step, LOW);
  digitalWrite(Wing_L_step, LOW);
  digitalWrite(Seat_R_step, LOW);
  digitalWrite(Seat_M_step, LOW);
  digitalWrite(Seat_L_step, LOW);
  return true;
  }
  

//////////////////////////////////////////////////////////////////////////////////////

void get_IMU_yaw(){
  fifoCount = mpu.getFIFOCount();
  while(fifoCount < packetSize){
    fifoCount = mpu.getFIFOCount();
  }
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  if(ypr[0]<0){
    ypr[0] += 2*pi ;
  }
  ypr[0] = 2*pi - ypr[0];
  mpu.resetFIFO();
}
//////////////////////////////////////////////////////////////////////////////////////
void Init_IMU_RFID(){
  // MPU Setup:
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  // initialize device
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(101); //102
  mpu.setYGyroOffset(-28); //-29
  mpu.setZGyroOffset(39); //39
  mpu.setZAccelOffset(1509); // 1510

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  
  //RFID Setup:
  // Initialize SPI bus
  SPI.begin();
  mfrc522.PCD_Init(); //Initialize MFRC522 card
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////// LOCALIZATION ALGORITHM /////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//time_prev NEEDED for localization: Before starting localization always ste time_prev to millis!!
void localize_Robot(){
  time_diff = millis() - time_prev;
  time_diff_IMU = millis() - time_prev_IMU;
  // Localization
  if (time_diff > encoder_frequency){ 
    //Calculate wheel speeds 
    getEnconderSpeeds(time_diff);
    // Calculate position via odometry
    Robot_Pose.newPoseOdometry((float*)w_is_sign, SRTMecanum, time_diff);
    time_prev = millis();
    switch(loc_method){
      case 0: //odometry
      {
        // new pose is odometry pose
        //Serial.println(Robot_Pose.odometryPose[0]);
        Robot_Pose.setglobalPose(Robot_Pose.odometryPose[0], Robot_Pose.odometryPose[1], Robot_Pose.odometryPose[2]);
        break;
      }
      case 1: //odometry + IMU
      case 2: //odometry + IMU + RFID
      {
		    if(time_diff_IMU > IMU_frequency){
    			// read a packet from FIFO of IMU
    			get_IMU_yaw(); //writes to ypr[0];
    			//new angle according to IMU
    			Robot_Pose.newAngleIMU(yaw_prev, ypr[0]);
    			// Claculate new Pose by fusing yaw angle from IMU with odometry data
    			KalmanOdoIMU.calcNewState((float*)w_is_sign, SRTMecanum, Robot_Pose, time_diff); // time_diff !!! Don't use time_diff_IMU !!!
          time_prev_IMU = millis();
    			Robot_Pose.setglobalPose(KalmanOdoIMU.new_State[0], KalmanOdoIMU.new_State[1], KalmanOdoIMU.new_State[2]);
    			yaw_prev = ypr[0];
		    }
    		else {
    			Robot_Pose.setglobalPose(Robot_Pose.odometryPose[0], Robot_Pose.odometryPose[1], Robot_Pose.odometryPose[2]);
    		}
        break;
      }
    }   
    
    // Print localization results
    if(print_to_COM){
      Print_Serial_Localization();
    }
  }
  // Include RFID readings if loc_method==2
  if(loc_method==2){
    // Tag reading
    buffer_size = sizeof(buffer);
    // Read NFC tag if available
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()){
      status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, buffer, &buffer_size);
      if (status != MFRC522::STATUS_OK) {
        if(print_to_COM){
          Serial.print(F("MIFARE_Read() failed: "));
          Serial.println(mfrc522.GetStatusCodeName(status));
        }
      }    
      float tag_x;
      float tag_y;
      KalmanNFC.newTagdetected(tag_x, tag_y);
      if(print_to_COM){
        Serial.print("Tag: x = ");
        Serial.print(tag_x);
        Serial.print("  y = ");
        Serial.print(tag_y);
        Serial.println("   detected");
      }
      KalmanNFC.orientationRobot(Robot_Pose);
      KalmanNFC.calcNewState(Robot_Pose);
      Robot_Pose.setglobalPose(KalmanNFC.new_State[0], KalmanNFC.new_State[1], KalmanNFC.new_State[2]);
    }
    mfrc522.PICC_HaltA(); 
  }
}



void Print_Serial_Localization(){
  /*switch(loc_method){
    case 0: {Serial.println("Pose odometry:   x   y   \theta");}
    case 1: {Serial.println("Pose odometry:   x   y   \theta    Angle IMU:  \theta    Pose Kalman:    x   y   \theta");}
    case 2: {Serial.println("Pose odometry:   x   y   \theta    Angle IMU:  \theta    Pose Kalman:    x   y   \theta    Pose Kalman RFID    x   y   \theta");}    
  }*/
  switch(loc_method){
    case 0:
    {
      Serial.print("\t\t");
      Serial.print(Robot_Pose.globalPose[0]);
      Serial.print("\t");
      Serial.print(Robot_Pose.globalPose[1]);
      Serial.print("\t");
      Serial.println(Robot_Pose.globalPose[2]);
      break;      
    }
    case 1:
    {
      Serial.print("\t\t");
      Serial.print(Robot_Pose.odometryPose[0]);
      Serial.print("\t");
      Serial.print(Robot_Pose.odometryPose[1]);
      Serial.print("\t");
      Serial.print(Robot_Pose.odometryPose[2]);
      Serial.print("\t\t\t");
      Serial.print(Robot_Pose.IMU_angle);
      Serial.print("\t\t\t");
      Serial.print(Robot_Pose.globalPose[0]);
      Serial.print("\t");
      Serial.print(Robot_Pose.globalPose[1]);
      Serial.print("\t");
      Serial.println(Robot_Pose.globalPose[2]);
      break;
    }
    case 2:
    {
      Serial.print("\t\t");
      Serial.print(Robot_Pose.odometryPose[0]);
      Serial.print("\t");
      Serial.print(Robot_Pose.odometryPose[1]);
      Serial.print("\t");
      Serial.print(Robot_Pose.odometryPose[2]);
      Serial.print("\t\t\t");
      Serial.print(Robot_Pose.IMU_angle);
      Serial.print("\t\t\t");
      Serial.print(KalmanOdoIMU.new_State[0]);
      Serial.print("\t");
      Serial.print(KalmanOdoIMU.new_State[1]);
      Serial.print("\t");
      Serial.print(KalmanOdoIMU.new_State[2]);
      Serial.print("\t\t\t");
      Serial.print(Robot_Pose.globalPose[0]);
      Serial.print("\t");
      Serial.print(Robot_Pose.globalPose[1]);
      Serial.print("\t");
      Serial.println(Robot_Pose.globalPose[2]);
      break;
    }
  }
}
