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
float l_x = 455/2;//227.5;
float l_y = 455/2;
float R = 30;


// translation of gearbox
const float translation = 74.831;
// Encoder sample time in milliseconds (is equal to odometry -> and fusion with IMU)
const int encoder_frequency = 50;
// Encoder counts per round
const float counts_per_round = 0.5*3591.84;
// PID sample time
const int PID_sample_time = 50;

// Steps, Step times and step delays for
const int steps_Wings = 49*51;
const int steps_Seats = 500; 
const int stepTime_Wings = 8000; // With gearbox:8000; //Microseconds
const int stepDelay_Wings = 1; // With gearbox: 1; //Milliseconds
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
float yaw_prev_IMU;         // Previos yaw angle by IMU before one driving part
float yaw_prev_Robot;       // Previous yaw angle of Robot before one driving part

// Create RFID reader object
MFRC522 mfrc522(SDA_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::StatusCode status; //variable to get card status

// RFID variables
char read_buffer[18];  //data transfer buffer (16+2 bytes data+CRC)
byte read_size = sizeof(read_buffer);
uint8_t pageAddr = 0x04; //read 16 bytes (page 4,5,6 and 7).

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
bool Magnets_ACTIVE;

// Create Encoders
Encoder Encoder_1(counts_per_round);
Encoder Encoder_2(counts_per_round);
Encoder Encoder_3(counts_per_round);
Encoder Encoder_4(counts_per_round);

// Encoder timing
unsigned long time;
unsigned long time_prev = 0;
int time_diff;
unsigned long time_prev_IMU;


// Speeds
double v[3]; // desired speed in local frame
double w[4]; // desired rotational wheel speed WITH SIGN
double w_should[4]; // Desired rotational wheel speeds WITHOUT SIGN
double w_is[4]; // measured rotational wheel speeds WITHOUT SIGN
double w_is_sign[4]; // measured rotational wheel speeds WITH SIGN

// Create PID for DC motors // Output limit is 0-255 by default
double PID_Out_DC[4]; //Output of PID controler
double Kp_DC = 5, Ki_DC = 100, Kd_DC = 0;
PID PID_DC1(&w_is[0], &PID_Out_DC[0], &w_should[0], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC2(&w_is[1], &PID_Out_DC[1], &w_should[1], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC3(&w_is[2], &PID_Out_DC[2], &w_should[2], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC4(&w_is[3], &PID_Out_DC[3], &w_should[3], Kp_DC, Ki_DC, Kd_DC, DIRECT);
double zero = 0;
// Create PID for position Control
double Kp_Pose_xy = 2, Ki_Pose_xy = 0.8, Kd_Pose_xy = 0;
PID PID_x(&zero, &v[0], &Robot_Pose.toGo_local[0], Kp_Pose_xy, Ki_Pose_xy, Kd_Pose_xy, DIRECT);
double Kp_Pose_theta = 1.7, Ki_Pose_theta = 0.6, Kd_Pose_theta = 0;
PID PID_theta(&Robot_Pose.globalPose[2], &v[2], &Robot_Pose.globalPose_should[2], Kp_Pose_theta, Ki_Pose_theta, Kd_Pose_theta, DIRECT);
PID PID_heading(&Robot_Pose.globalPose[2], &v[2], &Robot_Pose.heading_angle, Kp_Pose_theta, Ki_Pose_theta, Kd_Pose_theta, DIRECT);

// Maximal Speeds in mm/s and rad/s
const double v_max_x = 300;
const double v_max_y = 300;
const double v_max_theta = 0.6;

// Distances/angle at which PID takes over
const double PID_dis_xy = 50;
const double PID_dis_theta = 0.3;
// Allowed Pose Error:
const float Pose_error_xy = 10;
const float Pose_error_theta = 0.01;

// Print settings
bool print_to_COM = true;// true; //Print data da serial port (computer)
bool print_commands = false;
// Arguments from remote control can either be in local frame or global frame:
bool remote_local = true;
// Use NFC tags for angle estimation
bool NFC_angle = false;
float Reader_Offset[3] = {0,-80,0};

// States of the robot
int state;
/* switch(state){
 *  case(0): standby -> no mode selected, no movement, INITIAL Pose can be set
 *  case(1): driving -> Robot expects commands to move with velocity vector for a certain time --> int drive_time
 *  case(2): folding -> Robot expects commands to fold
 *  case(3): analog remote controll
 *  case(4): not implemented: predefined configurations
 *  case(5): PID position control
 */
 int drive_time = 3000; // drive for 3000 milliseconds

// layouts / configurations
 int layout;
 /* switch(layout){
  *  case(0): private/moving
  *  case(1): efficiency
  *  case(2): communication
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
  
  KalmanNFC.NFC_angle = NFC_angle;
  KalmanNFC.Reader_Offset[0] = Reader_Offset[0];
  KalmanNFC.Reader_Offset[1] = Reader_Offset[1];
  KalmanNFC.Reader_Offset[2] = 0;
  
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
  Serial.begin(38400);
  
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
  //Active magnets from start
  digitalWrite(magnets, LOW);
    
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
  PID_theta.SetMode(AUTOMATIC);
  PID_theta.SetSampleTime(PID_sample_time);
  PID_heading.SetMode(AUTOMATIC);
  PID_heading.SetSampleTime(PID_sample_time);
  PID_x.SetOutputLimit(-0.5*v_max_x, 0.5*v_max_x);
  PID_theta.SetOutputLimit(-0.5*v_max_theta, 0.5*v_max_theta);
  PID_heading.SetOutputLimit(-0.5*v_max_theta, 0.5*v_max_theta);
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
          set_global_pose((float)arg1,(float)arg2,(float)arg3);
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
          set_global_pose((float)arg1,(float)arg2,(float)arg3);
          print_globalPose();
          break;
        }
        case 'M': //Moving with velocities defined by arg1-3 for a certain time
        {
          Magnets_ACTIVE = deactivate_Magnets();
          DC_STOP = false;
          if(layout!=0){
            if(print_to_COM){
              Serial.println("Bevore moving change into privacy/moving configuration!!");
              break;
            }
          }
          // Calculations of wheel speeds (speeds are local)
          v[0] = v_max_x*arg1/100;
          v[1] = v_max_y*arg2/100;
          v[2] = v_max_theta*arg3/100;
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
          yaw_prev_IMU = ypr[0];
          yaw_prev_Robot = Robot_Pose.globalPose[2];
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
          Magnets_ACTIVE = activate_Magnets();
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
          set_global_pose((float)arg1,(float)arg2,(float)arg3);
          print_globalPose();
          break;
        }
        case 'M': //Moving with velocities defined by arg1-3 for a certain time
        {
          Magnets_ACTIVE = deactivate_Magnets();
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
          yaw_prev_IMU = ypr[0];
          yaw_prev_Robot = Robot_Pose.globalPose[2];
          //time_prev NEEDED for localization: Before starting localization always ste time_prev to millis!!
          time_prev = millis();
          localize_Robot();
          while(command=='M'){            
            if(remote_local){  
              // Calculations of wheel speeds (speeds are local)
              v[0] = 0.7*v_max_x*arg1/100;
              v[1] = 0.7*v_max_y*arg2/100;
              v[2] = 0.7*v_max_theta*arg3/100;
            }
            else{ // if speeds are from a global perspective
              Robot_Pose.speed_to_local_v((float*)v, 0.7*v_max_x*arg1/100, 0.7*v_max_y*arg2/100, 0.7*v_max_theta*arg3/100);
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
            localize_Robot();  
            if(!DC_STOP){// Compute PID results for DC motors and run DC motors
              run_DC();
            }
            read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);         
          }
          localize_Robot();   
          Magnets_ACTIVE = activate_Magnets();      
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
        case 'K': 
        {
          change_layout(arg1);
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
          set_global_pose((float)arg1,(float)arg2,(float)arg3);
          print_globalPose();
          break;
        }
        case 'T': //'T': target position
        {
          // Arguments contain the desired position
          Robot_Pose.globalPose_should[0] = (double)arg1;
          Robot_Pose.globalPose_should[1] = (double)arg2;
          Robot_Pose.globalPose_should[2] = ((double)arg3)*pi/180;
          if(print_to_COM){
            Serial.print("Heading to x: ");
            Serial.print(arg1);
            Serial.print("\t y: ");
            Serial.print(arg2);
            Serial.print("\t theta: ");
            Serial.println(arg3);
          }
          GoToTarget();
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
    {   
      // Check IMU
      if(!dmpReady){
        if(print_to_COM){
        Serial.println("IMU not ready");
        }
        change_loc_method(0);//change to odometry only 
        return;
      }
      //Get current yaw angle
      mpu.resetFIFO();
      // read a packet from FIFO of IMU
      get_IMU_yaw();
      yaw_prev_IMU = ypr[0];
      break;
    }
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
      if(print_to_COM){
        Serial.print("Virtual Tag: ");
        Serial.print(KalmanNFC.tag_det[0]);
        Serial.print("\t");
        Serial.println(KalmanNFC.tag_det[1]);
      }
      //Get current yaw angle
      mpu.resetFIFO();
      // read a packet from FIFO of IMU
      get_IMU_yaw();
      yaw_prev_IMU = ypr[0];
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
        Serial.println(yaw_prev_IMU);
        break;
      }      
      case 2: 
      {
        Serial.println("Localization method changed to: ODOMETRY + IMU + RFID");
        Serial.print("Current yaw-angle: ");
        Serial.println(yaw_prev_IMU); 
        break;
      }            
    }
  }
}
void change_state(int arg1){
  //activate magnets -> no current
  if(!Magnets_ACTIVE){
    Magnets_ACTIVE = activate_Magnets();  
  }
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
      change_loc_method(2);
      break;
    }      
    case 4: 
    {
      change_loc_method(2);
      break;
    }  
    case 5: 
    {
      change_loc_method(2);
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
        break;
      }      
      case 4: 
      {
        Serial.println("State changed to LAYOUT");
        break;
      }  
      case 5: 
      {
        Serial.println("State changed to PID POSITION CONTROL");
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
  			// read a packet from FIFO of IMU
  			get_IMU_yaw(); //writes to ypr[0];
  			//new angle according to IMU
  			Robot_Pose.newAngleIMU(yaw_prev_Robot, yaw_prev_IMU, ypr[0]);
  			// Claculate new Pose by fusing yaw angle from IMU with odometry data
      
  			KalmanOdoIMU.calcNewState((float*)w_is_sign, SRTMecanum, Robot_Pose, time_diff); //
     //  debug_KalmanFus(); // For debugging Kalman Filter
  			Robot_Pose.setglobalPose(KalmanOdoIMU.new_State[0], KalmanOdoIMU.new_State[1], KalmanOdoIMU.new_State[2]);
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
    int tag_x = 0;
    int tag_y = 0;
    read_size = sizeof(read_buffer);
    // Tag reading
    // Read NFC tag if available
    if (mfrc522.PICC_IsNewCardPresent()){// && mfrc522.PICC_ReadCardSerial()){
      status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, read_buffer, &read_size);
      if (status != MFRC522::STATUS_OK) {
        if(print_to_COM){
          Serial.print(F("MIFARE_Read() failed: "));
          Serial.println(mfrc522.GetStatusCodeName(status));
        }
      }
      else{
        read_tag(&tag_x,&tag_y);
        KalmanNFC.newTagdetected((float)tag_x, (float)tag_y);
        KalmanNFC.offset_Reader(Robot_Pose);
        if(print_to_COM){
          Serial.print("Tag: x = ");
          Serial.print(tag_x);
          Serial.print("  y = ");
          Serial.print(tag_y);
          Serial.println("   detected");
        }
        if(KalmanNFC.tag_det[1]!=KalmanNFC.tag_prev[1]&&KalmanNFC.tag_det[0]!=KalmanNFC.tag_prev[0]){
          KalmanNFC.orientationRobot(Robot_Pose);
          KalmanNFC.calcNewState(Robot_Pose);
          //debug_Kalman_NFC();
          Robot_Pose.setglobalPose(KalmanNFC.new_State[0], KalmanNFC.new_State[1], KalmanNFC.new_State[2]);
          // Reset IMU if NFC angle calculation
          if(NFC_angle){   
            get_IMU_yaw();
            yaw_prev_IMU = ypr[0];
            yaw_prev_Robot = Robot_Pose.globalPose[2];
          }
          // RESET other Localization algorithm: 
          getEnconderSpeeds(time_diff); 
          time_prev = millis();
        }
      }
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
      //Serial.println(time_diff);
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

void debug_KalmanFus(){
  Serial.println(time_diff, DEC);
  /*
  Serial.print("w = ");
  Serial.print(w_is_sign[0], DEC);
  Serial.print("\t");
  Serial.print(w_is_sign[1], DEC);
  Serial.print("\t");
  Serial.print(w_is_sign[2], DEC);
  Serial.print("\t");
  Serial.println(w_is_sign[3], DEC);
  
  Serial.print("Old pose = ");
  Serial.print(Robot_Pose.globalPose[0], DEC);
  Serial.print("\t");
  Serial.print(Robot_Pose.globalPose[1], DEC);
  Serial.print("\t");
  Serial.println(Robot_Pose.globalPose[2], DEC);

  Serial.print("Odometry Pose = ");
  Serial.print(Robot_Pose.odometryPose[0], DEC);
  Serial.print("\t");
  Serial.print(Robot_Pose.odometryPose[1], DEC);
  Serial.print("\t");
  Serial.println(Robot_Pose.odometryPose[2], DEC);
  
  Serial.print("Pose_pred = ");
  Serial.print(KalmanOdoIMU.Pose_pred[0], DEC);
  Serial.print("\t");
  Serial.print(KalmanOdoIMU.Pose_pred[1], DEC);
  Serial.print("\t");
  Serial.println(KalmanOdoIMU.Pose_pred[2], DEC);

  Serial.print("z_pred = ");
  Serial.println(KalmanOdoIMU.z_pred, DEC);

  Serial.print("z_is = ");
  Serial.println(Robot_Pose.IMU_angle, DEC);

  Serial.println("Jacobi_F = ");
  Serial.print(KalmanOdoIMU.Jacobi_F[0][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_F[0][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_F[0][2], DEC);
   Serial.print(KalmanOdoIMU.Jacobi_F[1][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_F[1][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_F[1][2], DEC);
   Serial.print(KalmanOdoIMU.Jacobi_F[2][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_F[2][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_F[2][2], DEC);

    Serial.println("Jacobi_W = ");
  Serial.print(KalmanOdoIMU.Jacobi_W[0][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_W[0][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_W[0][2], DEC);
   Serial.print(KalmanOdoIMU.Jacobi_W[1][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_W[1][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_W[1][2], DEC);
   Serial.print(KalmanOdoIMU.Jacobi_W[2][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Jacobi_W[2][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Jacobi_W[2][2], DEC);
  
  Serial.println("P_pred = ");
  Serial.print(KalmanOdoIMU.P_pred[0][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P_pred[0][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P_pred[0][2], DEC);
   Serial.print(KalmanOdoIMU.P_pred[1][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P_pred[1][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P_pred[1][2], DEC);
   Serial.print(KalmanOdoIMU.P_pred[2][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P_pred[2][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P_pred[2][2], DEC);
      */
  Serial.println("P = ");
  Serial.print(KalmanOdoIMU.P[0][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P[0][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P[0][2], DEC);
   Serial.print(KalmanOdoIMU.P[1][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P[1][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P[1][2], DEC);
   Serial.print(KalmanOdoIMU.P[2][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.P[2][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.P[2][2], DEC);

   Serial.print("K-Gain = ");
   Serial.print(KalmanOdoIMU.Kalman_Gain[0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.Kalman_Gain[1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.Kalman_Gain[2], DEC);
/*
   Serial.print("New_Pose");
   Serial.print(KalmanOdoIMU.new_State[0], DEC);
   Serial.print("\t");
   Serial.print(KalmanOdoIMU.new_State[1], DEC);
   Serial.print("\t");
   Serial.println(KalmanOdoIMU.new_State[2], DEC);

   Serial.println("\n");*/   
}

void read_tag(int * tag_x, int * tag_y){
  int i= 0;
  while (read_buffer[i] != '|') {
    *tag_x *= 10;
    *tag_x= *tag_x+ (read_buffer[i] - 48);
    i++;
  }
  i++;
  
  while (read_buffer[i] != '|') {
    *tag_y *= 10;
    *tag_y = *tag_y + (read_buffer[i] - 48);
    i++;
  }
}

void debug_Kalman_NFC(){
  Serial.print("Previous Tag = ");
  Serial.print(KalmanNFC.tag_prev[0]);
  Serial.print("\t");
  Serial.println(KalmanNFC.tag_prev[1]);
  
  Serial.print("Robot in Tag Zone = ");
  Serial.println(KalmanNFC.checkRobotinZone(Robot_Pose));

  Serial.println("P = ");
  Serial.print(KalmanNFC.P[0][0], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.P[0][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.P[0][2], DEC);
   Serial.print(KalmanNFC.P[1][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanNFC.P[1][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.P[1][2], DEC);
   Serial.print(KalmanNFC.P[2][0], DEC);
   Serial.print("\t");
   Serial.print(KalmanNFC.P[2][1], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.P[2][2], DEC);

  Serial.println("Kalman_Gain = ");
  Serial.print(KalmanNFC.Kalman_Gain[0][0], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.Kalman_Gain[0][1], DEC);
   Serial.print(KalmanNFC.Kalman_Gain[1][0], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.Kalman_Gain[1][1], DEC);
   Serial.print(KalmanNFC.Kalman_Gain[2][0], DEC);
   Serial.print("\t");
   Serial.println(KalmanNFC.Kalman_Gain[2][1], DEC);

   Serial.print("Correction = ");
   Serial.print(KalmanNFC.correction[0]);
   Serial.print("\t");
   Serial.println(KalmanNFC.correction[1]);

}

void set_global_pose(float arg1,float arg2,float arg3){
  Robot_Pose.setglobalPose(arg1,arg2,arg3);
  print_globalPose();
  // Set detected tag to current position --> "Virtual tags"
  KalmanNFC.tag_det[0] = Robot_Pose.globalPose[0];
  KalmanNFC.tag_det[1] = Robot_Pose.globalPose[1]; 
  if(print_to_COM){
    Serial.print("Virtual Tag: ");
    Serial.print(KalmanNFC.tag_det[0]);
    Serial.print("\t");
    Serial.println(KalmanNFC.tag_det[1]);
  }
}

//wings first argument: both(2) or left(1) or right(0) wing, second argument open(1) or close(0)
//for chair(seat): first argument Seat (R&L all left middle right) -> (4 3 2 1 0) & second argument direction (up down): (1 0)

/* layout:
 * 0 = privacy
 * 1 = efficiency
 * 2 = communication
 */

void change_layout(int arg1){
  float pos_privacy[3] = {1,1,1};
  float pos_efficiency[3] = {1,1,1};
  float pos_communication[3] = {1,1,1};
  
  switch(layout){
    case 0:
    {
      switch(arg1){
        case 0:
        {
          if(print_to_COM){
            Serial.println("You are in Privacy Mode");
          }
          break;
        }
        case 1:
        {
          do_Wings(2,1);
          do_Seats(1,1);
          layout = 1;
          if(print_to_COM){
            Serial.println("Layout changed to Efficiency");
          }
          break;          
        }
        case 2:
        {
          Robot_Pose.globalPose_should[0] = pos_communication[0];
          Robot_Pose.globalPose_should[1] = pos_communication[1];
          Robot_Pose.globalPose_should[2] = pos_communication[2];
          GoToTarget();
          do_Wings(2,1);
          do_Seats(4,0);
          layout = 2;
          if(print_to_COM){
            Serial.println("Layout changed to Communication");
          }
          break;
        }
        default:
        {
          if(print_to_COM){
            Serial.println("Not a valid layout chosen");
          }
          break;
        }
      }
      break;
    }
    case 1:
    {
      switch(arg1){
        case 0:
        { 
          do_Seats(1,0);
          do_Wings(2,0);
          layout = 0;
          if(print_to_COM){
            Serial.println("Layout changed to Privacy");
          }
          break;
        }
        case 1:
        {
          if(print_to_COM){
            Serial.println("You are in Efficiency Mode");
          }
          break;
        }
        case 2:
        {
          do_Seats(1,0);
          do_Wings(2,0);
          Robot_Pose.globalPose_should[0] = pos_communication[0];
          Robot_Pose.globalPose_should[1] = pos_communication[1];
          Robot_Pose.globalPose_should[2] = pos_communication[2];
          GoToTarget();
          do_Wings(2,1);
          do_Seats(4,0);
          layout = 2;
          if(print_to_COM){
            Serial.println("Layout changed to Communication");
          }
          break;
        }
        default:
        {
          if(print_to_COM){
            Serial.println("Not a valid layout chosen");
          }
          break;
        }
      }
      break;
    }
    case 2:
    {
      switch(arg1){
        case 0:
        { 
          do_Seats(4,1);
          do_Wings(2,0);
          Robot_Pose.globalPose_should[0] = pos_privacy[0];
          Robot_Pose.globalPose_should[1] = pos_privacy[1];
          Robot_Pose.globalPose_should[2] = pos_privacy[2];
          layout = 0;
          if(print_to_COM){
            Serial.println("Layout changed to Privacy Mode");
          }
          break;
        }
        case 1:
        {
          do_Seats(4,1);
          do_Wings(2,0);
          Robot_Pose.globalPose_should[0] = pos_efficiency[0];
          Robot_Pose.globalPose_should[1] = pos_efficiency[1];
          Robot_Pose.globalPose_should[2] = pos_efficiency[2];
          do_Wings(2,1);
          do_Seats(1,1);
          layout = 1;
          if(print_to_COM){
            Serial.println("Layout changed to Efficiency Mode");
          }
          break;
        }
        case 2:
        {
          if(print_to_COM){
            Serial.println("You are in Communication Mode");
          }
          break;
        }
        default:
        {
          if(print_to_COM){
            Serial.println("Not a valid layout chosen");
          }
          break;
        }
      }
      break;
    }
    default:
    {
      if(print_to_COM){
        Serial.println("You are not in a valid layout!");
      }
      break;
    }
  } 
}

void GoToTarget(){
  Magnets_ACTIVE = deactivate_Magnets();
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
  yaw_prev_IMU = ypr[0];
  yaw_prev_Robot = Robot_Pose.globalPose[2];
  // Calculated distance to go in local system         
  time = millis();
  time_prev = millis();
  localize_Robot();
  Robot_Pose.calctoGo_local();

  while((fabs(Robot_Pose.toGo_local[0])>Pose_error_xy||fabs(Robot_Pose.toGo_local[1])>Pose_error_xy )){
    Robot_Pose.calctoGo_local();
    
    // ROTATE to head to target
    DC_STOP = false;
    PID_heading.Initialize();
    Serial.println("Rotate to head to target");
    while(fabs(Robot_Pose.globalPose[2]-Robot_Pose.heading_angle)>Pose_error_theta){
      if(fabs(Robot_Pose.globalPose[2]-Robot_Pose.heading_angle)<PID_dis_theta){
        PID_heading.Compute();
      }
      else if(fabs(Robot_Pose.globalPose[2]-Robot_Pose.heading_angle)>pi){
        v[2] = -0.5*sgn(Robot_Pose.heading_angle-Robot_Pose.globalPose[2])*v_max_theta;
      }
      else{
        v[2] = 0.5*sgn(Robot_Pose.heading_angle-Robot_Pose.globalPose[2])*v_max_theta;
      }
      // Drive Robot
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
    }
    DC_STOP = allWheelsSTOP();
    
    
    // DRIVE Forward
    DC_STOP = false;
    PID_x.Initialize();
    Serial.println("Driving forward");
    while(fabs(Robot_Pose.toGo_local[0])>Pose_error_xy){
      if(fabs(Robot_Pose.toGo_local[0])<PID_dis_xy){
        PID_x.Compute();
      }
      else{
        v[0] = 0.5*sgn(Robot_Pose.toGo_local[0])*v_max_x;
      }
      // Drive Robot
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
    }
    DC_STOP = allWheelsSTOP();
  }
  

  // ROTATE
  DC_STOP = false;
  PID_theta.Initialize();
  Serial.println("Rotate to final position");
  while(fabs(Robot_Pose.toGo_local[2])>Pose_error_theta){
    if(fabs(Robot_Pose.toGo_local[2])<PID_dis_theta){
      PID_theta.Compute();
    }
    else if(fabs(Robot_Pose.toGo_local[2])>pi){
      v[2] = -0.5*sgn(Robot_Pose.toGo_local[2])*v_max_theta;
    }
    else{
      v[2] = 0.5*sgn(Robot_Pose.toGo_local[2])*v_max_theta;
    }

    // Drive Robot
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
  }
  DC_STOP = allWheelsSTOP();
  Magnets_ACTIVE = activate_Magnets();
  
  if(print_to_COM){
    Serial.println("Position reached!!");
  }
}

bool activate_Magnets(){
  digitalWrite(magnets,LOW);
  return true;
}
bool deactivate_Magnets(){
  digitalWrite(magnets,HIGH);
  return false;
}


