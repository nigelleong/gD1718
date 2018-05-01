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

// Pins for RFID Reader
const int SDA_PIN = 53;
const int RST_PIN = 5;
// SCK to 52; MOSI to 51; MISO to 50; 

// Bluetooth connection pins 10 and 11
SoftwareSerial BT(10, 11); 

// Define pins for DC-motors
const int DC_M1_Dir = 22;
const int DC_M2_Dir = 24;
const int DC_M3_Dir = 26;
const int DC_M4_Dir = 28;
const int DC_M1_Speed = 8;
const int DC_M2_Speed = 9;
const int DC_M3_Speed = 12;
const int DC_M4_Speed = 13;

// Define pins for stepper motors
/* direction pins: 
 *  HIGH: open wing and fold seat upwards
 *  LOW: close wing and fold seat downward 
 */
const int sleep_Stepper = 43;
const int Wing_R_dir = 25;
const int Wing_L_dir = 29;
const int Seat_R_dir = 33;
const int Seat_M_dir = 37;
const int Seat_L_dir = 41;
const int Wing_R_step = 23;
const int Wing_L_step = 27;
const int Seat_R_step = 31;
const int Seat_M_step = 35;
const int Seat_L_step = 39;

// Pin to deactivate magnets
const int magnets = 45;

// Encoders
const int Encoder_1_pin = 2;
const int Encoder_2_pin = 3;
const int Encoder_3_pin = 18;
const int Encoder_4_pin = 19;

// Dimensions of Robot
float l_x = 225.5;
float l_y = 217.5;
float R = 60;

// Dimenstions of operation area:
float area_x = 1250;
float area_y = 1250;
float safe_dis = 325; // safety distance to the edge of the stage

// Maximal Speeds in mm/s and rad/s
const float v_max_x = 10;
const float v_max_y = 10;
const float w_max = 1;

// translation of gearbox
const float translation = 74.831;
// Encoder sample time in milliseconds (is equal to odometry -> and fusion with IMU)
const int encoder_frequency = 50;
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

bool DC_STOP = true; 
bool Stepper_STOP = true; 

// Create Encoders
Encoder Encoder_1(counts_per_round);
Encoder Encoder_2(counts_per_round);
Encoder Encoder_3(counts_per_round);
Encoder Encoder_4(counts_per_round);

// Encoder timing
unsigned long time;
unsigned long time_prev = 0;
int time_diff;

// For Output
int Serial_diff_time;
unsigned long Serial_time = 0;

// Speeds
double v[3]; // desired speed in local frame
double w[4]; // desired rotational wheel speed WITH SIGN
double w_should[4]; // Desired rotational wheel speeds WITHOUT SIGN
double w_is[4]; // measured rotational wheel speeds WITHOUT SIGN
double w_is_sign[4]; // measured rotational wheel speeds WITH SIGN
double PID_Out_DC[4]; //Output of PID controler

// Create PID for DC motors // Output limit is 0-255 by default
double Kp_DC = 10, Ki_DC = 90, Kd_DC = 0;
PID PID_DC1(&w_is[0], &PID_Out_DC[0], &w_should[0], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC2(&w_is[1], &PID_Out_DC[1], &w_should[1], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC3(&w_is[2], &PID_Out_DC[2], &w_should[2], Kp_DC, Ki_DC, Kd_DC, DIRECT);
PID PID_DC4(&w_is[3], &PID_Out_DC[3], &w_should[3], Kp_DC, Ki_DC, Kd_DC, DIRECT);

// Create PID for position Control
/*double Kp_Pose = 1, Ki_Pose = 1, Kd_Pose = 0, Kf_Pose = 0;
PID PID_x(
PID PID_y(
PID PID_theta(
// Set Output limits:
PID_x.SetOutputLimits(-v_max_x, v_max_x);
PID_y.SetOutputLimits(-v_max_y, v_max_y);
PID_theta.SetOutputLimits(-w_max, w_max);*/

bool print_to_COM = false; //Print data da serial port (computer)

// States of the robot
int state;
/* switch(state){
 *  case(0): standby -> no mode selected, no movement, INITIAL Pose can be set
 *  case(1): driving -> Robot expects commands to move with velocity vector for a certain time
 *  case(3): folding -> Robot expects commands to fold
 *  case(4): not implemented: analog remote controll
 *  case(5): not implemented: predefined configurations
 *  case(6): not implemented: PID position control
 */

// layouts / configurations
 int layout;
 /* switch(layout){
  *  case(0): private/moving
  *  case(1): peak hour lean
  *  case(2): peak hour seat
 */
 
void setup() {
  state = 0;
  // Set speed in local frame
  v[0] = 0;
  v[1] = 0;
  v[2] = 0;
  // Set global initial pose:
  Robot_Pose.setglobalPose(0,0,0);
  Robot_Pose.setglobalPose_should(0,0,0);
  // Set initial yaw angle from IMU
  yaw_prev = Robot_Pose.globalPose[2];

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

/*  PID_x.SetMode(AUTOMATIC);
  PID_x.SetSampleTime(PID_sample_time);
  PID_y.SetMode(AUTOMATIC);
  PID_y.SetSampleTime(PID_sample_time);
  PID_theta.SetMode(AUTOMATIC);
  PID_theta.SetSampleTime(PID_sample_time);*/
}

void loop() {
  // Check IMU
  if(!dmpReady){
    if(print_to_COM){
    Serial.println("IMU not ready");
    }
    return;
  }

  switch(state){
    // STANDBY /////////////////////////////////////////////////
    case 0:{
      if(!DC_STOP){
        DC_STOP = allWheelsSTOP();
      }
      if(!Stepper_STOP){
        Stepper_STOP = allSteppersSTOP();
      }
      // Waiting for commands
      char  command_buffer[20]; // stores entire command
      char command = 'd'; //'d' = default
      int arg1 = 0;
      int arg2 = 0;
      int arg3 = 0;
      read_BT_command(command_buffer, &command, &arg1, &arg2, &arg3);
      //Check/print command:
      if(print_to_COM){
        print_Command(command_buffer, &command, &arg1, &arg2, &arg3);       
      }
      switch(command){
        case 'S': //'S'= switch -> switch to state defined in arg1 (see above)
        {
          state = arg1;          
        }
        default: {state = 0;}// Stay in STANDBY 
      }
    }
    ////////////////////////////////////////////////////////////
    // Driving /////////////////////////////////////////////////
    case 1:{
      
            
    }
    ////////////////////////////////////////////////////////////
    default: {Serial.println("ERROR: No state selected");}
  }
  // Tag reading
  buffer_size = sizeof(buffer);
  // Read NFC tag if available
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()){
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, buffer, &buffer_size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }    
/*    SET tag_det at the beginning!!!! (to starting position of robot)
    newTagdetected();
    orientationRobot();
    calcNewState();*/
    Serial.print(F("Read data: "));
    for (byte i = 0; i < 16; i++) {
      Serial.write(buffer[i]);
    }
    Serial.println();
  }
  mfrc522.PICC_HaltA();

  // Calculate desired wheels speeds WITH SIGN
  SRTMecanum.CalcWheelSpeeds((float*)v, (float*)w);
  // Desired wheel speeds WITHOUT SIGN (needed for PID controller
  SRTMecanum.WheelSpeeds_NoSign((float*)w, (float*)w_should);
  
  // Determine position derived from odometry and IMU
  fifoCount = mpu.getFIFOCount(); // Get size of FIFO buffer of IMU
  time = millis();
  time_diff = time - time_prev;
  if (time_diff > encoder_frequency && fifoCount > packetSize) { //time_diff has to have a certain size and IMU FIFO must be big enough
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
    include_sign(w, w_is, w_is_sign);

    // read a packet from FIFO of IMU
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("Yaw:\t");
    //Serial.println(ypr[0]*180/M_PI);
    mpu.resetFIFO();
      
    // Calculate position calculated from odometry
    Robot_Pose.newPoseOdometry((float*)w_is_sign, SRTMecanum, time_diff);
    Robot_Pose.newAngleIMU(yaw_prev, ypr[0]); //yaw_prev and ypr in degrees (euler) but result in radian!!
    // Claculate new Pose by fusing yaw angle from IMU with odometry data
    KalmanOdoIMU.calcNewState((float*)w_is_sign, SRTMecanum, Robot_Pose, time_diff);
    Robot_Pose.setglobalPose(KalmanOdoIMU.new_State[0], KalmanOdoIMU.new_State[1], KalmanOdoIMU.new_State[2]);
    
    time_prev = time; //Reset time to obtain the difference later
    yaw_prev = ypr[0]; //Reste yaw angle to obtain difference later
  }

  if(!DC_STOP){// Compute PID results for DC motors if robot should move
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
  else{
    for(int i=0;i<4;i++){
      PID_Out_DC[i] = 0;
    }
  } 

  Serial_diff_time = time - Serial_time;
  if(Serial_diff_time>1000){
    Print_Serial();
    Serial_time = time;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Read Bluetooth Commands
void read_BT_command(char* command_buffer, char *command, int *arg1, int *arg2, int *arg3){
  int input;
  int i = 0;
  while(1){
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
        Serial.println("Bluetooth not available");
      }      
    }
  }    
  *command = command_buffer[0];
  //Converts subsequent characters in the array into integer as function argument
  while (command_buffer[i] != '|') {
      *arg1 *= 10;
      *arg1 = *arg1 + (command_buffer[i] - 48);
      i++;
  }
  i++;
  while (command_buffer[i] != '|') {
      *arg2 *= 10;
      *arg2 = *arg2 + (command_buffer[i] - 48);
      i++;
  }
  i++;
  while (command_buffer[i] != '!') {
      *arg3 *= 10;
      *arg3 = *arg3 + (command_buffer[i] - 48);
      i++;
  }
} 
//////////////////////////////////////////////////////////////////////////////////////

void print_Command(char * command_buffer, char * command, int * arg1, int * arg2, int * arg3){
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
bool allWheelsSTOP(){
  digitalWrite(DC_1.pin_dir, 0);
  analogWrite(DC_1.pin_speed, 0);
  digitalWrite(DC_2.pin_dir, 0);
  analogWrite(DC_2.pin_speed, 0);
  digitalWrite(DC_3.pin_dir, 0);
  analogWrite(DC_3.pin_speed, 0);
  digitalWrite(DC_4.pin_dir, 0);
  analogWrite(DC_4.pin_speed, 0);
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
  // Correct sign of w_is with w
void include_sign(double * w, double * w_is, double * w_is_sign){
  for(int i=0; i++; i<=4){
    if(w[i]<0){
      w_is_sign[i] = -w_is[i];
    }
    else {
      w_is_sign[i] = w_is[i];
    }
  }
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
  mpu.setXGyroOffset(99);
  mpu.setYGyroOffset(-31); //-30
  mpu.setZGyroOffset(40); //41
  mpu.setZAccelOffset(1483); // 1484

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

void Print_Serial(){
  Serial.println("\t w \t\t  w_should \t w_is \t dir_is \t PID_Out_DC \t DC_Input");
      
      Serial.print("Wheel 1 \t");
      Serial.print(w[0]);
      Serial.print("\t\t");
      Serial.print(w_should[0]);
      Serial.print("\t");
      Serial.print(w_is[0]);
      Serial.print("\t");
      Serial.print(DC_1.dir);
      Serial.print("\t\t");
      Serial.print(PID_Out_DC[0]);
      Serial.print("\t\t");
      Serial.println(DC_1.mapped_speed);
      
      Serial.print("Wheel 2 \t");
      Serial.print(w[1]);
      Serial.print("\t\t");
      Serial.print(w_should[1]);
      Serial.print("\t");
      Serial.print(w_is[1]);
      Serial.print("\t");
      Serial.print(DC_2.dir);
      Serial.print("\t\t");
      Serial.print(PID_Out_DC[1]);
      Serial.print("\t\t");
      Serial.println(DC_2.mapped_speed);
      
      Serial.print("Wheel 3 \t");
      Serial.print(w[2]);
      Serial.print("\t\t");
      Serial.print(w_should[2]);
      Serial.print("\t");
      Serial.print(w_is[2]);
      Serial.print("\t");
      Serial.print(DC_3.dir);
      Serial.print("\t\t");
      Serial.print(PID_Out_DC[2]);
      Serial.print("\t\t");
      Serial.println(DC_3.mapped_speed);
    
      Serial.print("Wheel 4 \t");
      Serial.print(w[3]);
      Serial.print("\t\t");
      Serial.print(w_should[3]);
      Serial.print("\t");
      Serial.print(w_is[3]);
      Serial.print("\t");
      Serial.print(DC_4.dir);
      Serial.print("\t\t");
      Serial.print(PID_Out_DC[3]);
      Serial.print("\t\t");
      Serial.println(DC_4.mapped_speed);
      

      Serial.print("\n");
      Serial.print("\n");

      Serial.println(Robot_Pose.odometryPose[0]);
      Serial.println(Robot_Pose.odometryPose[1]);
      Serial.println(Robot_Pose.odometryPose[2]);
      Serial.print("\n");
      Serial.print("\n");
}
//////////////////////////////////////////////////////////////////////////////////////

// Folding Functions for Wings
void openWings(){
  digitalWrite(Wing_R_dir, HIGH);
  digitalWrite(Wing_L_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void openWing_R(){
  digitalWrite(Wing_R_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void openWing_L(){
  digitalWrite(Wing_L_dir, HIGH);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWings(){
  digitalWrite(Wing_R_dir, LOW);
  digitalWrite(Wing_L_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWing_R(){
  digitalWrite(Wing_R_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_R_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_R_step, LOW);
    delay(stepDelay_Wings);
  } 
}
void closeWing_L(){
  digitalWrite(Wing_L_dir, LOW);
  for(int i=0; i<steps_Wings ; i++){
    digitalWrite(Wing_L_step, HIGH);
    delayMicroseconds(stepTime_Wings);
    digitalWrite(Wing_L_step, LOW);
    delay(stepDelay_Wings);
  } 
}

// Folding Functions for Seating plates
void foldSeats_up(){
  digitalWrite(Seat_R_dir, HIGH);
  digitalWrite(Seat_M_dir, HIGH);
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_RL_up(){
  digitalWrite(Seat_R_dir, HIGH);
  digitalWrite(Seat_L_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_M_up(){
  digitalWrite(Seat_M_dir, HIGH);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_down(){
  digitalWrite(Seat_R_dir, LOW);
  digitalWrite(Seat_M_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_M_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_M_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_RL_down(){
  digitalWrite(Seat_R_dir, LOW);
  digitalWrite(Seat_L_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_R_step, HIGH);
    digitalWrite(Seat_L_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_R_step, LOW);
    digitalWrite(Seat_L_step, LOW);
    delay(stepDelay_Seats);
  } 
}
void foldSeats_M_down(){
  digitalWrite(Seat_M_dir, LOW);
  for(int i=0; i<steps_Seats ; i++){
    digitalWrite(Seat_M_step, HIGH);
    delayMicroseconds(stepTime_Seats);
    digitalWrite(Seat_M_step, LOW);
    delay(stepDelay_Seats);
  } 
}

