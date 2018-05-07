
// Pins for RFID Reader
const int SDA_PIN = 53;
const int RST_PIN = 5;
// SCK to 52; MOSI to 51; MISO to 50; 

// Bluetooth connection pins 5V TXD to 10 and RXD to 11
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
