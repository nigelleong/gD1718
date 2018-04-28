#include <SoftwareSerial.h>
SoftwareSerial BT(10, 11); 
// creates a "virtual" serial port/UART
// connect BT module TX to D10
// connect BT module RX to D11
// connect BT Vcc to 5V, GND to GND
void setup()  
{
    // set digital pin to control as an output
    pinMode(LED_BUILTIN, OUTPUT);
    BT.begin(9600);
    
    // Test message to device
    BT.println("Hello from Arduino");
}

void loop() 
{
    /* Declare global variables */
    char  input; // stores incoming character from other device
    char  command_buffer[10];
    int   i = 0;
    int   arg1 = 0;
    int   arg2 = 0;
    int   arg3 = 0;

    while (1){
        if ( BT.available() ) {
            input = ( BT.read() );
            command_buffer[i] = input;
            i++;
            if (input == '|'){       /* setting pointer back to argument */
                i = 1;
                break;
            }
        }
        BT.println("not available");
    }

    //Converts subsequent characters in the array into integer as function argument
    while (command_buffer[i] != '|') {
        arg1 *= 10;
        arg1 = arg1 + (command_buffer[i] - 48);
        i++;
    }

    i++;
    while (command_buffer[i] != '|') {
        arg2 *= 10;
        arg2 = arg2 + (command_buffer[i] - 48);
        i++;
    }

    i++;
    while (command_buffer[i] != '|') {
        arg3 *= 10;
        arg3 = arg3 + (command_buffer[i] - 48);
        i++;
    }
  
    /*----------------------------------------------
     *  First character in array is the command 
     *  
     *  Command Legends:
     *  ----------------
     *  Z ---> Read Sensor Value
     *  C ---> Calibrate Bot
     *  R ---> Resend reply string
     *  
     *  M ---> Robot moves
     *  S---> Robot fold seat
     *  W---> Robot fold wings
     *  
     *  1 ---> On LED
     *  0 ---> Off LED
     *  B ---> Blink LED
    ----------------------------------------------*/

    switch (command_buf[0]) {
      case '1':                     /* turn on led */
          BT.println("LED on");
          digitalWrite(LED_BUILTIN, HIGH);
          break;
          
      case '0':                     /* turn off led */
          BT.println("LED off");
          digitalWrite(LED_BUILTIN, LOW);
          break;

      case 'B':                     /* blink led */
          BT.println("Blink LED");
          blinkLED(arg1);
          break;

      case 'M':                     /* move robot */
          //call function : move robot with |Xvelocity|Yvelocity|
          robotMove(arg1, arg2, arg3);
          break;

      case 'W':                     /* move robot */
          //call function : move robot with |Xvelocity|Yvelocity|
          robotFoldWings(arg1, arg2, arg3);
          break;

      case 'S':                     /* move robot */
          //call function : move robot with |Xvelocity|Yvelocity|
          robotFoldSeat(arg1, arg2, arg3);
          break;
          
      default :                     /* help */ 
          BT.println("Send '1||||' to turn LED on");
          BT.println("Send '0||||' to turn LED off");
          BT.println("Send 'B|5|||' to blink LED 5 times");
          BT.println("Send 'W||||' to fold wings");
          BT.println("Send 'S||||' to fold seat");
          BT.println("Send 'M|2000|1000|0|' to move 2000mm/sec in x, 1000mm/sec, 0 change in degree");
    }
}

void blinkLED (int times){
    while (times){
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(1000);
        times--;  
    }
}

void  robotMove ( int arg1,
                  int arg2,
                  int arg3)
{
    
}

void  robotFoldSeat ( int arg1,
                      int arg2,
                      int arg3)
{
    
}

void  robotFoldWings  ( int arg1,
                        int arg2,
                        int arg3)
{
    
}

