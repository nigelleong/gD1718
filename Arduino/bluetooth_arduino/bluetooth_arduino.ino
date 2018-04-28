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
    int   arg = 0;

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
        arg *= 10;
        arg = arg + (command_buffer[i] - 48);
        i++;
    }
  
    /*----------------------------------------------
     *  First character in array is the command 
     *  
     *  Command Legends:
     *  ----------------
     *  W ---> Move Forward
     *  A ---> Move Left
     *  D ---> Move Right
     *  S ---> Move Back
     *  Z ---> Read Sensor Value
     *  C ---> Calibrate Bot
     *  R ---> Resend reply string
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
          blinkLED(arg);
          
          break;
      
      case 'W':                     /* turn off led */
          //call function : go straight one grid
          moveForward(arg);
          break;
          
      case 'D':                     /* turn off led */
          //call function : go right one grid
          break;
          
      case 'A':                     /* turn off led */
          //call function : go left one grid
          break;
          
      case 'S':                     /* turn off led */
          //call function : go back one grid
          break;
          
      default :                     /* help */ 
          BT.println("Send '1' to turn LED on");
          BT.println("Send '0' to turn LED off");
          BT.println("Send 'B5' to blink LED 5 times");
          BT.println("Send 'W1' to go straight 1 grid");
          BT.println("Send 'S1' to go back 1 grid");
          BT.println("Send 'A1' to go left 1 grid");
          BT.println("Send 'D1' to go right 1 grid");
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

void moveForward  (int numGrid){
    
}


