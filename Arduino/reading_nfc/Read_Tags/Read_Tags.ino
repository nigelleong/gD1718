

#include <SPI.h>
#include <MFRC522.h>


#define SS_PIN          53
#define RST_PIN         5
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::StatusCode status; //variable to get card status

byte read_buffer[18];
byte read_size = sizeof(read_buffer);

/* Ultraligth mem = 16 pages
*  4 bytes per page.
*/
uint8_t pageAddr = 0x04;  //write/read 16 bytes (page 4,5,6 and 7).
    
void setup() 
{
    Serial.begin(9600); 
    Serial.println(F("Setting up MFRC522..."));
    
    SPI.begin();
    mfrc522.PCD_Init(); // Init MFRC522 card  

    Serial.println(F("Finish setup"));

}

void loop() 
{   
    int tag_x =0;
    int tag_y =0;
    
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent())
      return;
  
    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
      return;

  
    /*------------------------------ Read data ------------------------------*/
    //Reading data are always 4 block at once.
    Serial.println(F("Reading data ... "));
    read_size = sizeof(read_buffer);        //reinitiate buffer size which is modified after error reporting
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, read_buffer, &read_size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("Reading failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        Serial.println();
        return;
    }
    
    Serial.println(F("Received data from MIFARE Ultralight:"));
    for (byte i = 0; i < 16; i++) {
        Serial.write(read_buffer[i]);
    }
    Serial.print("\n");
    
    /*-----------------------------------------------------------------------*/
    mfrc522.PICC_HaltA();

    read_tag(&tag_x, &tag_y);
    Serial.print("tag_x = ");
    Serial.println(tag_x);
    Serial.print("tag_y = ");
    Serial.println(tag_y);
    Serial.println();
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



