

#include <SPI.h>
#include <MFRC522.h>


#define SS_PIN          53
#define RST_PIN         5
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
MFRC522::StatusCode status; //variable to get card status

byte write_buffer[18];  //data transfer buffer (16+2 bytes data+CRC)
byte read_buffer[18];
byte write_size = sizeof(write_buffer);
byte read_size = sizeof(read_buffer);

/* Ultraligth mem = 16 pages
*  4 bytes per page.
*/
uint8_t pageAddr = 0x04;  //write/read 16 bytes (page 4,5,6 and 7).

   int x_val = 0;
    int y_val = 0;
    String x_str = String(x_val);
    String cut = "|";
    String y_str = String(y_val);
    String str = x_str + cut + y_str + cut;
    //Serial.println(str);
    //memcpy(write_buffer,"2500|2500|",16);
    
void setup() 
{
    Serial.begin(9600); 
    Serial.println(F("Setting up MFRC522..."));
    
    SPI.begin();
    mfrc522.PCD_Init(); // Init MFRC522 card  
         memcpy(write_buffer,&str[0],16);

    Serial.println(F("Finish setup"));

}

void loop() 
{
    // Look for new cards
    if ( ! mfrc522.PICC_IsNewCardPresent())
      return;
  
    // Select one of the cards
    if ( ! mfrc522.PICC_ReadCardSerial())
      return;
  
    /*------------------------------ Write data ------------------------------*/
    Serial.println(F("Writing data ... "));
    for (int i=0; i < 4; i++) {
      //data is writen in blocks of 4 bytes (4 bytes per page)
      status = (MFRC522::StatusCode) mfrc522.MIFARE_Ultralight_Write(pageAddr+i, &write_buffer[i*4], 4);
      if (status != MFRC522::STATUS_OK) {
        Serial.print(F("Writing failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return;
      }
    }
    x_val += 250;
    x_str = String(x_val);
    cut = "|";
    y_str = String(y_val);
    str = x_str + cut + y_str + cut;
    //Serial.println(str);
     memcpy(write_buffer,&str[0],16);
    Serial.println(F("Written to MIFARE Ultralight"));
    Serial.println();
  
    /*------------------------------ Read data ------------------------------*/
    //Reading data are always 4 block at once.
    
    Serial.println(F("Reading data ... "));
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(pageAddr, read_buffer, &read_size);
    if (status != MFRC522::STATUS_OK) {
      Serial.print(F("Reading failed: "));
      Serial.println(mfrc522.GetStatusCodeName(status));
      return;
    }
  
    Serial.println(F("Received data from MIFARE Ultralight:"));
    for (byte i = 0; i < 16; i++) {
      Serial.write(read_buffer[i]);
    }
    Serial.println("\n");
    
    /*-----------------------------------------------------------------------*/
    mfrc522.PICC_HaltA();
}

