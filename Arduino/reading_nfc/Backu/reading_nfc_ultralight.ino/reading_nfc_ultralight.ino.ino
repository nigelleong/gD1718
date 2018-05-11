/*
 *   RFID-RC522 (SPI connexion)
 *   
 *   CARD RC522      Arduino (UNO)
 *     SDA  -----------  10 (Configurable, see SS_PIN constant)
 *     SCK  -----------  13
 *     MOSI -----------  11
 *     MISO -----------  12
 *     IRQ  -----------  
 *     GND  -----------  GND
 *     RST  -----------  9 (onfigurable, see RST_PIN constant)
 *     3.3V ----------- 3.3V
 *     
 */

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN          10
#define RST_PIN         9
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
  
void setup() 
{
    Serial.begin(9600); 
    Serial.println(F("Setting up MFRC522..."));
    
    SPI.begin();
    mfrc522.PCD_Init(); // Init MFRC522 card  
    
    Serial.println(F("Finish setup"));
    
    memcpy(write_buffer,"x:240.1|y:111.8|",16);
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
    Serial.println();
    
    /*-----------------------------------------------------------------------*/
    
    mfrc522.PICC_HaltA();
}
