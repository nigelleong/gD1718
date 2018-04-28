#include <SPI.h>
#include <require_cpp11.h>
#include <MFRC522.h>
#include <deprecated.h>
#include <MFRC522Extended.h>


#define RST_PIN 9
#define SS_PIN  8
MFRC522 mfrc522(SS_PIN, RST_PIN);

String str = "";
float x;
float y;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial.println("Setting up MFRC522...");
    
    SPI.begin();
    mfrc522.PCD_Init();

    Serial.println();
    Serial.println("Finish setup");
}

void loop() {
    // put your main code here, to run repeatedly:
//    str = scan();
    str = "x:240.1|y:111.8|";

    x = str.substring(2,6).toDouble();
    y = str.substring(10,14).toDouble();

    Serial.println(x);
    Serial.println(y);
}

String  scan  ()
{
    byte status;
    byte buffer[18];
    int err = 0;
    byte size = sizeof(buffer);

    if (mfrc522.PICC_IsNewCardPresent()) {
        if (mfrc522.PICC_ReadCardSerial()) {
            const String ID = dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
            Serial.println("New tag read: " + ID);
            mfrc522.PICC_HaltA();
            return ID;
        }
    }
}


String dump_byte_array(byte *buffer, byte bufferSize) {
    String out = "";
    for (byte i = 0; i < bufferSize; i++) {
        out += String(buffer[i] < 0x10 ? " 0" : " ") + String(buffer[i], HEX);
    }
    out.toUpperCase();
    out.replace(" ", "");
     
    return out;
}
