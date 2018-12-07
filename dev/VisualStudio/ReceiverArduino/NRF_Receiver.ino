#include <RF24_config.h>
#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);
const byte address[5] = { 0x00, 0x00, 0x00, 0x00, 0x01 };

void setup() 
{
    Serial.begin(115200);
    
    if(!radio.begin())
    {
        Serial.println("Failed radio startup");
        while(1);
    }

    Serial.println("Radio connected");

    radio.setChannel(0);
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();
}

void loop()
{
    if(radio.available())
    {
        char text[32] = "";
        radio.read(&text, sizeof(text));
        Serial.println(text);
    }
    else
    {
        Serial.println("No data rx yet");
    }

    delay(1000);
}
