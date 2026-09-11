// #include "CommsSerial.h"
#include <Arduino.h>

void setup() 
{
    Serial.begin(9600);

    Serial.println("init"); // had to add this; for some reason without it we don't pull in Print.cpp which causes a linker error because _write is not defined
}

void loop() {}