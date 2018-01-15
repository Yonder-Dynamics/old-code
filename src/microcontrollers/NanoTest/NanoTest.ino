#include "ArduinoUtil.h"

void setup() {
  	// put your setup code here, to run once:
	pinMode(LED_BUILTIN, OUTPUT);
	arduinoHandshake(HANDSHAKE_WAIT);

}

void loop() {
  	// put your main code here, to run repeatedly:
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(500);                       // wait for a second
	digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
	delay(500);                       // wait for a second
}
