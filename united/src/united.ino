/*
 * Project united
 * Description: Transceiver de code Manchester
 * Author: Étienne Machabée (mace2801) et Coralie Grégoire (grec3306)
 * Date: Juin 2022
 */

#include "Particle.h"

SYSTEM_THREAD(ENABLED);

#define CRC16 0x8005

void threadFunction(void *param);

Thread thread1("encoder", encoder);
Thread thread2("decoder", decoder);

// Parties hardcodées du code Manchester
const uint8_t preambule = 0b01010101;
const uint8_t start = 0b01111110;
const uint8_t type_flags = 0b11111111;
uint8_t payloadlength = 0;
uint8_t payload[73] = {0};
uint8_t crc16 = 0;

const uint8_t end = 0b01111110;

// Le CRC16 est 16 bits, donc 2 octets
// Charge utile maximale de 73 octets

uint8_t frametosend[80] = {0};  // Une trame a une longueur de 80 octets de long

volatile int counter = 0;
unsigned long lastReport = 0;
system_tick_t lastThreadTime = 0;
system_tick_t lastThreadTime2 = 0;
unsigned long pulseLength = 0;
bool measuring = false;
unsigned long lastPulseTime = 0;
unsigned long lastPulse = 0;

void framebuilder(uint8_t* frame)
{
    frame[0] = preambule;
    frame[1] = start;
    frame[2] = type_flags;
}

void setup() {
	Serial.begin(9600);
	pinMode(D0, OUTPUT);
	pinMode(D12, INPUT);
	attachInterrupt(D12, isr, CHANGE, 0);
}

void loop() {
	if (millis() - lastReport >= 1000) {
		lastReport = millis();

		//Serial.printlnf("counter=%d", counter);
		Serial.printlnf("Pulse length: %d", lastPulse);
	}
}


void encoder(void *param) {
	while(true) {
		//counter++;

        digitalWrite(D0, HIGH);
        
		// Delay so we're called every 10 milliseconds (100 times per second)
		os_thread_delay_until(&lastThreadTime, 5);
		digitalWrite(D0, LOW);
		os_thread_delay_until(&lastThreadTime, 5);
	}
	// You must not return from the thread function
}

void decoder(void *param) {
    while (true) {
        //pulseLength = pulseIn(D12, HIGH);
        int a = 0;
        a++;
        os_thread_yield();
    }
}

void isr(){
    if (!measuring){
        lastPulseTime = millis();
        measuring = true;
    }
    else
    {
        lastPulse = millis() - lastPulseTime;
        measuring = false;
    }
    
    
}
