/*
 * Project united
 * Description: Transceiver de code Manchester
 * Author: Étienne Machabée (mace2801) et Coralie Grégoire (grec3306)
 * Date: Juin 2022
 */

#include "Particle.h"

SYSTEM_THREAD(ENABLED);

#define CRC16 0x8005
#define INPUTPIN D12
#define OUTPUTPIN D0

void threadFunction(void *param);

Thread thread1("encoder", encoder);
Thread thread2("decoder", decoder);

// Parties hardcodées du code Manchester
const uint8_t preambule = 0b01010101;
const uint8_t start = 0b01111110;
const uint8_t type_flags = 0b11111111;
uint8_t payloadlength = 0;
uint8_t payload[73] = {0};
uint8_t crc16_1 = 0;
uint8_t crc16_2 = 0;

const uint8_t end = 0b01111110;

// Le CRC16 est 16 bits, donc 2 octets
// Charge utile maximale de 73 octets

const int frameSize = 3;
uint8_t frametosend[frameSize] = {0};  // Une trame a une longueur de 80 octets de long

// Variables pour l'ISR de réception
const int tRef = 5;
bool lastBit = false;           // On commence toujours en assumant que avant c'était un 0
system_tick_t timer1 = 0;
uint8_t bitPosition = 0;        // Position du bit actuel dans un octet
uint8_t byteToStore = 0b00000000;        // Octet à stocker

// Variables pour la réception
uint8_t receivedFrame[80] = {0};    // Trame reçue
uint8_t frameCounter = 0;           // Position dans la trame

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
    //frame[1] = start;
    //frame[2] = type_flags;
}

void setup() {
	Serial.begin(9600);
	pinMode(OUTPUTPIN, OUTPUT);
    digitalWrite(OUTPUTPIN, HIGH);
	pinMode(INPUTPIN, INPUT);
    frametosend[0] = preambule;
    frametosend[1] = start;
    frametosend[2] = type_flags;
	attachInterrupt(INPUTPIN, isr, CHANGE, 0);
}

void loop() {
	if (millis() - lastReport >= 1000) {
		lastReport = millis();

		//Serial.printlnf("counter=%d", counter);
		//Serial.printlnf("Pulse length: %i", lastBit);
        
        //for(int loop = 0; loop < 80; loop++)
        //    Serial.printlnf("byte[%d]: %d ", loop, receivedFrame[loop]);
	}
}


void encoder(void *param) {
    os_thread_delay_until(&lastThreadTime, 1000);
    digitalWrite(OUTPUTPIN, HIGH);
	while(true) {
        for (int i = 0; i < frameSize; i++){
            uint8_t comp = 0b10000000;
            Serial.printlnf("START");
            for (uint8_t j = 0; j < 8; j++){
                uint8_t cc = frametosend[i] & comp;
                Serial.printlnf("CC: %i", cc);
                if (cc){
                    Serial.printlnf("HIGH");
                    digitalWrite(OUTPUTPIN, HIGH);
                    os_thread_delay_until(&lastThreadTime, tRef);
                    digitalWrite(OUTPUTPIN, LOW);
                    os_thread_delay_until(&lastThreadTime, tRef);
                }
                else{
                    Serial.printlnf("LOW");
                    digitalWrite(OUTPUTPIN, LOW);
                    os_thread_delay_until(&lastThreadTime, tRef);
                    digitalWrite(OUTPUTPIN, HIGH);
                    os_thread_delay_until(&lastThreadTime, tRef);
                }
                comp = comp>>1; // Bitshift à gauche de 1
            }
        }
        digitalWrite(OUTPUTPIN, HIGH);
        os_thread_delay_until(&lastThreadTime, 1000);

		// //counter++;
        // os_thread_delay_until(&lastThreadTime, 1000);
        // for (int i = 0; i < 40*8; i++){
        // digitalWrite(D0, HIGH);
        
		// // Delay so we're called every 10 milliseconds (100 times per second)
		// os_thread_delay_until(&lastThreadTime, tRef);
		// digitalWrite(D0, LOW);
		// os_thread_delay_until(&lastThreadTime, 2*tRef);
        // }
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
    system_tick_t time = millis() - timer1;
    timer1 = millis();
    //if (time <= tRef+1){
        // Ne rien faire et ne pas inverser l'ancien bit, non compilé
    //}
    // else if(time >= tRef-1 && time <= 3*tRef){
    if(time >= tRef-1 && time <= 3*tRef){
        // Inverser le lastBit
        lastBit = !lastBit;
}
            // Sauvegarder le bit courant
        if(bitPosition == 0){
            if(lastBit) byteToStore = byteToStore | 0b10000000;
            bitPosition++;
        }
        else if(bitPosition == 1){
            if(lastBit) byteToStore = byteToStore | 0b01000000;
            bitPosition++;
        }
        else if(bitPosition == 2){
            if(lastBit) byteToStore = byteToStore | 0b00100000;
            bitPosition++;
        }
        else if(bitPosition == 3){
            if(lastBit) byteToStore = byteToStore | 0b00010000;
            bitPosition++;
        }
        else if(bitPosition == 4){
            if(lastBit) byteToStore = byteToStore | 0b00001000;
            bitPosition++;
        }
        else if(bitPosition == 5){
            if(lastBit) byteToStore = byteToStore | 0b00000100;
            bitPosition++;
        }
        else if(bitPosition == 6){
            if(lastBit) byteToStore = byteToStore | 0b00000010;
            bitPosition++;
        }
        else if(bitPosition == 7){
            if(lastBit) byteToStore = byteToStore | 0b00000001;
            receivedFrame[frameCounter] = byteToStore;
            frameCounter++;
            // Serial.printlnf("Byte stored %d", byteToStore);
            byteToStore = 0b00000000;
            bitPosition = 0;
        }
   
}
