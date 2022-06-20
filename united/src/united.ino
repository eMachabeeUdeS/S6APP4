/*
 * Project united
 * Description: Transceiver de code Manchester
 * Author: Étienne Machabée (etienne@machab.ee) et Coralie Grégoire
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

uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}

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
