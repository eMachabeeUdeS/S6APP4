/*
 * Project united
 * Description: Transceiver de code Manchester
 * Author: Étienne Machabée (mace2801) et Coralie Grégoire (grec3306)
 * Date: Juin 2022
 */

#include "Particle.h"

SYSTEM_THREAD(ENABLED);

#define CRC16           0x8005
#define INPUTPIN        D12
#define OUTPUTPIN       D0
#define MAX_FRAME_SIZE  80

void threadFunction(void *param);

Thread thread1("encoder", encoder);
Thread thread2("decoder", decoder);

// Parties hardcodées du code Manchester
const uint8_t preambule = 0b01010101;
const uint8_t start = 0b01111110;
const uint8_t type_flags = 0b11111111;
uint8_t payloadlength = 0;
uint8_t payload[73] = {0};


const uint8_t end = 0b01111110;

// Le CRC16 est 16 bits, donc 2 octets
// Charge utile maximale de 73 octets

// Variables pour l'ISR de réception
const int tRef = 5;
bool lastBit = true;           // On commence toujours en assumant que avant c'était un 0
system_tick_t timer2 = 0;     // Temps avant, pour comparer et skipper des fronts montants inutiles
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

void addBit();

/**
 * @brief Construit une trame à envoyer
 * 
 * @param frame Pointeur vers la trame
 * @param size Taille de la charge utile, maximum 73
 * @return int Taille de la trame complète
 */
int framebuilder(uint8_t* frame, int size)
{
    frame[0] = preambule;
    frame[1] = start;
    frame[2] = type_flags;
    frame[3] = (uint8_t)size;
    for (int i = 3; i < size; i++){
        frame[i+4] = i;
    }

    uint8_t crc16_1 = 0;
    uint8_t crc16_2 = 0;
    uint16_t crc16 = gen_crc16(&frame[4],size);
    // uint16_t crc16 = 0b1011001010100110;

    crc16_1 = (uint8_t)(0b0000000011111111 & crc16);
    crc16_2 = (uint8_t)(0b1111111100000000 & crc16)<<8;

    // memcpy(&crc16_1,&crc16,1);
    // memcpy(&crc16_2,&crc16+1,1);

    Serial.printlnf("crc16 : %d",crc16);
    Serial.printlnf("crc8_1 : %d",crc16_1);
    Serial.printlnf("crc8_2 : %d",crc16_2);

    /***** METTRE LE RÉSULTAT DU CRC16 EN 2 BYTES ICI *****/
    frame[size + 4] = crc16_1;   // CORALIE CE SONT LES 2 BYTES POUR STOCKER LE CRC16
    frame[size + 5] = crc16_2;   // CORALIE CE SONT LES 2 BYTES POUR STOCKER LE CRC16
    /***** FIN DU RÉSULTAT DU CRC16 *****/
    frame[size + 6] = start;

    return size + 7;

}

void setup() {
	Serial.begin(9600);
	pinMode(OUTPUTPIN, OUTPUT);
    digitalWrite(OUTPUTPIN, HIGH);
	pinMode(INPUTPIN, INPUT);
    // frametosend[0] = preambule; //85
    // frametosend[1] = start;     //126
    // frametosend[2] = type_flags;//255
	attachInterrupt(INPUTPIN, isr, CHANGE, 0);
}

void loop() {
	if (millis() - lastReport >= 1000) {
		lastReport = millis();

		//Serial.printlnf("counter=%d", counter);
		//Serial.printlnf("Pulse length: %i", lastBit);
        
        //for(int loop = 0; loop < 3; loop++)
        //    Serial.printlnf("byte[%d]: %d ", loop, receivedFrame[loop]);
	}
}


void encoder(void *param) {
    os_thread_delay_until(&lastThreadTime, 1000);

    // Variables pour la transmission
    uint8_t frametosend[MAX_FRAME_SIZE] = {0};  // Une trame a une longueur de 80 octets de long
    int frameSize = framebuilder(frametosend, 73);
    digitalWrite(OUTPUTPIN, HIGH);
	while(true) {
        for (int i = 0; i < frameSize; i++){
            uint8_t comp = 0b10000000;
            // Serial.printlnf("START");
            for (uint8_t j = 0; j < 8; j++){
                uint8_t cc = frametosend[i] & comp;
                // Serial.printlnf("CC: %i", cc);
                if (cc){
                    // Serial.printlnf("HIGH");
                    // Serial.printf("1");
                    digitalWrite(OUTPUTPIN, HIGH);
                    os_thread_delay_until(&lastThreadTime, tRef);
                    digitalWrite(OUTPUTPIN, LOW);
                    os_thread_delay_until(&lastThreadTime, tRef);
                }
                else{
                    // Serial.printlnf("LOW");
                    // Serial.printf("0");
                    digitalWrite(OUTPUTPIN, LOW);
                    os_thread_delay_until(&lastThreadTime, tRef);
                    digitalWrite(OUTPUTPIN, HIGH);
                    os_thread_delay_until(&lastThreadTime, tRef);
                }
                comp = comp>>1; // Bitshift à gauche de 1
            }
            // Serial.printf("\n");
        }
        digitalWrite(OUTPUTPIN, HIGH);
        os_thread_delay_until(&lastThreadTime, 3000);

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

void addBit(){
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
        if(lastBit) byteToStore = byteToStore | 0b00000100 ;
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
        // Serial.printf("\n");
        // Serial.printlnf("Byte stored %d", byteToStore);
        byteToStore = 0b00000000;
        bitPosition = 0;
    }
}

void isr(){
    system_tick_t time = millis() - timer1;
    system_tick_t time2 = millis() - timer2;
//    Serial.printlnf("Time: %d, Time2: %d", time, time2);
    if (time <= 1.5*tRef && time2 <= 1.5*tRef) timer1 = millis();
    else if (time <= 1.5*tRef && time2 >= 1.5*tRef){
        // Serial.printf("%d", lastBit);
        addBit();
        timer1 = millis();
        timer2 = millis();
    }
    else if (time >= 1.5*tRef){
        lastBit = !lastBit;
        // Serial.printf("%d", lastBit);
        addBit();
        timer1 = millis();
        timer2 = millis();
    }
}

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
