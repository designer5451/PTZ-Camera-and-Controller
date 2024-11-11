#ifndef LANC_CONTROL_H
#define LANC_CONTROL_H

int PinLANC = 34;  // 5V limited input signal from LANC data
int PinCMD = 2;   // Command to send to LANC
byte typeCam = B00101000;
byte typeVTR = B00011000;
int bitDuration = 100;  // Writing to the digital port takes about 8 microseconds so only 96 microseconds are left

// Function prototypes
void ZoomOut(int Wide);
void ZoomIn(int In);
void Focus(int direction);
void sendCMD(unsigned char cmd1, unsigned char cmd2);

// LANC initialization
void setupLANC() {
    pinMode(PinLANC, INPUT);
    pinMode(PinCMD, OUTPUT);
}

// Zoom out function
void ZoomOut(int Wide) {
    switch (Wide) {
        case 0: sendCMD(typeCam, B00010000); break;
        case 1: sendCMD(typeCam, B00010010); break;
        case 2: sendCMD(typeCam, B00010100); break;
        case 3: sendCMD(typeCam, B00010110); break;
        case 4: sendCMD(typeCam, B00011000); break;
        case 5: sendCMD(typeCam, B00011010); break;
        case 6: sendCMD(typeCam, B00011100); break;
        case 7: sendCMD(typeCam, B00011110); break;
    }
}

// Zoom in function
void ZoomIn(int In) {
    switch (In) {
        case 0: sendCMD(typeCam, B00000000); break;
        case 1: sendCMD(typeCam, B00000010); break;
        case 2: sendCMD(typeCam, B00000100); break;
        case 3: sendCMD(typeCam, B00000110); break;
        case 4: sendCMD(typeCam, B00001000); break;
        case 5: sendCMD(typeCam, B00001010); break;
        case 6: sendCMD(typeCam, B00001100); break;
        case 7: sendCMD(typeCam, B00001110); break;
    }
}

// Focus function
void Focus(int direction) {
    switch (direction) {
        case 0: sendCMD(typeCam, 0x45); break;  // Focus Far
        case 1: sendCMD(typeCam, 0x47); break;  // Focus Near
    }
}

// Send LANC command
void sendCMD(unsigned char cmd1, unsigned char cmd2) {
    for (int cmdRepeatCount = 0; cmdRepeatCount < 5; cmdRepeatCount++) {  // Repeat 5 times to ensure the camera accepts the command
        while (pulseIn(PinLANC, HIGH) < 5000) {
            // Wait for a new data packet start
        }
        delayMicroseconds(bitDuration);  // Wait START bit duration

        for (int i = 0; i < 8; i++) {
            digitalWrite(PinCMD, (cmd1 & (1 << i)) ? HIGH : LOW);
            delayMicroseconds(bitDuration);
        }

        digitalWrite(PinCMD, LOW);  // Set LANC line back to +5V
        delayMicroseconds(10);  // Ensure we are in the stop bit before byte 1

        while (digitalRead(PinLANC)) {
            // Wait for the stop bit to finish
        }
        
        delayMicroseconds(bitDuration);  // Wait START bit duration for Byte 1

        for (int i = 0; i < 8; i++) {
            digitalWrite(PinCMD, (cmd2 & (1 << i)) ? HIGH : LOW);
            delayMicroseconds(bitDuration);
        }

        digitalWrite(PinCMD, LOW);  // Set LANC line back to +5V
    }
}

#endif
