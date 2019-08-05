// #include <iostream>
#include <mbed.h>

#ifndef SERIAL_ACK_H
#define SERIAL_ACK_H

using namespace std;

class SerialAck: public Serial {
    public:
        SerialAck(PinName tx, PinName rx, int baud);
        void flushSerial();
        void writeWithIdentifier(char *str, char identifier);
        void readWithIdentifier(char *str, char identifier);
        void sendError(bool error);
        bool checkForError();
        
    private:
        int leggi(int timeout);
        Serial *debugSerial;
};

#endif
