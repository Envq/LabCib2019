#include "SerialAck.h"

SerialAck::SerialAck(PinName tx, PinName rx, int baud) : Serial(tx,rx,baud) {
    debugSerial = new Serial(USBTX, USBRX, 921600);
}

int SerialAck::leggi(int timeout) {
  // if infinite timeout or we have a character, call base getc()
  if ( timeout == -1 || readable() )
    return getc();
    
  // no character yet  
  bool has_data = false;
  // count elapsed time
  Timer timer;
  timer.start();
  // loop until we have data or timeout elapses
  while ( !has_data && timer.read_ms() < timeout )
  {
    // wait a short time
    wait_ms(1);
    // check again
    has_data = readable();
  }
  // do we have anything?
  if ( has_data )
    // yes, read it
    return getc();
  else
    // no, timed out
    return -1;
}

/**
 *  Flush the serial
 */
void SerialAck::flushSerial() {
    while(this->readable()) {
        this->getc();
    }
}
        
/**
 *  Write to the serial with the identifier before the string and wait the ack
 */
void SerialAck::writeWithIdentifier(char *str, char identifier) {
    char car;
    bool ack = false;

    // write to the serial
    this->printf("%c,%s\n",identifier,str);
    do {    
        car = this->getc();//leggi(1);
        if(car==identifier) {
            ack = true;
        }
    } while(ack==false);
    //flushSerial();
}

/**
 *  Read from the serial waiting the right string and sending back the ack
 */
void SerialAck::readWithIdentifier(char *str, char identifier) {
    char buf[1024];
    char car;
    bool ack = false;

    do {
        // read from the serial
        this->gets(buf,sizeof(buf));
        
        // check if it's the right command
        if(buf[0]==identifier) {
            ack = true;
            this->printf("%c\n",identifier);
        }
    } while(ack==false);
    // flushSerial();

    strcpy(str,buf);
}

void SerialAck::sendError(bool error) {
    char buf[1024];
    bool ack = false;
    
    if(error) {
        sprintf(buf,"1");
    } else {
        sprintf(buf,"0");
    }

    writeWithIdentifier(buf,'e');
}

bool SerialAck::checkForError() {
    char buf[1024];
    char car, separator[2] = ",";

    readWithIdentifier(buf,'e');
    char *token = strtok(buf,separator);
    
    // take data
    token = strtok(NULL,separator);
    return atoi(token)==1 ? true : false;
}
