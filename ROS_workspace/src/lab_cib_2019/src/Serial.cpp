#include "Serial.h"

Serial::Serial(const char *comname, cc_t min_char, cc_t min_time) {
    struct termios tattr;   // scructure for serial's settings

	com = open(comname, O_RDWR | O_NOCTTY | O_SYNC );   // open the device in read/write sync

    if (com == -1)
    {
        perror("ERROR: can't open the serial");
        exit(1);
    }

    // for more info: libc->"Low-Level Terminal Interface"->"Noncanonical Mode Example"
    tcgetattr(com, &tattr);

    // input modes - turn off written flags
    tattr.c_iflag &= ~(INLCR|IGNCR|ICRNL|IXON);
    // output modes - turn off written flags
    tattr.c_oflag &= ~(OPOST|ONLCR|OCRNL|ONLRET); 
    // control modes - set 8 bit (CS8), enable receive (CREAD), ignore control signals (CLOCAL)
    tattr.c_cflag = CS8 | CREAD | CLOCAL; 
    // remove traduction of character (ICANON) and ECHO
    tattr.c_lflag &= ~(ICANON|ECHO);         
    // give data after 0 char received
    tattr.c_cc[VMIN] = min_char;                       
    // timeout of 1, after that the read return 0
    tattr.c_cc[VTIME] = min_time;

    // receive speed
    cfsetispeed (&tattr, B921600);
    // transmit speed
    cfsetospeed (&tattr, B921600);

    // set the new attributes
    tcsetattr (com, TCSAFLUSH, &tattr);
}
    
void Serial::writeWithIdentifier(char *str, char identifier) {
    char temp[256];
    char ret;
    sprintf(temp,"%c,%s\n",identifier,str);
    ::write(com,temp,strlen(temp));
    do{
        ret = readChar();
    }while(ret!=identifier);

    flushSerial();
}

void Serial::writeWithout(char *str) {
    char temp[256];
    sprintf(temp,"%s\n",str);
    ::write(com,temp,strlen(temp));

    flushSerial();
}

void Serial::readWithIdentifier(char identifier) {
    char c;
    do {
        c = readChar();
    } while(c!=identifier);
}

char Serial::readChar() {
    char car;
    ::read(com,&car,1);
    return car;
}

bool Serial::isReadable() {
    FD_ZERO(&set);       // empty the set
    FD_SET(com,&set);    // add serial to the set

    return FD_ISSET(com,&set);
}

void Serial::flushSerial() {
    char c;
    do {
        c=readChar();
    } while(!c);
}
