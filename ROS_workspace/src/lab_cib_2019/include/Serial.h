#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

using namespace std;

class Serial {
private:
    int com;    // serial file
    char buf[256];  // serial buffer
    fd_set set;     // descriptors set for 'select'

public:
    Serial(const char *comname, cc_t min_char, cc_t min_time);
    void writeWithIdentifier(char *str, char identifier);
    void readWithIdentifier(char identifier);
	void writeWithout(char *str);
    char readChar();
    bool isReadable();
    void flushSerial();
};
