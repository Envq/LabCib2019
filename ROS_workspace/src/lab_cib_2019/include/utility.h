#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <fstream>

int write_to_csv(const char *filename, char *str, bool truncate=false)
{
    std::fstream file;

    if(truncate==false) {
	    // open the file in write mode
	    file.open(filename,std::ios::out);
    } else {
	    // open the file in write mode and truncate
	    file.open(filename,std::ios::out|std::ios::trunc);
    }
    // error if the file doesn't open
    if(file.is_open() == false)
    {
        std::perror("ERROR: can't open the file: ");
        std::perror(filename);
        return -1;
    }
    
    // write the string
    file << str << std::endl;

    // close the file
    file.close();
    
    return 1;
}

#endif  // UTILITY_H
