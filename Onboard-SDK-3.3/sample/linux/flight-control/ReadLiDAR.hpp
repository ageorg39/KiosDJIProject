
using namespace std;

#include <string>
#include <cstring>

#ifdef __cplusplus__
  #include <cstdlib>
#else
  #include <stdlib.h>
#endif

#include <iostream>

#include <fstream>
ifstream ArduinoFileIN;

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <stdlib.h> 



void setupUSBfile(int USB);

float readAndSendArduinoDistance(int USB);

