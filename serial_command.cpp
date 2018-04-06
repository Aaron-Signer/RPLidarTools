#include <string>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include "serial_command.hpp"
#include "rs232.h"

using namespace std;
const int bdrate = 57600;
const int BUFFER_SIZE = 128;

const char READ_SOUND = 's';
const char FIND_FLAME = 'f';

void sendArduinoCommand(int serialPort, char command) {
  //RS232_cputs(serialPort, command.c_str());
  RS232_SendByte(serialPort, command);
  RS232_flushTX(serialPort);
}

string readArduinoResponse(int serialPort) {
  unsigned char buffer[BUFFER_SIZE];
  stringstream ss;
  bool done = false;;
  while(!done) {
    int n = RS232_PollComport(serialPort, buffer, BUFFER_SIZE);
    if (n > 0) {
      buffer[n] = '\0';
      ss << buffer;
      done = buffer[n-1] == '\n';
    }
  }
  return ss.str();
}


int setupArduinoSerial()
{
  char name[] = "ttyACM1"; // may need to change
  
  int serialP = RS232_GetPortnr(name);
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  if(RS232_OpenComport(serialP, bdrate, mode))
  {
    cout << "Can not open arduino serial port: " << name << endl;
    return(0);
  }
  return serialP;

}
