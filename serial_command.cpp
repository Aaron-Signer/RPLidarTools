#include <string>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include "serial_command.hpp"
#include "rs232.h"

using namespace std;
const int bdrate = 9600;
const int BUFFER_SIZE = 128;

const char READ_SOUND = 's';
const char FIND_FLAME = 'f';

void sendArduinoCommand(int serialPort, char command) {
  cout << "sending command" << command << endl;
  RS232_SendByte(serialPort, command);
  RS232_flushTX(serialPort);
}

string readArduinoResponse(int serialPort) {
  unsigned char buffer[BUFFER_SIZE];
  stringstream ss;
  bool done = false;;
  while(!done && ss.str().substr(0,5) != "ping") {
    int n = RS232_PollComport(serialPort, buffer, BUFFER_SIZE);
    if (n > 0) {
      buffer[n] = '\0';
      cout << "--------partial from arduino: " << buffer << endl;
      ss << buffer;
      done = buffer[n-1] == '\n';
    }
  }
  cout << "from arduino: " << ss.str() << endl;
  return ss.str();
}


int setupArduinoSerial()
{
  char name[] = "ttyACM0"; // may need to change
  
  int serialP = RS232_GetPortnr(name);
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  if(RS232_OpenComport(serialP, bdrate, mode))
  {
    cout << "Can not open arduino serial port: " << name << endl;
    return(0);
  }
  return serialP;

}
