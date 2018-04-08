#pragma once
#include <string>

// call before anything else, returns serialPort
int setupArduinoSerial(char*);

// serialPort, commandLetter
void sendArduinoCommand(int, char);

// serialPort -- reads upto and including a newline character
std::string readArduinoResponse(int);

