
#define LIN_OUT 1 // use the log output function
#define FFT_N 256 // set to 256 point fft
#include <FFT.h>

const char READ_SOUND = 's';
const char FIND_FLAME = 'f';

void setup() {
  // Set up serial port and leds
  Serial.begin(57600);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  
  redOn();

  // testing led
  while(false)
  {
    redOn();
    delay(1000);
    redOff();
    blueOn();
    delay(1000);
    blueOff();  
  }
  setupFan();
}

void setupFan()
{
   pinMode(7, OUTPUT);
   pinMode(6, OUTPUT);
}

void fanOn()
{
  digitalWrite(7,HIGH);
  digitalWrite(6, HIGH);
}

void fanOff()
{
  digitalWrite(7,LOW);
  digitalWrite(6, LOW);
}

void loop() 
{
//    redOn();
//    delay(500);
//    redOff();
//    delay(500);
  if(Serial.available()>0)
  {
    char incomingByte = char(Serial.read());
    if(incomingByte == READ_SOUND) 
    {
      redOff();
      bool hear = sound();
      
      if(hear)
      {
        Serial.println("Y");
        Serial.flush();
        blueOn();
      }
      else
      {
        Serial.println("N");
        Serial.flush();
        
      }


    }
    if(incomingByte == FIND_FLAME)
    {
    Serial.print(analogRead(1));
    Serial.print(" ");
    Serial.print(analogRead(2));
    Serial.print(" ");
    Serial.print(analogRead(3));
    Serial.print(" ");
    Serial.print(analogRead(4));
    Serial.print(" ");
    Serial.print(analogRead(5));
    Serial.print(" ");
    Serial.println();
    Serial.flush();
    }
  }
}

void blueOff()
{
  digitalWrite(2,LOW);
}

void blueOn()
{
  digitalWrite(2,HIGH);
}

void redOff()
{
  digitalWrite(3,LOW);
}

void redOn()
{
  digitalWrite(3,HIGH);
}
bool sound() {
  read_sound();
  int low_frequency_counter = 0; 
  for(int i = 0; i<10; i++)
  {
    if (low_frequency_sum() > 140)
      low_frequency_counter++;
      
    read_sound();
  }
  
  return (low_frequency_counter==10);
}

void read_sound() {
  for(int i=0; i < 512; i += 2) 
  {

    fft_input[i] = analogRead(A0)-512;
    //Serial.println(fft_input[i]);
    fft_input[i+1] = 0;
  }
  // window data, then reorder, then run, then take output
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_lin(); // take the output of the fft
}

int low_frequency_sum() {
  int sum = 0;

  for(int i = 90; i<125; i++)
    sum+=fft_lin_out[i];
    
  return (sum);
}

void display_sound() {
  for(int i=0; i < 512; i += 2) {
    Serial.println(fft_input[i]);
  }
}
