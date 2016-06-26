#include<Wire.h>

#define BUZZER 17

int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;


float pitch, roll, yaw;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(BUZZER, OUTPUT);

  shortBeep();
  
  initSensors();

  blip();
  delay(50);
  blip();
  
  //initReceiver();

}



double Time, tmp;

int t;

void loop() {
  t++;
  if(t == 125) {
    t =0;
    Serial.print(pitch);Serial.print(";");Serial.println(roll);
  }

  signalen();
    
  
  
  long b = micros();
  while(micros()-b < 4000);
}
