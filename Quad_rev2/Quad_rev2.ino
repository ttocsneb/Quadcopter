#include<Wire.h>


int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;


float pitch, roll, yaw;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  initSensors();
  
  initReceiver();

}



double startTime, endTime;

void loop() {
  startTime = micros();
  accel_signalen();
  endTime = micros();
  
  
  
  Serial.print("Pitch: ");Serial.print(pitch);Serial.print(", Roll: ");Serial.print(roll);Serial.print("; Time: ");Serial.println(endTime-startTime);

  delay(250);
  
}
