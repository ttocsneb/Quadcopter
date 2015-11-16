#include<Wire.h>
#include<Adafruit_NeoPixel.h>

/**
 * PID Variables
 */

float pid_p_gain_roll = 0.4;                 //Gain setting for the roll P-controller (0.7)0.3
float pid_i_gain_roll = 0.02;              //Gain setting for the roll I-controller (0.015)
float pid_d_gain_roll = 10.5;              //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-) 

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

int throttle;

/**
 * Receiver Variables
 */
 
unsigned long current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4, receiver_input_channel_5, receiver_input_channel_6;

/**
 * Gyro Variables
 */

#define gyro_addr 0x6B

int cal_int;
unsigned long UL_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte lowByte, highByte;

/**
 * ESC Variable
 */
 
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
unsigned long zero_timer, timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
int esc_1, esc_2, esc_3, esc_4;

/**
 * Misc Variables
 */
 
int battery_voltage;
unsigned long loop_timer;
int loops;
int last;
int brightness;

unsigned long time, timeLast;

Adafruit_NeoPixel light = Adafruit_NeoPixel(5, 3, NEO_GRB + NEO_KHZ800);
#define RED light.Color(255, 0, 0)
#define GREEN light.Color(0, 255, 0)
#define BLUE light.Color(0, 0, 255)
#define YELLOW light.Color(255, 255, 0)
#define PURPLE light.Color(255, 0, 255)
#define ORANGE light.Color(255, 128, 0)
#define WHITE light.Color(255, 255, 255)
#define OFF light.Color(0, 0, 0)

/**
 * Called at program startup.
 */
void setup() {
  DDRD |= B11110000;
  
  Wire.begin();
  Serial.begin(9600);
  light.begin();
  
  //Initialize gyro
  Wire.beginTransmission(gyro_addr);
  Wire.write(0x20);
  Wire.write(0x0F);
  Wire.endTransmission();
  
  Wire.beginTransmission(gyro_addr);
  Wire.write(0x23);
  Wire.write(0x80);
  Wire.endTransmission();
  
  while(loops < 4) {  //Blink the light 4 times once every second.
    light.setPixelColor(0, YELLOW);
    light.show();
    delay(500);
    light.setPixelColor(0, OFF);
    light.show();
    delay(500);
    loops++;
  }
  
  loops = 0;
  while(loops < 8) { //Blink the light 8 times over the course of one second
    light.setPixelColor(0, BLUE);
    light.show();
    delay(128);
    light.setPixelColor(0, OFF);
    light.show();
    delay(128);
    loops++;
  }
  
  Serial.print("Calibrating");
  light.setPixelColor(0, RED);
  light.show();
  
  light.setPixelColor(1, BLUE);
  
  uint32_t c;
  for(cal_int=0; cal_int<2000; cal_int++) {
    gyro_signalen();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    if(cal_int %16 == 0) {
      c = light.getPixelColor(1);
      light.setPixelColor(1, light.getPixelColor(2));
      light.setPixelColor(2, light.getPixelColor(3));
      light.setPixelColor(3, light.getPixelColor(4));
      light.setPixelColor(4, c);
      light.show();
    }
    delay(4);
  }
  Serial.println("\nDone");
  
  
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
  
  //Activate the Pin Change Interrupt for the Receiver.
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  PCMSK0 |= (1 << PCINT4);
  PCMSK0 |= (1 << PCINT5);
  
  
  bool connected;
  if(millis() - timer_6/1000 > 30) {
      light.setPixelColor(0, RED);
      light.setPixelColor(1, RED);
      light.setPixelColor(2, RED);
      light.setPixelColor(3, RED);
      light.setPixelColor(4, RED);
      light.show();
      connected = false;
    } else {
      light.setPixelColor(0, GREEN);
      light.setPixelColor(1, OFF);
      light.setPixelColor(2, OFF);
      light.setPixelColor(3, OFF);
      light.setPixelColor(4, OFF);
      light.show();
      connected = true;
    }
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    start ++;                                        //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                        //Wait 3 milliseconds before the next loop.
    
    if(connected == true && millis() - timer_6/1000 > 30) {
      light.setPixelColor(0, RED);
      light.setPixelColor(1, RED);
      light.setPixelColor(2, RED);
      light.setPixelColor(3, RED);
      light.setPixelColor(4, RED);
      light.show();
      connected = false;
    } else if(connected == false && millis() - timer_6/1000 <= 30){
      light.setPixelColor(0, GREEN);
      light.setPixelColor(1, OFF);
      light.setPixelColor(2, OFF);
      light.setPixelColor(3, OFF);
      light.setPixelColor(4, OFF);
      light.show();
      connected = true;
    }
  }
  Serial.println("Start!");
  start = 0;
  zero_timer = micros();                             //Set the zero_timer for the first loop.

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals 3.56V @ Analog 0.
  //12.6V equals 729 analogRead(0).
  //1260 / 729 = 1.7283.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.7283;
  Serial.print("Voltage: ");Serial.println(battery_voltage);

}

void loop() {
  loops++;
  
  gyro_signalen();
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.
  
  
  if(loops == 250) {
    loops = 0;
    
    
    
    if(!(last + 10 > receiver_input_channel_6 && last - 10 < receiver_input_channel_6) && start == 2) {
      brightness = map(constrain(receiver_input_channel_6, 1000, 2000), 1000, 2000, 0, 255);
      brightness = brightness < 10 ? 0 : brightness;
      
      light.setPixelColor(0, OFF);
      light.setPixelColor(2, light.Color(brightness, 0, 0));
      light.setPixelColor(3, light.Color(brightness, 0, 0));
      light.setPixelColor(1, light.Color(0, brightness, 0));
      light.setPixelColor(4, light.Color(0, brightness, 0));
      //Turn on the led if battery voltage is to low.
      if(battery_voltage < 1050 && battery_voltage > 600) {
        light.setPixelColor(0, YELLOW);
        light.setPixelColor(1, light.Color(0, 0, brightness));
        light.setPixelColor(4, light.Color(0, 0, brightness));
        
      } else if(millis() - timer_6/1000 > 30) {
        light.setPixelColor(0, RED);
        light.setPixelColor(3, RED);
        light.setPixelColor(2, RED);
        light.setPixelColor(1, RED);
        light.setPixelColor(4, RED);
      }
      light.show();
    } else if(battery_voltage < 1050 && battery_voltage > 600) {
      light.setPixelColor(0, YELLOW);
      light.setPixelColor(1, BLUE);
      light.setPixelColor(2, BLUE);
      light.setPixelColor(3, BLUE);
      light.setPixelColor(4, BLUE);
      light.show();
    } else if(millis() - timer_6/1000 > 30 ) {
      light.setPixelColor(0, RED);
      light.setPixelColor(3, RED);
      light.setPixelColor(2, RED);
      light.setPixelColor(1, RED);
      light.setPixelColor(4, RED);
      light.show();
    }
    //Serial.print("Throttle: ");Serial.print(throttle);Serial.print(", Yaw: ");Serial.print(yaw);Serial.print(", Pitch: ");Serial.print(pitch);Serial.print(", Roll: ");Serial.print(roll);Serial.print(", Voltage: ");Serial.println(battery_voltage);
  }
  
  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    //Reset the pid controllers for a bumpless start.
    light.setPixelColor(0, OFF);
    brightness = map(constrain(receiver_input_channel_6, 1000, 2000), 1000, 2000, 0, 255);
    brightness = brightness < 10 ? 0 : brightness;
      
    light.setPixelColor(3, light.Color(brightness, 0, 0));
    light.setPixelColor(2, light.Color(brightness, 0, 0));
    light.setPixelColor(1, light.Color(0, brightness, 0));
    light.setPixelColor(4, light.Color(0, brightness, 0));
    light.show();
    
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950) { 
    start = 0; 
    light.setPixelColor(0, GREEN);
    light.setPixelColor(3, OFF);
    light.setPixelColor(2, OFF);
    light.setPixelColor(1, OFF);
    light.setPixelColor(4, OFF);
    light.show();
  }
  
  
  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;
  
  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508)/3.0;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492)/3.0;
  
  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();
  
  
  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.04629 = 0.08 * 1.7283.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.04629;
  
  
  
  throttle = receiver_input_channel_3;
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
    
    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  
  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.
  
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
  //roll, pitch, throttle, yaw, vrb, vra;

}

void gyro_signalen() {
  Wire.beginTransmission(gyro_addr);                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                   //Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                            //End the transmission
  Wire.requestFrom(gyro_addr, 6);                    //Request 6 bytes from the gyro
  while(Wire.available() < 6);                       //Wait until the 6 bytes are received
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);               //Multiply highByte by 256 and ad lowByte
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;     //Only compensate after the calibration
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);              //Multiply highByte by 256 and ad lowByte
  gyro_pitch *= -1;                                  //Invert axis
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;   //Only compensate after the calibration
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_yaw = ((highByte<<8)|lowByte);                //Multiply highByte by 256 and ad lowByte
  gyro_yaw *= -1;                                    //Invert axis
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;       //Only compensate after the calibration
}

void print_output(){
  Serial.print("Pitch:");
  if(gyro_pitch >= 0)Serial.print("+");
  Serial.print(gyro_pitch/57.14286,0);               //Convert to degree per second
  if(gyro_pitch/57.14286 - 2 > 0)Serial.print(" NoU");
  else if(gyro_pitch/57.14286 + 2 < 0)Serial.print(" NoD");
  else Serial.print(" ---");
  Serial.print("  Roll:");
  if(gyro_roll >= 0)Serial.print("+");
  Serial.print(gyro_roll/57.14286,0);                //Convert to degree per second
  if(gyro_roll/57.14286 - 2 > 0)Serial.print(" RwD");
  else if(gyro_roll/57.14286 + 2 < 0)Serial.print(" RwU");
  else Serial.print(" ---");
  Serial.print("  Yaw:");
  if(gyro_yaw >= 0)Serial.print("+");
  Serial.print(gyro_yaw/57.14286,0);                 //Convert to degree per second
  if(gyro_yaw/57.14286 - 2 > 0)Serial.println(" NoR");
  else if(gyro_yaw/57.14286 + 2 < 0)Serial.println(" NoL");
  else Serial.println(" ---");
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

/**
 * Called when receiver inputs change state.
 */
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
  //Channel 5=========================================
  if(PINB & B00010000 ){                                       //Is input 11 high?
    if(last_channel_5 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_5 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_input_channel_5 = current_time - timer_5;         //Channel 4 is current_time - timer_4
  }
  //Channel 6=========================================
  if(PINB & B00100000 ){                                       //Is input 11 high?
    if(last_channel_6 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_6 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_input_channel_6 = current_time - timer_5;         //Channel 4 is current_time - timer_4
  }
}
