#define gyro_addr 0x6B
#define accel_addr 0x19
#define mag_addr 0x1E
#define slav_addr 0x7

#define accel_scale 4

#define x_cal 0 //0.48
#define y_cal 0 //.75
#define z_cal 0 //.185

int cal_int;
unsigned long UL_timer;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float gyro_roll, gyro_pitch;
double accel_x_cal, accel_y_cal, ccel_z_cal;
byte lowByte, highByte;
double x, y, z;

void initSensors() {
  //Initiate the Gyroscope Sensor
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x20);
    Wire.write(0x0F);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x23);
    Wire.write(0x80);
  Wire.endTransmission();
  
  //Calibrate the gyroscope
  for(cal_int=0; cal_int<2000; cal_int++) {
    gyro_signalen();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += yaw;
    delay(4);
  }
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
  
  
  //Initiate the Linear acceleration register
  Wire.beginTransmission(accel_addr);
    Wire.write(0x20);
    Wire.write(0b01110111);
  Wire.endTransmission();
  
  Wire.beginTransmission(accel_addr);
    Wire.write(0x23);
    Wire.write(0b100100);  //Set the Scale to +-4G
  Wire.endTransmission();
}

void signalen() {
  accel_signalen();
  gyro_signalen();
  
  pitch = 0.05*pitch + 0.95*gyro_pitch;
  roll = 0.05*pitch + 0.95*gyro_roll;
}

void accel_signalen() {
  Wire.beginTransmission(accel_addr);
  Wire.write(168);
  Wire.endTransmission();
  Wire.requestFrom(accel_addr, 6);
  while(Wire.available() < 6);
  lowByte = Wire.read();
  highByte = Wire.read();
  x = ((highByte<<8)|lowByte)/(32767.0/accel_scale)-x_cal;
  lowByte = Wire.read();
  highByte = Wire.read();
  y = ((highByte<<8)|lowByte)/(32767.0/accel_scale)-y_cal;
  lowByte = Wire.read();
  highByte = Wire.read();
  z = ((highByte<<8)|lowByte)/(32767.0/accel_scale)-z_cal;
  
  // apply trigonometry to get the pitch and roll:
  pitch = atan(x/sqrt(pow(y,2) + pow(z,2)));
  roll = atan(y/sqrt(pow(x,2) + pow(z,2)));
  //convert radians into degrees
  pitch *= (180.0/PI);
  roll *= (180.0/PI) ;
}

/**
 *  Read the gyroscope data
 */
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
  yaw = ((highByte<<8)|lowByte);                //Multiply highByte by 256 and ad lowByte
  yaw *= -1;                                    //Invert axis
  if(cal_int == 2000)yaw -= gyro_yaw_cal;       //Only compensate after the calibration
}

