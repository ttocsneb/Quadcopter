#define gyro_addr 0x6B
#define accel_addr 0x19
#define mag_addr 0x1E
#define slav_addr 0x7

#define accel_scale 8

#define x_cal 0 //0.48
#define y_cal 0 //.75
#define z_cal 0 //.185

int cal_int;
unsigned long UL_timer;
long gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
double x, y, z;
double accel_x_cal, accel_y_cal, ccel_z_cal;
int gyro_roll, gyro_pitch;
float gyro_pitch_i, gyro_roll_i;

/////////////

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z;

long gyro_x_cal, gyro_y_cal, gyro_z_cal;

float angle_pitch, angle_roll;

int angle_pitch_buffer, angle_roll_buffer;

////////////

byte lowByte, highByte;

void initSensors() {

  //shortBeep();
  
  //Initiate the Gyroscope Sensor
  /*
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x20);
    Wire.write(0x0F);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x23);
    Wire.write(0x80);
  Wire.endTransmission();
/**/
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x20);
    Wire.write(0b00001111);
  Wire.endTransmission();
  Wire.beginTransmission(gyro_addr);
    Wire.write(0x23);
    Wire.write(0b10000000);
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
    Wire.write(0b01111111);
  Wire.endTransmission();
  
  Wire.beginTransmission(accel_addr);
    Wire.write(0x23);
    Wire.write(0b10100000);  //Set the Scale to +-4G
  Wire.endTransmission();

  accel_signalen();

  gyro_pitch_i = x;
  gyro_roll_i = y;
}

void signalen() {
  accel_signalen();
  gyro_signalen();
  

  gyro_roll -= gyro_roll_cal;
  gyro_pitch -= gyro_pitch_cal;
  yaw -= gyro_yaw_cal;


  gyro_roll_i  += gyro_roll  * 0.0000598;//= 1 / 250Hz / 66.87units/deg/s
  gyro_pitch_i += gyro_pitch * 0.0000598;

  gyro_pitch_i -= gyro_roll_i  * sin(yaw * 0.000001043842);//= 0.0000598 * (3.142(PI) / 180deg)
  gyro_roll_i  += gyro_pitch_i * sin(yaw * 0.000001043842);
  
  
  pitch = 0.2*x + 0.80*gyro_pitch_i;
  roll =  0.2*y + 0.80*gyro_roll_i;
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
  x = atan(x/sqrt(y*y + z*z));
  y = atan(y/sqrt(x*x + z*z));
  //convert radians into degrees
  x *= (180.0/PI);
  y *= (180.0/PI);
}

/**
 *  Read the gyroscope data
 */
void gyro_signalen() {
  Wire.beginTransmission(gyro_addr);                 //Start communication with the gyro (adress 1101001)
  Wire.write(168);                                   //Start reading @ register 28h and auto increment with every read
  Wire.requestFrom(gyro_addr, 6);                    //Request 6 bytes from the gyro                  //End the transmission
  Wire.endTransmission();                            //End the transmission
  
  while(Wire.available() < 6);                       //Wait until the 6 bytes are received
  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_roll = ((highByte<<8)|lowByte);               //Multiply highByte by 256 and ad lowByte

  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  gyro_pitch = ((highByte<<8)|lowByte);              //Multiply highByte by 256 and ad lowByte
  gyro_pitch *= -1;                                  //Invert axis

  lowByte = Wire.read();                             //First received byte is the low part of the angular data
  highByte = Wire.read();                            //Second received byte is the high part of the angular data
  yaw = ((highByte<<8)|lowByte);                //Multiply highByte by 256 and ad lowByte
  yaw *= -1;                                    //Invert axis
}

