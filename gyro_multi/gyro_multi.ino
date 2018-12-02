/* 
MPU-6050 Accelerometer + Gyro
-----------------------------

By arduino.cc user "Krodal".

June 2012
first version
July 2013 
The 'int' in the union for the x,y,z
changed into int16_t to be compatible
with Arduino Due.

Open Source / Public Domain

Using Arduino 1.0.1
It will not work with an older version, 
since Wire.endTransmission() uses a parameter 
to hold or release the I2C bus.

Documentation:
- The InvenSense documents:
- "MPU-6000 and MPU-6050 Product Specification",
PS-MPU-6000A.pdf
- "MPU-6000 and MPU-6050 Register Map and Descriptions",
RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
- "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
AN-MPU-6000EVB.p

The accuracy is 16-bits.

Temperature sensor from -40 to +85 degrees Celsius
340 per degrees, -512 at 35 degrees.

At power-up, all registers are zero, except these two:
Register 0x6B (PWR_MGMT_2) = 0x40 (I read zero).
Register 0x75 (WHO_AM_I) = 0x68.
*/

#include <Wire.h>
#include "mpu_6050_reg.h"

// declare an array of pin number of mpu-6050
const int pin[] = {2, 3, 4, 5};

// then calculate it's size. Now if you add a pin it will automatically include it
const int pinCount = sizeof(pin) / sizeof(pin[0]);

base_data base[pinCount];
last_data last[pinCount];

void setup()
{

 for (int i = 0; i < pinCount; i++)
 {
    pinMode(pin[i], OUTPUT);
 }

 Serial.begin(115200);
 Serial.println(F("InvenSense MPU-6050"));

 // Initialize the 'Wire' class for the I2C-bus.
 Wire.begin();
 
 int error;

 // Initialize MPU-6050
 for (int i = 0; i < pinCount; i++)
 {
    error = mpu_initialize(pin[i]);
    if(error != 0)
      Serial.println(F("MPU-6050 not connected!"));
    calibrate_sensors(pin[i], i);
    last[i].read_time = millis();
    last[i].x_angle = 0;
    last[i].y_angle = 0;
    last[i].z_angle = 0;
    last[i].gyro_x_angle = 0;
    last[i].gyro_y_angle = 0;
    last[i].gyro_z_angle = 0; 
 }

}


void loop()
{
 
 for (int i = 0; i < pinCount; i++)
 {
   accel_t_gyro_union accel_t_gyro;
   mpu_read_data(pin[i],(uint8_t*) &accel_t_gyro);
   // unsigned long t_now = millis();
   // complementary_filter(i, t_now, (uint8_t*) &accel_t_gyro);
   delay(5);
 }
 delay(1000);
}


void mpu_initialize(int mpu_en_pin)
{
  int error;
  uint8_t c;
  // default at power-up:
 // Gyro at 250 degrees second
 // Acceleration at 2g
 // Clock source at internal 8MHz
 // The device is in sleep mode.
 
 Serial.print("MPU-6050 pin ");
 Serial.print(mpu_en_pin,DEC);
 Serial.println(" Initializing...");

 output_all_low();
 digitalWrite(mpu_en_pin, HIGH);

 error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1, 0x69);
 Serial.print(F("WHO_AM_I : "));
 Serial.print(c,HEX);
 Serial.print(F(", error = "));
 Serial.println(error,DEC);

 // According to the datasheet, the 'sleep' bit
 // should read a '1'.
 // That bit has to be cleared, since the sensor
 // is in sleep mode at power-up. 
 error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1, 0x69);
 Serial.print(F("PWR_MGMT_1 : "));
 Serial.print(c,HEX);
 Serial.print(F(", error = "));
 Serial.println(error,DEC);

 // Clear the 'sleep' bit to start the sensor.
 MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0, 0x69);

 Serial.print("MPU-6050 pin ");
 Serial.print(mpu_en_pin,DEC);
 Serial.println(" Initialized!!");
 
}


void mpu_read_data(int mpu_en_pin, uint8_t* accel_t_gyro_ptr)
{
 int error;
 double dT;
 accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;

 
 //  Serial.println(F(""));
 //  Serial.print("MPU-6050 ");
 //  Serial.println(mpu_en_pin,DEC);

 output_all_low();
 digitalWrite(mpu_en_pin, HIGH);

 // Read the raw values.
 // Read 14 bytes at once, 
 // containing acceleration, temperature and gyro.
 // With the default settings of the MPU-6050,
 // there is no filter enabled, and the values
 // are not very stable.
 
 error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro), 0x69);
 //  Serial.print(F("Read accel, temp and gyro, error = "));
 //  Serial.println(error,DEC);


 // Swap all high and low bytes.
 // After this, the registers values are swapped, 
 // so the structure name like x_accel_l does no 
 // longer contain the lower byte.
 uint8_t swap;

 SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
 SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
 SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
 SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
 SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
 SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
 SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);
}

void calibrate_sensors(int mpu_en_pin, base_data base) {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;
  
  Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  mpu_read_data(mpu_en_pin, (uint8_t *) &accel_t_gyro);
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    mpu_read_data(mpu_en_pin, (uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base.x_accel = x_accel;
  base.y_accel = y_accel;
  base.z_accel = z_accel;
  base.x_gyro = x_gyro;
  base.y_gyro = y_gyro;
  base.z_gyro = z_gyro;
  
  //Serial.println("Finishing Calibration");
}

/* --------------------------------------------------------
complementary filter

Referenced by http://www.geekmomprojects.com/gyroscopes-and-accelerometers-on-a-chip/
-------------------------------------------------------- */

void complementary_filter(int mpu_en_pin_count, unsigned long time, uint8_t* accel_t_gyro_ptr)
{
 accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;
 
 // Convert gyro values to degrees/sec
 float FS_SEL = 131;
 float RADIANS_TO_DEGREES = 180/3.14159;

 float gyro_x = ((*accel_t_gyro).value.x_gyro - base[mpu_en_pin_count].x_gyro)/FS_SEL;
 float gyro_y = ((*accel_t_gyro).value.y_gyro - base[mpu_en_pin_count].y_gyro)/FS_SEL;
 float gyro_z = ((*accel_t_gyro).value.z_gyro - base[mpu_en_pin_count].z_gyro)/FS_SEL;
 float accel_x = (*accel_t_gyro).value.x_accel;
 float accel_y = (*accel_t_gyro).value.y_accel;
 float accel_z = (*accel_t_gyro).value.z_accel;

 float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
 float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
 float accel_angle_z = 0;

 // Compute the (filtered) gyro angles
 float dt = (time - last[mpu_en_pin_count].read_time)/1000.0;
 float gyro_angle_x = gyro_x*dt + last[mpu_en_pin_count].x_angle;
 float gyro_angle_y = gyro_y*dt + last[mpu_en_pin_count].y_angle;
 float gyro_angle_z = gyro_z*dt + last[mpu_en_pin_count].z_angle;

 // Compute the drifting gyro angles
 float unfiltered_gyro_angle_x = gyro_x*dt + last[mpu_en_pin_count].gyro_x_angle;
 float unfiltered_gyro_angle_y = gyro_y*dt + last[mpu_en_pin_count].gyro_y_angle;
 float unfiltered_gyro_angle_z = gyro_z*dt + last[mpu_en_pin_count].gyro_z_angle;
 
 // Apply the complementary filter to figure out the change in angle - choice of alpha is
 // estimated now.  Alpha depends on the sampling rate...
 float alpha = 0.96;
 float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
 float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
 float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

 last[mpu_en_pin_count].read_time = time;
 last[mpu_en_pin_count].x_angle = angle_x;
 last[mpu_en_pin_count].y_angle = angle_y;
 last[mpu_en_pin_count].z_angle = angle_z;
 last[mpu_en_pin_count].gyro_x_angle = unfiltered_gyro_angle_x;
 last[mpu_en_pin_count].gyro_y_angle = unfiltered_gyro_angle_y;
 last[mpu_en_pin_count].gyro_z_angle = unfiltered_gyro_angle_z;
 
 Serial.println(F(""));
 Serial.print("MPU-6050 ");
 Serial.print(mpu_en_pin_count,DEC);
 Serial.print(F("#FIL:"));             //Filtered angle
 Serial.print(angle_x, 2);
 Serial.print(F("\t  ,  "));
 Serial.print(angle_y, 2);
 Serial.print(F("\t  ,  "));
 Serial.print(angle_z, 2);
 Serial.println(F(""));

}


/* --------------------------------------------------------
output_all_low

Set all pins of connected mpu-6050 into LOW
-------------------------------------------------------- */
void output_all_low()
{
   for (int i = 0; i < pinCount; i++)
   {
      digitalWrite(pin[i], LOW);
      delay(1);
   }
}

/* --------------------------------------------------------
MPU6050_read

This is a common function to read multiple bytes 
from an I2C device.

It uses the boolean parameter for Wire.endTransMission()
to be able to hold or release the I2C-bus. 
This is implemented in Arduino 1.0.1.

Only this function is used to read. 
There is no function for a single byte.
-------------------------------------------------------- */

int MPU6050_read(int start, uint8_t *buffer, int size, int device_address)
{
 int i, n, error;

 Wire.beginTransmission(device_address);
 n = Wire.write(start);
 if (n != 1)
 return (-10);

 n = Wire.endTransmission(false); // hold the I2C-bus
 if (n != 0)
 return (n);

 // Third parameter is true: relase I2C-bus after data is read.
 Wire.requestFrom(device_address, size, true);
 i = 0;
 while(Wire.available() && i<size)
 {
 buffer[i++]=Wire.read();
 }
 if ( i != size)
 return (-11);

 return (0); // return : no error
}


/* --------------------------------------------------------
MPU6050_write

This is a common function to write multiple bytes to an I2C device.

If only a single register is written,
use the function MPU_6050_write_reg().

Parameters:
start : Start address, use a define for the register
pData : A pointer to the data to write.
size : The number of bytes to write.

If only a single register is written, a pointer
to the data has to be used, and the size is
a single byte:
int data = 0; // the data to write
MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
-------------------------------------------------------- */

int MPU6050_write(int start, const uint8_t *pData, int size, int device_address)
{
 int n, error;

 Wire.beginTransmission(device_address);
 n = Wire.write(start); // write the start address
 if (n != 1)
 return (-20);

 n = Wire.write(pData, size); // write data bytes
 if (n != size)
 return (-21);

 error = Wire.endTransmission(true); // release the I2C-bus
 if (error != 0)
 return (error);

 return (0); // return : no error
}

/* --------------------------------------------------------
MPU6050_write_reg

An extra function to write a single register.
It is just a wrapper around the MPU_6050_write()
function, and it is only a convenient function
to make it easier to write a single register.
-------------------------------------------------------- */

int MPU6050_write_reg(int reg, uint8_t data, int device_address)
{
 int error;

 error = MPU6050_write(reg, &data, 1, device_address);

 return (error);
}
