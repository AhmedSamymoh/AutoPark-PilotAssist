#include <MadgwickAHRS.h>
#include<Wire.h>

const int MPU_addr = 0x68, QMC_addr = 0x0D; // I2C address of the MPU-6050 and QMC
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MaX, MaY, MaZ ;
long GyX_cal = 0, GyY_cal = 0, GyZ_cal = 0, AcX_cal = 0, AcY_cal = 0;
unsigned long loop_timer=0, interval = 0;
Madgwick filter;
float calibrated_values[3];
float uncalibrated_values[3];
float roll = 0, pitch = 0, heading = 0;
void setup()
{
  
  Wire.begin();
  Serial.begin(2000000);
  MPU_init();
  QMC_init();
  Gyro_cal();
  //filter.begin(128);
 loop_timer = micros(); 
}
void loop()
{

//  MPU reading values
  MPU_read();
  
//  Gyroscope offset / calibration
  GyX -= GyX_cal;
  GyY -= GyY_cal;
  GyZ -= GyZ_cal;

// Accelerometer offset / calibration

  AcX -= AcX_cal;
  AcY -= AcY_cal;
  AcZ -= 0;
  

  GyX *=0.015267;  
  GyY *=0.015267;
  GyZ *=0.015267;


  QMC_read();
  
  uncalibrated_values[0]= MaX;
  uncalibrated_values[1]= MaY;
  uncalibrated_values[2]= MaZ;
  transformation(uncalibrated_values);
  MaX = calibrated_values[0] ;
   MaY = calibrated_values[1] ;
  MaZ = calibrated_values[2] ;


    filter.update(GyX, GyY, GyZ, AcX, AcY, AcZ, MaX, MaY, MaZ);

  
    
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    

    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
//    Serial.println(micros()-loop_timer);

  loop_timer = micros(); 
}





void MPU_init()
{
//Activate the MPU-6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
//Configure the accelerometer (+/-8g)
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission(true);
//Configure the gyro (500dps full scale)
  Wire.beginTransmission(MPU_addr);  
  Wire.write(0x1B);  
  Wire.write(0x08);   
  Wire.endTransmission(true);
}


void QMC_init()
{
  Wire.beginTransmission(QMC_addr); //start talking
  Wire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x01); // value
  Wire.endTransmission();
  Wire.beginTransmission(QMC_addr); //start talking
  Wire.write(0x09); // Tell the HMC5883 to Continuously Measure
  Wire.write(0x1D); // value
  Wire.endTransmission();
  
}



void MPU_read()
{ //Subroutine for reading the raw gyro and accelerometer data
   Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Wire.endTransmission(); 
}

void QMC_read()
{
  Wire.beginTransmission(QMC_addr); 
  Wire.write(0x00); 
  Wire.endTransmission();
  Wire.requestFrom(QMC_addr, 14); 
  while (Wire.available() < 3); 
  MaX = (float)(Wire.read() | Wire.read() <<8);
  MaY = (float)(Wire.read() | Wire.read() << 8);
  MaZ = (float)(Wire.read() | Wire.read() << 8);
  Wire.endTransmission();
}



void Gyro_cal()
{
  Serial.println("Wait. Calibrating IMU.");
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++)    //Run this code 2000 times
  {
    MPU_read();
    GyX_cal += GyX;
    GyY_cal += GyY; 
    GyZ_cal += GyZ;

    AcX_cal += AcX;
    AcY_cal += AcY;
    delay(3);
  }
  GyX_cal /= 2000;
  GyY_cal /= 2000;
  GyZ_cal /= 2000;
  AcX_cal /= 2000;
  AcY_cal /= 2000; 
  Serial.println("Calibration done!");
}


void transformation(float uncalibrated_values[3])    
{
  double calibration_matrix[3][3] = 
  {
 
    {1.169, -0.007, 0.147},
    {-0.041, 1.167, 0.085},
    {-0.028, -0.001, 1.398}  
  };

  double bias[3] = 
  {
    -563.145,
    -80.378,
    -31.331
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}