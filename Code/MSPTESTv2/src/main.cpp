#include <Arduino.h>
#include <Wire.h>

//Gyro Setup & Config
#define MPU9250_I2C_address                   0x68
#define MPU9250_I2C_master_enable             0x6A
#define MPU9250_Interface_bypass_mux_enable   0x37

#define Frequency                             125
#define Sensitivity                           65.5

#define Raw_sensor_to_deg 1/(Sensitivity*Frequency)
#define Raw_sensor_to_rad Raw_sensor_to_deg*DEG_TO_RAD

#define Loop_time                             1000000/Frequency
long    Loop_start;

int     gx, gy, gz;
long    gx_cal, gy_cal, gz_cal;
float   g_pitch, g_roll, g_yaw;
float   g_pitch_out, g_roll_out;

//Accelerometer Setup
long    ax, ay, az, a_total_vector;
float   a_pitch, a_roll;

//Magnetometer Setup & Config
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM

// ----- Compass heading
/*
  The magnetic declination for Lower Hutt, New Zealand is +22.5833 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/
float   declination = +22.5833;                                             //  Degrees ... replace this declination with yours
int     heading;

int     mx, my, mz;
float   mx_dampened, my_dampened, mz_dampened;
float   mx_hor, my_hor;
float   m_pitch, m_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/
bool    Record_data = false;
int     mx_offset = 46,   my_offset = 190,  mz_offset = -254;   // Hard-iron offsets
float   mx_scale = 1.01,  my_scale = 0.99,  mz_scale = 1.00;    // Soft-iron scale factors
float   ASAX = 1.17,      ASAY = 1.18,      ASAZ = 1.14;        // (A)sahi (S)ensitivity (A)djustment fuse ROM values.

// ----- LED
const int LED = 13;                     // Status LED

// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug
long Loop_start_time;
long Debug_start_time;

void configure_magnetometer(){

  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();
  
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
  delay(100); 
}

void calibrate_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register

  int mag_x_min =  32767;                                         // Raw data extremes
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;

  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;

  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer

    // ----- Point to status register 1
    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
    delay(4);                                                     // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  mx_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  my_offset = (mag_y_max + mag_y_min) / 2;
  mz_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

  mx_scale = chord_average / chord_x;                          // Calculate X scale factor
  my_scale = chord_average / chord_y;                          // Calculate Y scale factor
  mz_scale = chord_average / chord_z;                          // Calculate Z scale factor

  // ----- Record magnetometer offsets
  /*
     When active this feature sends the magnetometer data
     to the Serial Monitor then halts the program.
  */
  if (Record_data == true)
  {
    // ----- Display data extremes
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");

    // ----- Display hard-iron offsets
    Serial.print("Hard-iron: ");
    Serial.print(mx_offset); Serial.print("\t");
    Serial.print(my_offset); Serial.print("\t");
    Serial.println(mz_offset);
    Serial.println("");

    // ----- Display soft-iron scale factors
    Serial.print("Soft-iron: ");
    Serial.print(mx_scale); Serial.print("\t");
    Serial.print(my_scale); Serial.print("\t");
    Serial.println(mz_scale);
    Serial.println("");

    // ----- Display fuse ROM values
    Serial.print("ASA: ");
    Serial.print(ASAX); Serial.print("\t");
    Serial.print(ASAY); Serial.print("\t");
    Serial.println(ASAZ);

    // ----- Halt program
    while (true);                                       // Wheelspin ... program halt
  }
}

void read_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;

  // ----- Point to status register 1
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      mx = (mag_x - mx_offset) * mx_scale;
      my = (mag_y - my_offset) * my_scale;
      mz = (mag_z - mz_offset) * mz_scale;
    }
  }
}

void config_gyro()
{
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}

void read_mpu_6050_data()
{
  /*
    Subroutine for reading the raw gyro and accelerometer data
  */

  // ----- Locals
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  ax = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  ay = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  az = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  gx = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  gz = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  gz = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}

void calibrate_gyro()
{

  // ----- LED Status (ON = calibration start)
  pinMode(LED, OUTPUT);                                 //Set LED (pin 13) as output
  digitalWrite(LED, HIGH);                              //Turn LED on ... indicates startup

  // ----- Calibrate gyro
  for (int counter = 0; counter < 2000 ; counter ++)    //Run this code 2000 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    gx_cal += gx;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    gy_cal += gy;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    gz_cal += gz;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  gx_cal /= 2000;                                   //Divide the gyro_x_cal variable by 2000 to get the average offset
  gy_cal /= 2000;                                   //Divide the gyro_y_cal variable by 2000 to get the average offset
  gz_cal /= 2000;                                   //Divide the gyro_z_cal variable by 2000 to get the average offset

  // ----- Status LED
  digitalWrite(LED, LOW);                               // Turn LED off ... calibration complete
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(A0, INPUT_PULLUP);
  digitalWrite(LED, LOW);

  configure_magnetometer();

  if (Record_data == true)
  {
    calibrate_magnetometer();
  }

  config_gyro();

  calibrate_gyro();

  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time = micros();  
}




void loop() {
  // put your main code here, to run repeatedly:
  read_mpu_6050_data();

  gx -= gx_cal;
  gy -= gy_cal;
  gz -= gz_cal;

  g_pitch += -gy*Raw_sensor_to_deg;
  g_roll += gx*Raw_sensor_to_deg;
  g_yaw += gz*Raw_sensor_to_deg;

  g_pitch += g_roll * sin(gz*Raw_sensor_to_rad);
  g_roll -= g_pitch * sin(gz*Raw_sensor_to_rad);

  a_total_vector = sqrt((ax*ax)+(ay*ay)+(az*az));
  a_pitch = asin((float)ax/a_total_vector)*RAD_TO_DEG;
  a_roll = asin((float)ay/a_total_vector)*RAD_TO_DEG;

  a_pitch -= 0.0f;
  a_roll -= 0.0f;

  if(Gyro_synchronised){
    g_pitch = g_pitch * 0.9996 + a_pitch * 0.0004;
    g_roll = g_roll * 0.9996 + a_roll * 0.0004;
  } else {
    g_pitch = a_pitch;
    g_roll = a_roll;
    Gyro_synchronised = true;
  }

  g_pitch_out = g_pitch_out * 0.9 + g_pitch * 0.1;
  g_roll_out = g_roll_out * 0.9 + g_roll * 0.1;

  read_magnetometer();

  m_pitch = -g_roll_out * DEG_TO_RAD;
  m_roll = g_pitch_out * DEG_TO_RAD;

  mx_hor = mx * cos(m_pitch) + my * sin(m_roll) * sin(m_pitch) - mz * cos(m_roll) * sin(m_pitch);
  my_hor = my * cos(m_roll) + mz * sin(m_roll);

  if(!(digitalRead(A0))){
    mx_hor = mx;
    my_hor = my;
  }

  mx_dampened = mx_dampened * 0.9 + mx_hor * 0.1;
  my_dampened = my_dampened * 0.9 + my_hor * 0.1;

  heading = atan2(mx_dampened, my_dampened) * RAD_TO_DEG;
  heading += declination;

  if(heading > 360.0){heading -= 360.0;}
  if(heading < 0.0){heading += 360.0;}

  if(heading < 0){heading += 360;}
  if(heading >= 360){heading -= 360;}

  while((micros() - Loop_start_time) < 8000);
  Loop_start_time = micros();

}