
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Stepper.h>

#define RESTRICT_PITCH

Kalman kalmanX;
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;   // Gyroscope angle
double compAngleX, compAngleY;   // Complementary filter angle
double kalAngleX, kalAngleY;     // Angle after Kalman filter
double corrected_x, corrected_y; // Corrected with offset

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

char a;
double m = 0.7;
double m1 = -0.7;
int d = 0;
int c = 0;
char p;

Stepper stepperL(200, 2, 3, 4, 5);
Stepper stepperR(200, 6, 7, 8, 9);

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

//------------------------------------------------------------------------------
void setup()
{
    // Start serial console
    Serial.begin(115200);
    //BT.begin(9600);
    delay(50);
    // Initiate the Wire library and join the I2C bus as a master or slave
    Wire.begin();

    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

    i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to 250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to 2g

    while (i2cWrite(0x19, i2cData, 4, false))
        ; // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true))
        ; // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1))
        ;
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1)
            ;
    }
    delay(100); // Wait for sensor to stabilize

    /** 
* Set kalman and gyro starting angle
*
*/
    while (i2cRead(0x3B, i2cData, 6))
        ;
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

// atan2 outputs the value of - to  (radians) - see http://en.wikipedia.org/wiki/Atan2
// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;
    timer = micros();
}

//------------------------------------------------------------------------------
void loop()
{
    while (i2cRead(0x3B, i2cData, 14))
        ;
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];
    // Calculate delta time
    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

#ifdef RESTRICT_PITCH
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    }
    else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
    {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    }
    else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate;                        // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
    delay(2);
    Serial.println();
    // Corrected angles with offset
    corrected_x = kalAngleX - 171, 746;
    corrected_y = kalAngleY - 81, 80;
    corrected_y = corrected_y + 84;
    Serial.print(corrected_y);

    speed_adjust(corrected_y);
    if (corrected_y >= m && corrected_y < 20)
    {
        if (c > 6)
        {
            m -= 0.2;
            m1 -= -0.2;
            c = 0;
        }
        backward();
    }
    else if (corrected_y >= -20 && corrected_y <= m1)
    {
        Serial.print("  ");
        if (d > 6)
        {
            m += 0.2;
            m1 += 0.2;
            d = 0;
        }
        forward();
    }
    else
    {
        m = 0.7;
        m1 = -0.7;
    }
}

void forward()
{
    d++;
    stepperL.step(1);
    stepperL.step(-1);
}

void backward()
{
    c++;
    stepperL.step(-1);
    stepperL.step(1);
}

void speed_adjust(int value_y)
{
    if (value_y >= -1 && value_y <= 1)
    {
        stepperL.setSpeed(5);
        stepperR.setSpeed(5);
    }
    else if ((value_y >= -3 && value_y < -1) || (value_y > 1 && value_y <= 3))
    {
        stepperL.setSpeed(50);
        stepperR.setSpeed(50);
    }
    else if (value_y > 5 || value_y <= -5)
    {
        stepperL.setSpeed(120);
        stepperR.setSpeed(120);
    }
    else
    {
        stepperL.setSpeed(0);
        stepperR.setSpeed(0);
    }
}
