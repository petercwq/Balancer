#include <CmdMessenger.h>

// Arduino Pro Min 5V + L298N *2 + STH 39236 *2

#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro
#include <Stepper.h>

// This is the list of recognized commands. These can be commands that can either be sent or received.
// In order to receive, attach a callback function to these events
enum
{
  kAcknowledge,
  kError,
  kSetParams, // Command to request led to be set in specific state
  kGetParams,
  kMove,
  kCalibrate,
  kGetBattery
};

// #define DEBUG

const int battery_low_thresh = 1110, battery_high_thresh = 800, diode_voltage = 83;
const unsigned long voltage_loop_time_ms = 5000;
const unsigned long loop_time_us = 4000;

const int gyro_address = 0x68; //MPU-6050 I2C address (0x68 or 0x69)
long gyro_pitch_calib_value = -21, gyro_yaw_calib_value = -521, acc_z_calib_value = -400;
const int acc_raw_limit = 8200;

unsigned long next_loop_time_us, next_voltage_loop_time_ms, last_gyro_time_us, t;
int bat_vol;

float pid_p = 15, pid_i = 1.0, pid_d = 5;

// turn: 20 ~ 50, move: 50 ~ 150
float turn_speed = 30, move_speed = 30;

byte start, low_bat, receive_counter, move_byte, receive_index, receive_buffer[3], reply_buf[3] = {'$', 0, 0};

Stepper stepperL(200, 4, 5, 6, 7);
Stepper stepperR(200, 8, 9, 10, 12);

float angle_comp, angle_acc, angle_gyro;
const float stop_angle = 30, start_angle = 0.3;

unsigned long left_step_time_us, right_step_time_us, left_delay_mem_us, right_delay_mem_us;

float self_balance_pid_setpoint, pid_setpoint, pid_i_mem, pid_last_d_error, pid_error_temp, pid_output, pid_output_left, pid_output_right;
float pid_out_max = 400, pid_out_dead_band = 5;

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(OnUnknownCommand);
  cmdMessenger.attach(kSetParams, OnSetParams);
  cmdMessenger.attach(kGetParams, OnGetParams);
  cmdMessenger.attach(kMove, OnMove);
  cmdMessenger.attach(kCalibrate, OnCalibrate);
  cmdMessenger.attach(kGetBattery, OnGetBattery);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
  cmdMessenger.sendCmd(kError, "No attached callback");
}

// Callback function that sets led on or off
void OnSetParams()
{
  pid_p = cmdMessenger.readFloatArg();
  pid_i = cmdMessenger.readFloatArg();
  pid_d = cmdMessenger.readFloatArg();
  turn_speed = cmdMessenger.readFloatArg();
  move_speed = cmdMessenger.readFloatArg();
  cmdMessenger.sendCmd(kAcknowledge, "Params set");
}

// Callback function that sets leds blinking frequency
void OnGetParams()
{
  // Send back the result of the addition
  //cmdMessenger.sendCmd(kFloatAdditionResult,a + b);
  cmdMessenger.sendCmdStart(kGetParams);
  cmdMessenger.sendCmdArg(pid_p);
  cmdMessenger.sendCmdArg(pid_i);
  cmdMessenger.sendCmdArg(pid_d);
  cmdMessenger.sendCmdArg(turn_speed);
  cmdMessenger.sendCmdArg(move_speed);
  cmdMessenger.sendCmdEnd();
}

void OnMove()
{
  move_byte = cmdMessenger.readInt16Arg() & 0x0F;
  receive_counter = 0;
}

void OnCalibrate()
{
  calibrate(cmdMessenger.readInt16Arg());
  cmdMessenger.sendCmd(kAcknowledge, "Calibrated");
}

void OnGetBattery()
{
  cmdMessenger.sendCmd(kGetBattery, (float)(bat_vol / 100.0));
}

void calibrate(int16_t cmd)
{
  byte c_gyro, c_acc;
  if (cmd & 0b00000001)
  {
    c_gyro = 1;
    gyro_yaw_calib_value = 0;
    gyro_pitch_calib_value = 0;
  }
  if (cmd & 0b00000010)
  {
    c_acc = 1;
    acc_z_calib_value = 0;
  }
  int calibration_loops = 200;
  for (int counter = 0; counter < calibration_loops; counter++)
  {
    //Create calibration_loops loops
    if (counter % 10 == 0)
      digitalWrite(13, !digitalRead(13)); //Change the state of the LED every 15 loops to make the LED blink fast

    if (c_gyro)
    {
      Wire.beginTransmission(gyro_address); //Start communication with the gyro
      Wire.write(0x43);                     //Start reading the Who_am_I register 75h
      Wire.endTransmission();               //End the transmission
      Wire.requestFrom(gyro_address, 4);    //Request 2 bytes from the gyro
      gyro_yaw_calib_value += Wire.read() << 8 | Wire.read();
      gyro_pitch_calib_value += Wire.read() << 8 | Wire.read();
    }

    if (c_acc)
    {
      Wire.beginTransmission(gyro_address);
      Wire.write(0x3F); // Z axis
      Wire.endTransmission();
      Wire.requestFrom(gyro_address, 2);
      acc_z_calib_value += Wire.read() << 8 | Wire.read();
    }

    delay(10);
  }
  if (c_gyro)
  {
    gyro_pitch_calib_value /= calibration_loops;
    gyro_yaw_calib_value /= calibration_loops;
  }
  if (c_acc)
  {
    acc_z_calib_value /= calibration_loops;
  }
}

void setup()
{
  Serial.begin(115200);

  // Adds newline to every command
  cmdMessenger.printLfCr();

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Send the status to the PC that says the Arduino has booted
  // Note that this is a good debug function: it will let you also know
  // if your program had a bug and the arduino restarted
  cmdMessenger.sendCmd(kAcknowledge, "Arduino has started!");

  // LED pin
  pinMode(13, OUTPUT);

  // set speed to a very high value in rpm
  // add we will control speed instead of the stepper.h
  stepperL.setSpeed(400);
  stepperR.setSpeed(400);

  Wire.begin(); //Start the I2C bus as master
  TWBR = 12;    //Set the I2C clock speed to 400kHz

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x6B);                     //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();               //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1B);                     //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                     //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();               //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search.
  Wire.write(0x1C);                     //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                     //Set the register bits as 00001000 (+/- 4g full scale range), 8192 LSB/g
  Wire.endTransmission();               //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address); //Start communication with the address found during search
  Wire.write(0x1A);                     //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to 44Hz for Acc with 4.9ms delay, and 42Hz for Gyro with 4.8ms delay)
  Wire.endTransmission();               //End the transmission with the gyro

  calibrate(0x03);

  t = micros();
  left_step_time_us = t;
  right_step_time_us = t;
  last_gyro_time_us = t;
  next_loop_time_us = t + loop_time_us;
}

void updateAngle()
{
  Wire.beginTransmission(gyro_address); //Start communication with the gyro
  Wire.write(0x3F);                     //Start reading at register 0x3F
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(gyro_address, 2);    //Request 2 bytes from the gyro, ACCEL_ZOUT_H ACCEL_ZOUT_L
  //Prevent division by zero by limiting the acc data to +/-acc_raw_limit;
  //Calculate the current angle according to the accelerometer
  int acc_z = (Wire.read() << 8 | Wire.read()) - (int)acc_z_calib_value;
  acc_z = constrain(acc_z, -acc_raw_limit, acc_raw_limit);
  angle_acc = asin((float)acc_z / 8200.0) * 57.296;
#ifdef DEBUG
  Serial.print(F("acc ang:"));
  Serial.println(angle_acc);
#endif

  if (start == 0 && angle_acc > -start_angle && angle_acc < start_angle)
  {
    //If the accelerometer angle is almost 0
    angle_gyro = angle_acc; //Load the accelerometer angle in the angle_gyro variable
    last_gyro_time_us = micros();
    start = 1; //Set the start variable to start the PID controller
  }

  Wire.beginTransmission(gyro_address); //Start communication with the gyro
  Wire.write(0x43);                     //Start reading at register 0x43
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(gyro_address, 4);    //Request 4 bytes from the gyro, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L
  int gyro_yaw_data_raw = (Wire.read() << 8 | Wire.read()) - (int)gyro_yaw_calib_value;
  // 500°/s / 2^16 * loop_time = 0.00762939453125 * loop_time
  angle_gyro += ((Wire.read() << 8 | Wire.read()) - (int)gyro_pitch_calib_value) * 0.0076294f * (micros() - last_gyro_time_us) / 1000000.0f;
  //Calculate the traveled during this loop angle and add this to the angle_gyro variable
  last_gyro_time_us = micros();

#ifdef DEBUG
  Serial.print(F("gyro ang:"));
  Serial.println(angle_gyro);
#endif

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot.
  //This can be caused by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.
  //Uncomment the following line to make the compensation active
  //Compensate the gyro offset when the robot is rotating
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;
  //Correct the drift of the gyro angle with the accelerometer angle
  angle_comp = angle_gyro * 0.998 + angle_acc * 0.002;

  if (angle_comp > stop_angle || angle_comp < -stop_angle)
  {
    start = 0;
  }

#ifdef DEBUG
  Serial.print(F("comp ang:"));
  Serial.println(angle_comp);
#endif
}

void stop()
{
  pid_output = 0; //Set the PID controller output to 0 so the motors stop moving
  pid_i_mem = 0;  //Reset the I-controller memory
  pid_last_d_error = 0;
  self_balance_pid_setpoint = 0; //Reset the self_balance_pid_setpoint variable
  pid_setpoint = 0;
  pid_output_left = 0;
  pid_output_right = 0;
}

// STH 39D236, 1000ms ~ 4ms, vibration at 20~40ms
// speed from 1 ~ 250 step/sec
// PID from angle [-2, 2] map to speed [-500, 500]
void calcPid()
{
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_comp - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10)
    pid_error_temp += pid_output * 0.015;

  //Calculate the I-controller value and add it to the pid_i_mem variable
  pid_i_mem += pid_i * pid_error_temp;
  pid_i_mem = constrain(pid_i_mem, -pid_out_max, pid_out_max);

  //Calculate the PID output value
  pid_output = pid_p * pid_error_temp + pid_i_mem + pid_d * (pid_error_temp - pid_last_d_error);
  pid_output = constrain(pid_output, -pid_out_max, pid_out_max);

  pid_last_d_error = pid_error_temp; //Store the error for the next loop

  if (abs(pid_output) < pid_out_dead_band)
    pid_output = 0; //Create a dead-band to stop the motors when the robot is balanced

  pid_output_left = pid_output;  //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output; //Copy the controller output to the pid_output_right variable for the right motor

  //Serial.println(pid_output);
}

long pid2delay(float pid_value)
{
  if (pid_value != 0)
  {
    if (pid_value > 0)
      pid_value = (pid_out_max + pid_out_dead_band) - (1 / (pid_value + 9)) * 5500;
    else if (pid_value < 0)
      pid_value = -(pid_out_max + pid_out_dead_band) - (1 / (pid_value - 9)) * 5500;

    if (pid_value > 0)
      pid_value = pid_out_max - pid_value;
    else if (pid_value < 0)
      pid_value = -pid_out_max - pid_value;

    return (long)(pid_value * 100);
    //return (unsigned long)fabs(1000000.0f / pid_value); // + 1
  }
  return 0;
}

int pid2step(long delay_value)
{
  if (delay_value > 0)
    return 1;
  else if (delay_value < 0)
    return -1;
  else
    return 0;
}

void calcMove()
{
  if (move_byte & B00000001)
  {                                 //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turn_speed;  //Increase the left motor speed
    pid_output_right -= turn_speed; //Decrease the right motor speed
  }
  if (move_byte & B00000010)
  {                                 //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turn_speed;  //Decrease the left motor speed
    pid_output_right += turn_speed; //Increase the right motor speed
  }

  if (move_byte & B00000100)
  { //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint > -2.5)
      pid_setpoint -= 0.05; //Slowly change the setpoint angle so the robot starts leaning forewards
    if (pid_output > move_speed * -1)
      pid_setpoint -= 0.005; //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if (move_byte & B00001000)
  { //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint < 2.5)
      pid_setpoint += 0.05; //Slowly change the setpoint angle so the robot starts leaning backwards
    if (pid_output < move_speed)
      pid_setpoint += 0.005; //Slowly change the setpoint angle so the robot starts leaning backwards
  }

  if (!(move_byte & B00001100))
  { //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if (pid_setpoint > 0.5)
      pid_setpoint -= 0.05; //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (pid_setpoint < -0.5)
      pid_setpoint += 0.05; //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else
      pid_setpoint = 0; //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0)
  { //If the setpoint is zero degrees
    if (pid_output < 0) // 0.0015
      self_balance_pid_setpoint += 0.005; //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if (pid_output > 0)
      self_balance_pid_setpoint -= 0.005; //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }
}

void stepMotors()
{
  t = micros();
  if (t - left_step_time_us >= left_delay_mem_us)
  {
    left_step_time_us = t;
    long delay = pid2delay(pid_output_left);
    if (pid_output_left != 0)
      stepperL.step(pid2step(delay));
    left_delay_mem_us = abs(delay);
  }
  if (t - right_step_time_us >= right_delay_mem_us)
  {
    right_step_time_us = t;
    long delay = pid2delay(pid_output_right);
    if (pid_output_right != 0)
      stepperR.step(pid2step(delay));
    right_delay_mem_us = abs(delay);
  }
}

void updateBatVoltage()
{

  //Load the battery voltage to the battery_voltage variable.
  //diode_voltage_compensation is the voltage compensation for the diode.
  //Resistor voltage divider => (2k + 1k)/1k = 3
  //15V equals ~5V @ Analog 0.
  //15V equals 1023 analogRead(0).
  //1500 / 1023 = 1.466.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  if (millis() > next_voltage_loop_time_ms)
  {
    next_voltage_loop_time_ms = millis() + voltage_loop_time_ms;
    bat_vol = (int)(analogRead(0) * 1.466) + diode_voltage;
    if (bat_vol < battery_low_thresh) // && bat_vol > battery_high_thresh)
    {                                 //If batteryvoltage is below 11.1V and higher than 8.0V
      low_bat = 1;                    //Set the low_bat variable to 1
    }
#ifdef DEBUG
    Serial.print(F("vol: "));
    Serial.println(float(bat_vol) / 100);
#endif
  }
}

void loop()
{
  // Process incoming serial data, and perform callbacks
  cmdMessenger.feedinSerialData();

  updateBatVoltage();

  // reset move command
  if (receive_counter <= 10)
    receive_counter++;
  else
  {
    //After 100 (25 * loop_time) milliseconds the received byte is deleted
    move_byte = 0x00;
  }

  if (low_bat == 1)
  {
    stop();
    // flash LED when the battery is low
    digitalWrite(13, !digitalRead(13)); //Change the state of the LED every 15 loops to make the LED blink fast
    delay(200);
  }
  else
  {
    updateAngle();
    if (start == 1)
    {
      calcPid();
      calcMove();
      while (next_loop_time_us > micros() + 200)
      {
        stepMotors();
      }
    }
    else
    {
      stop();
    }
  }
  while (next_loop_time_us > micros())
    ;
  next_loop_time_us = micros() + loop_time_us;
}