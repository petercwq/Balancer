#include <Wire.h> //Include the Wire.h library so we can communicate with the gyro
#include <Stepper.h>

const int battery_low_threshold = 1050, diode_voltage_compensation = 83;
const unsigned long angle_loop_time = 4000, voltage_loop_time = 5000000;
const int calibration_loops = 500;
const int acc_raw_limit = 8200;
// const int motor_interval = 500;
const int motor_speed = 90;    // rpms
const int gyro_address = 0x68; //MPU-6050 I2C address (0x68 or 0x69)

//Various settings
float pid_p_gain = 15;        //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;       //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;        //Gain setting for the D-controller (30)
float turning_speed = 30;     //Turning speed (20)
float max_target_speed = 150; //Max target speed (100)

unsigned long last_angle_loop_time, last_voltage_loop_time, left_motor_start_time, right_motor_start_time;
unsigned long time;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_right_motor_memory;

// AccelStepper stepperL(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
// AccelStepper stepperR(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
Stepper stepperL(200, 2, 3, 4, 5);
Stepper stepperR(200, 6, 7, 8, 9);

int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value, acc_calibration_value;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  Serial.println("---begin setup---");
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

  pinMode(13, OUTPUT); //Configure digital poort 13 as output

  Serial.println("---calibrate---");
  for (receive_counter = 0; receive_counter < calibration_loops; receive_counter++)
  {
    //Create calibration_loops loops
    if (receive_counter % 15 == 0)
      digitalWrite(13, !digitalRead(13));                           //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                           //Start communication with the gyro
    Wire.write(0x43);                                               //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                         //End the transmission
    Wire.requestFrom(gyro_address, 4);                              //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();   //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read(); //Combine the two bytes to make one integer

    Wire.beginTransmission(gyro_address);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 2);
    acc_calibration_value -= Wire.read() << 8 | Wire.read();
    delayMicroseconds(angle_loop_time - 300); //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= calibration_loops; //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= calibration_loops;   //Divide the total value by 500 to get the avarage gyro offset
  acc_calibration_value /= calibration_loops;

  stepperL.setSpeed(motor_speed);
  stepperR.setSpeed(motor_speed);

  Serial.print("gyro_pitch_calibration_value:");
  Serial.println(gyro_pitch_calibration_value);
  Serial.print("gyro_yaw_calibration_value:");
  Serial.println(gyro_yaw_calibration_value);
  Serial.print("acc_calibration_value:");
  Serial.println(acc_calibration_value);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  if (Serial.available())
  {                                //If there is serial data available
    received_byte = Serial.read(); //Load the received serial data in the received_byte variable
    receive_counter = 0;           //Reset the receive_counter variable
  }
  if (receive_counter <= 25)
    receive_counter++; //The received byte will be valid for 25 program loops (100 milliseconds)
  else
    received_byte = 0x00; //After 100 (25 * angle_loop_time) milliseconds the received byte is deleted

  //Load the battery voltage to the battery_voltage variable.
  //diode_voltage_compensation is the voltage compensation for the diode.
  //Resistor voltage divider => (2k + 1k)/1k = 3
  //15V equals ~5V @ Analog 0.
  //15V equals 1023 analogRead(0).
  //1500 / 1023 = 1.466.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.

  unsigned long t = micros();
  if (t - last_voltage_loop_time > voltage_loop_time)
  {
    battery_voltage = (analogRead(0) * 1.466) + diode_voltage_compensation;
    if (battery_voltage < battery_low_threshold)
    {                         //If batteryvoltage is below 10.5V and higher than 8.0V
      digitalWrite(13, HIGH); //Turn on the led if battery voltage is too low
      low_bat = 1;            //Set the low_bat variable to 1
      Serial.print("low battery voltage:");
      Serial.println(battery_voltage);
    }
    last_voltage_loop_time = t;
  }

  t = micros();
  if (t - last_angle_loop_time > angle_loop_time)
  {
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Angle calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Wire.beginTransmission(gyro_address);                    //Start communication with the gyro
    Wire.write(0x3F);                                        //Start reading at register 0x3F
    Wire.endTransmission();                                  //End the transmission
    Wire.requestFrom(gyro_address, 2);                       //Request 2 bytes from the gyro, ACCEL_ZOUT_H ACCEL_ZOUT_L
    accelerometer_data_raw = Wire.read() << 8 | Wire.read(); //Combine the two bytes to make one integer
    accelerometer_data_raw += (int)acc_calibration_value;    //Add the accelerometer calibration value

    //Prevent division by zero by limiting the acc data to +/-acc_raw_limit;
    accelerometer_data_raw = constrain(accelerometer_data_raw, -acc_raw_limit, acc_raw_limit);
    angle_acc = asin((float)accelerometer_data_raw / acc_raw_limit) * 57.296; //Calculate the current angle according to the accelerometer

    if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5)
    {
      //If the accelerometer angle is almost 0
      angle_gyro = angle_acc; //Load the accelerometer angle in the angle_gyro variable
      start = 1;              //Set the start variable to start the PID controller
    }

    Wire.beginTransmission(gyro_address);                 //Start communication with the gyro
    Wire.write(0x43);                                     //Start reading at register 0x43
    Wire.endTransmission();                               //End the transmission
    Wire.requestFrom(gyro_address, 4);                    //Request 4 bytes from the gyro, GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L
    gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();   //Combine the two bytes to make one integer
    gyro_pitch_data_raw = Wire.read() << 8 | Wire.read(); //Combine the two bytes to make one integer

    gyro_pitch_data_raw -= gyro_pitch_calibration_value;                                           //Add the gyro calibration value
                                                                                                   // 500°/s / 2^16 * angle_loop_time = 0.00762939453125 * angle_loop_time
    angle_gyro += gyro_pitch_data_raw * 0.0076294 * (float)(t - last_angle_loop_time) / 1000000.0; //Calculate the traveled during this loop angle and add this to the angle_gyro variable
    last_angle_loop_time = t;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //MPU-6050 offset compensation
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Not every gyro is mounted 100% level with the axis of the robot.
    //This can be caused by misalignments during manufacturing of the breakout board.
    //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
    //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
    //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

    gyro_yaw_data_raw -= gyro_yaw_calibration_value; //Add the gyro calibration value
    //Uncomment the following line to make the compensation active
    //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

    angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004; //Correct the drift of the gyro angle with the accelerometer angle

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //PID controller calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
    //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
    //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
    pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
    if (pid_output > 10 || pid_output < -10)
      pid_error_temp += pid_output * 0.015;

    pid_i_mem += pid_i_gain * pid_error_temp;    //Calculate the I-controller value and add it to the pid_i_mem variable
    pid_i_mem = constrain(pid_i_mem, -400, 400); //Limit the I-controller to the maximum controller output

    //Calculate the PID output value
    pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
    pid_output = constrain(pid_output, -400, 400); //Limit the PI-controller to the maximum controller output

    pid_last_d_error = pid_error_temp; //Store the error for the next loop

    if (pid_output < 5 && pid_output > -5)
      pid_output = 0; //Create a dead-band to stop the motors when the robot is balanced

    if (angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1)
    {                                //If the robot tips over or the start variable is zero or the battery is empty
      pid_output = 0;                //Set the PID controller output to 0 so the motors stop moving
      pid_i_mem = 0;                 //Reset the I-controller memory
      start = 0;                     //Set the start variable to 0
      self_balance_pid_setpoint = 0; //Reset the self_balance_pid_setpoint variable
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Control calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pid_output_left = pid_output;  //Copy the controller output to the pid_output_left variable for the left motor
    pid_output_right = pid_output; //Copy the controller output to the pid_output_right variable for the right motor

    if (received_byte & B00000001)
    {                                    //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
      pid_output_left += turning_speed;  //Increase the left motor speed
      pid_output_right -= turning_speed; //Decrease the right motor speed
    }
    if (received_byte & B00000010)
    {                                    //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
      pid_output_left -= turning_speed;  //Decrease the left motor speed
      pid_output_right += turning_speed; //Increase the right motor speed
    }

    if (received_byte & B00000100)
    { //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
      if (pid_setpoint > -2.5)
        pid_setpoint -= 0.05; //Slowly change the setpoint angle so the robot starts leaning forewards
      if (pid_output > max_target_speed * -1)
        pid_setpoint -= 0.005; //Slowly change the setpoint angle so the robot starts leaning forewards
    }
    if (received_byte & B00001000)
    { //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
      if (pid_setpoint < 2.5)
        pid_setpoint += 0.05; //Slowly change the setpoint angle so the robot starts leaning backwards
      if (pid_output < max_target_speed)
        pid_setpoint += 0.005; //Slowly change the setpoint angle so the robot starts leaning backwards
    }

    if (!(received_byte & B00001100))
    { //Slowly reduce the setpoint to zero if no foreward or backward command is given
      if (pid_setpoint > 0.5)
        pid_setpoint -= 0.05; //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
      else if (pid_setpoint < -0.5)
        pid_setpoint += 0.05; //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
      else
        pid_setpoint = 0; //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
    }

    //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter.
    //This way the robot will always find it's balancing point
    if (pid_setpoint == 0)
    { //If the setpoint is zero degrees
      if (pid_output < 0)
        self_balance_pid_setpoint += 0.0015; //Increase the self_balance_pid_setpoint if the robot is still moving forewards
      if (pid_output > 0)
        self_balance_pid_setpoint -= 0.0015; //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Motor pulse calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
    if (pid_output_left > 0)
      pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
    else if (pid_output_left < 0)
      pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

    if (pid_output_right > 0)
      pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
    else if (pid_output_right < 0)
      pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

    //Calculate the needed pulse time for the left and right stepper motor controllers
    if (pid_output_left > 0)
      left_motor = 400 - pid_output_left;
    else if (pid_output_left < 0)
      left_motor = -400 - pid_output_left;
    else
      left_motor = 0;

    if (pid_output_right > 0)
      right_motor = 400 - pid_output_right;
    else if (pid_output_right < 0)
      right_motor = -400 - pid_output_right;
    else
      right_motor = 0;

    //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
    throttle_left_motor = left_motor * 20;   // 20 us
    throttle_right_motor = right_motor * 20; // 20 us
#if DEBUG
    time = micros() - t;
    Serial.print("left_motor:");
    Serial.println(left_motor);
    Serial.print("right_motor:");
    Serial.println(right_motor);

    Serial.print("angle calculation takes: ");
    Serial.println(time);
#endif

    t = micros();
    if (t - left_motor_start_time > throttle_left_motor_memory)
    {
      left_motor_start_time = t;
      throttle_left_motor_memory = throttle_left_motor;
      if (throttle_left_motor_memory < 0)
      {
        throttle_left_motor_memory = -throttle_left_motor_memory;
        stepperL.step(-1);
      }
      else if (throttle_left_motor_memory > 0)
      {
        stepperL.step(1);
      }
    }

#if DEBUG
    time = micros() - t;
    Serial.print("left_motor takes: ");
    Serial.println(time);
#endif

    t = micros();
    if (t - right_motor_start_time > throttle_right_motor_memory)
    {
      right_motor_start_time = t;
      throttle_right_motor_memory = throttle_right_motor;
      if (throttle_right_motor_memory < 0)
      {
        throttle_right_motor_memory = -throttle_right_motor_memory;
        stepperR.step(1);
      }
      else if (throttle_right_motor_memory > 0)
      {
        stepperR.step(-1);
      }
    }

#if DEBUG
    time = micros() - t;
    Serial.print("right_motor takes: ");
    Serial.println(time);
#endif
  }
}