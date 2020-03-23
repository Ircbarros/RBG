// Libraries
#include <Pushbutton.h>
#include <ZumoMotors.h>
#include <Wire.h>
#include <L3G.h>
#include <QTRSensors.h>

// Defines
#define LED_PIN 13
#define n_ANGLE_THRESHOLD 2.0
#define n_SPEED_REFERENCE 100
#define n_SPEED_MAX 200
#define n_QTRRC_THRESHOLD 500

// QRTRC
#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// Objects
L3G gyro;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

// QRTRC
QTRSensorsRC qtrrc((unsigned char[]) {5, A2, A0, 11, A3, 4}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

// Global Variables
// Gyroscope OFFSET
float gyroOffsetZ;
// Gyroscope ANGLE
float angle = 0;

// Instrumentation Initialization
void setup()
{
  // LED PIN 13
  pinMode(LED_PIN, OUTPUT);

  // Serial Initialization
  Serial.begin(9600);

  // I2C Initialization
  Wire.begin();

  // Set up the L3GD20H gyro.
  gyro.init();

  // 800 Hz output data rate,
  // low-pass filter cutoff 100 Hz.
  gyro.writeReg(L3G::CTRL1, 0b11111111);

  // 2000 dps full scale.
  gyro.writeReg(L3G::CTRL4, 0b00100000);

  // High-pass filter disabled.
  gyro.writeReg(L3G::CTRL5, 0b00000000);

  // Calibrate the gyro.
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!gyro.readReg(L3G::STATUS_REG) & 0x08);
    gyro.read();

    // Add the Y axis reading to the total.
    gyroOffsetZ += gyro.g.z;
  }
  gyroOffsetZ /= 1024;

  // Calibrate the reflectance sensor array
  delay(1000);
  digitalWrite(13, HIGH);   
  // make the calibration take about 5 seconds
  // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  for (int i = 0; i < 200; i++)  
    qtrrc.calibrate();       
  digitalWrite(13, LOW); 
}

// Global Variables
int n_state = 0;
int n_speed = 0;
float n_angle = 0, n_angleTarget = 0;
int counter=0;

// PID variables
int n_kp = 10;
float n_error = 0;

// Main Loop
void loop()
{
    button.waitForButton();
    // Update control output at 50 Hz (20 ms)
    static uint8_t lastCorrectionTime = 0;
    uint8_t m = millis();
    if ((uint8_t)(m - lastCorrectionTime) >= 20)
    {
      // Read calibrated sensor values and obtain a measure of the line position from 0 to 5000
      unsigned int position = qtrrc.readLine(sensorValues);
      
      // Update the angle using the gyro as often as possible.
      updateAngleGyro();
            
      if(counter++<4000){
      // Finite state machine
      if (n_state == 0)
      {
        // State 0
        //Serial.println("State 0 ");
        
         // Target angle
         n_angleTarget = 0;
               
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
  
        // PID
        n_speed = n_error * n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX + n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        // Set motors
        motors.setSpeeds(n_SPEED_REFERENCE + n_speed, n_SPEED_REFERENCE - n_speed);
   
      
        // Transition - Border detection
        //if (button.getSingleDebouncedRelease())
        if (sensorValues[0] > n_QTRRC_THRESHOLD)
         {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 0 ");
          
          // Set state 1
          n_state = 1;

          // Target angle
          n_angleTarget = fmod(n_angle + 90, 360);
        }
      }
      else if (n_state == 1)
      {
        // State 1
        //Serial.println("State 1 ");
               
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
  
        // PID
        n_speed = n_error * -n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX + n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        float x = -n_speed;
        float y = n_SPEED_REFERENCE+n_speed;
        if(x>-100){
          x=-100;
        }
        // Set motors
        motors.setSpeeds(x,n_SPEED_REFERENCE+n_speed);
   

        // Angle detection
        if (abs(n_error) < n_ANGLE_THRESHOLD)
        {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 1 ");
          
          // Set state 2
          n_state = 2;
    
          // Target angle
          n_angleTarget = fmod(n_angle + 90, 360);     
        }
      }
            else if (n_state == 2)
      {
        // State 1
        //Serial.println("State 2 ");
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
  
        // PID
        n_speed = n_error * -n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX + n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        float x = -n_speed;
        float y = n_SPEED_REFERENCE+n_speed;
        if(x>-100){
          x=-100;
        }
        // Set motors
        motors.setSpeeds(x,n_SPEED_REFERENCE+n_speed);
   

        // Angle detection
        if (abs(n_error) < n_ANGLE_THRESHOLD)
        {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 1 ");
          
          // Set state 2
          n_state = 3;   
        }
      }else if (n_state == 3)
      {
        // State 2
        //Serial.println("State 2 ");
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
  
        // PID
        n_speed = n_error * n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX + n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        // Set motors
        motors.setSpeeds(n_SPEED_REFERENCE+n_speed,n_SPEED_REFERENCE-n_speed);    
   
        // Transition - Border detection
 
        //if (button.getSingleDebouncedRelease())
        if (sensorValues[0] > n_QTRRC_THRESHOLD)
        {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 2 ");
          
          // Set state 1
          n_state = 4;
    
          // Target angle
          n_angleTarget = fmod(n_angle - 90, 360);
        }
      }
      else if (n_state == 4)
      {
        // State 3
        //Serial.println("State 3 ");
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
        
        // PID
        n_speed = n_error * -n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX - n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        float x = -n_speed;
        float y = n_SPEED_REFERENCE+n_speed;
        if(x>-100){
          x=-150;
        }
        // Set motors
        motors.setSpeeds(-x,n_SPEED_REFERENCE+n_speed);
   
        // Angle detection
        if (abs(n_error) < n_ANGLE_THRESHOLD)
        {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 3 ");
          
          // Set state 1
          n_state = 5;
    
          // Target angle
           n_angleTarget = fmod(n_angle - 90, 360);
        }
      }      
      else if (n_state == 5)
      {
        // State 3
        
        // Actual Angle
        n_angle = angle;
  
        // Yields the angle difference in degrees between two headings
        n_error = n_angle - n_angleTarget;
  
        // PID
        n_speed = n_error * -n_kp;
        n_speed = constrain(n_speed, -n_SPEED_MAX - n_SPEED_REFERENCE, n_SPEED_MAX - n_SPEED_REFERENCE);
        float x = -n_speed;
        float y = n_SPEED_REFERENCE+n_speed;
        if(x>-100){
          x=-150;
        }
        // Set motors
        motors.setSpeeds(-x,n_SPEED_REFERENCE+n_speed);
        
   
        // Angle detection
        if (abs(n_error) < n_ANGLE_THRESHOLD)
        {
          motors.setSpeeds(0,0);
          //Serial.println("Transition 3 ");
          
          // Set state 1
          n_state = 0;

        }
      }
      }else{
        motors.setSpeeds(0,0);
      }
      
     
    }
//        // Constrain to -180 to 180 degree range
       if (n_error > 180)
          n_error -= 360;
        if (n_error < -180)
          n_error += 360;  
      
}

// Reads the gyro and uses it to update the angle estimation.
void updateAngleGyro()
{
  // Figure out how much time has passed since the last update.
  static uint16_t lastUpdate = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdate;
  lastUpdate = m;

  gyro.read();

  // Calculate how much the angle has changed, in degrees, and
  // add it to our estimation of the current angle.  The gyro's
  // sensitivity is 0.07 dps per digit.
  angle += ((float)gyro.g.z - gyroOffsetZ) * 70 * dt / 1000000000;
}
