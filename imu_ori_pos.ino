/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.F 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"



#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;

float vx=0, vy=0, vz=0;//used for tracking velocity and position
float px=0, py=0, pz=0;

float lastax=0, lastay4=0, lastaz=0;

float xAclAdjust=0, yAclAdjust=0, zAclAdjust=0;

float lastTime = millis();

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(38400);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  initializeIMU();

  calibrateAccelerations();

  lastTime = millis();//update time
}


void loop(){
  
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    float deltat = millis()-lastTime;
    readIMU(); //gets values for each 9 DOF
    updateVelocity(deltat);
    updatePosition(deltat);
    lastTime = millis();
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like! We have to make some allowance for this
  // orientation mismatch in feeding in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;


  // update LCD once per half-second independent of read rate
  if (myIMU.delt_t > 500)
  {
    printSensorValues();
    printPosition();

    updateEulerAngles();
    
    printEulerAngles();

    float R [4][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
    quaternionToMatrix(*getQ(), *(getQ()+1), *(getQ()+2), *(getQ()+3), R);
    printMatrix(R);
    
    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)
}

void initializeIMU(){
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void readIMU(){ //called in interupt
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
}

void updateEulerAngles(){
  // Define output variables from updated quaternion---these are Tait-Bryan
  // angles, commonly used in aircraft orientation. In this coordinate system,
  // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
  // x-axis and Earth magnetic North (or true North if corrected for local
  // declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the
  // Earth is positive, up toward the sky is negative. Roll is angle between
  // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
  // arise from the definition of the homogeneous rotation matrix constructed
  // from quaternions. Tait-Bryan angles as well as Euler angles are
  // non-commutative; that is, the get the correct orientation the rotations
  // must be applied in the correct order which for this configuration is yaw,
  // pitch, and then roll.
  // For more see
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // which has additional links.
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  
  myIMU.yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  myIMU.yaw   -= 8.5;
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.roll  *= RAD_TO_DEG;
}

void quaternionToMatrix(float w, float x, float y, float z, float R[4][4]){
  float Nq = pow(w,2) + pow(x,2) + pow(y,2) + pow(z,2);
  float s;
  if (Nq > 0.0){
    s = 2/Nq;
  }
  else{
    s = 0.0;
  }
  float X = x*s; float Y = y*s; float Z = z*s;
  float wX = w*X; float wY = w*Y; float wZ = w*Z;
  float xX = x*X; float xY = x*Y; float xZ = x*Z;
  float yY = y*Y; float yZ = y*Z; float zZ = z*Z;
  
  // rotation matrix conversion:
  // [ 1.0-(yY+zZ)       xY-wZ        xZ+wY  ]
  // [      xY+wZ   1.0-(xX+zZ)       yZ-wX  ]
  // [      xZ-wY        yZ+wX   1.0-(xX+yY) ]
  
  R[0][0]=1.0-(yY+zZ);  R[1][0]=xY+wZ;       R[2][0]=xZ-wY; 
  R[0][1]=xY-wZ;        R[1][1]=1.0-(xX+zZ); R[2][1]=yZ+wX;
  R[0][2]=xZ+wY;        R[1][2]=yZ-wX;       R[2][2]=1.0-(xX+yY);
}

void calibrateAccelerations(){
  Serial.println("preparing to calibrate...");
  delay(2000);
  Serial.println("calibrating...");
  float startTime = millis();
  long xsum = 0, ysum = 0, zsum =0;
  int readingCount = 0;
  while(millis() - startTime < 5000){
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  
      readIMU(); //gets values for each 9 DOF
      xsum = xsum + (int)1000*myIMU.ax;
      ysum = ysum + (int)1000*myIMU.ay;
      zsum = zsum + (int)1000*myIMU.az;
      readingCount++;
    }
  }//while loop
  xAclAdjust = (float)xsum/readingCount;
  yAclAdjust = (float)ysum/readingCount;
  zAclAdjust = (float)zsum/readingCount;
  Serial.println("calibration complete.");
  Serial.print("acceleration trims: x = ");Serial.print(xAclAdjust);
  Serial.print(" y = ");Serial.print(yAclAdjust);
  Serial.print(" z = ");Serial.println(zAclAdjust);
}

void updateVelocity(float deltat){
  //wont work if gravity direction changes
  vx = vx + ((int)1000*myIMU.ax - xAclAdjust) * deltat / 1000;
  vy = vy + ((int)1000*myIMU.ay - yAclAdjust) * deltat / 1000;
  vz = vz + ((int)1000*myIMU.az - zAclAdjust) * deltat / 1000;
}

float tolerancedAcceleration(float accel, String axis){
//  float toleranced = 0;
//  if (axis == "x"){
//     if (accel-lastax > tolerance){
//        
//     } else{
//        
//     }
//  } else if (axis == "y"){
//    if (accel-lastay > tolerance){
//      
//     } else{
//      
//     }
//  } else if (axis == "z"){
//    if (accel-lastaz > tolerance){
//      
//     } else{
//      
//     }
//  } else{
//    return 999.999;
//  }
}

void updatePosition(float deltat){
  px = px + vx * deltat;
  py = py + vy * deltat;
  pz = pz + vz * deltat;
}

void printSensorValues(){
  if(SerialDebug)
  {
     Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
     Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
     Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
     Serial.println(" mg");
  
     Serial.print("gx = "); Serial.print( myIMU.gx, 2);
     Serial.print(" gy = "); Serial.print( myIMU.gy, 2);
     Serial.print(" gz = "); Serial.print( myIMU.gz, 2);
     Serial.println(" deg/s");
  
     Serial.print("mx = "); Serial.print( (int)myIMU.mx );
     Serial.print(" my = "); Serial.print( (int)myIMU.my );
     Serial.print(" mz = "); Serial.print( (int)myIMU.mz );
     Serial.println(" mG");
  
     Serial.print("q0 = "); Serial.print(*getQ());
     Serial.print(" qx = "); Serial.print(*(getQ() + 1));
     Serial.print(" qy = "); Serial.print(*(getQ() + 2));
     Serial.print(" qz = "); Serial.println(*(getQ() + 3));
  }
}

void printPosition(){
  if(SerialDebug)
  {
    Serial.print("px = "); Serial.print(px);
    Serial.print(" py = "); Serial.print(py);
    Serial.print(" pz = "); Serial.print(pz);
    Serial.println(" mm");
  }
}

void printEulerAngles(){
  if(SerialDebug)
  {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(myIMU.yaw, 2);
    Serial.print(", ");
    Serial.print(myIMU.pitch, 2);
    Serial.print(", ");
    Serial.println(myIMU.roll, 2);
  
    Serial.print("rate = ");
    Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
    Serial.println(" Hz");
  }
}

void printMatrix(float R[][4]){
  for (int i = 0; i<4; i++){
      Serial.print(R[i][0]); Serial.print(" ");
      Serial.print(R[i][1]); Serial.print(" ");
      Serial.print(R[i][2]); Serial.print(" ");
      Serial.println(R[i][3]);
  }
}
