/* RTP_Gobilda_Robot_Arm_Due_Controller.ino
  by Rowan Patterson

	An Arduino Due sketch to control an RRRRR robot arm constructed from GoBilda
	build system components. The servos are controlled by an Adafruit PCA9685 I2C PWM 
	controller.
	
	The robot arm design is influenced by the Arduino Braccio arm, 
	and this sketch's concepts are influenced by BraccioV2.cpp - version 0.1 written by 
	Lukas Severinghaus which was based upon the Braccio library by Andrea Martino and 
	Angelo Ferrante (GNU GPL V1.2).
  MPU6050 code is based on Adafruit's MPU6050 library unified sensors example (BSD license).
  Input command parsing from Serial Monitor is based on interpreter.ino example by Edgar Bonet (MIT License).
	
	This library is free software; you can redistribute it and/or
 	modify it under the terms of the GNU Lesser General Public
 	License as published by the Free Software Foundation; either
 	version 2.1 of the License, or (at your option) any later version.
 	This library is distributed in the hope that it will be useful,
 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 	Lesser General Public License for more details.
 	You should have received a copy of the GNU Lesser General Public
 	License along with this library; if not, write to the Free Software
 	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  The sketch has been calibrated for Rowan's second PCA9685 Arduino Due Shield (Note OSC_FREQ changes between devices)
  Servo Type Notes
    joint1 - Gobilda 2000 Torque      (Base Rotation) 0.5ms - 2.5ms
    joint2 - Gobilda / Hitec HS-788HB (Shoulder)      0.6ms - 2.4ms **** WARNING: do not over or under drive this servo.  Min 0.6ms Max 2.4ms pulsewidth ****
    joint3 - Gobilda 2000 Torque      (Elbow)         0.5ms - 2.5ms
    joint4 - Gobilda 2000 Torque      (Wrist)         0.5ms - 2.5ms
    joint5 - Gobilda 2000 Torque      (Wrist Rotation)0.5ms - 2.5ms
    joint6 - Gobilda 2000 Torque      (Actuator)      0.5ms - 2.5ms
  */

//Libraries & Pre-requisites
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  #include <Adafruit_MPU6050.h>
  #include <avr/pgmspace.h>

  #include <array> 
  using namespace std;
   
  #ifndef ARDUINO_SAM_DUE
    #warning Sketch is designed for Arduino DUE (SAM3x) board. Not tested on AVR memory architecture.
    // Assumes Due memory architecture (string literals and constants intended to be in Flash memory)  
  #endif 
//Constants
  const int8_t pwmPort [6] =                  {   1,   2,   3,   4,   5,   6}; //Define PCA9685 port for each joint
  // Define the configuration space of the robot arm.  WARNING: Workspace will be more limited (most commonly by a tabletop)
  // Values are ticks out of 4095 defining the endpoint of the high PWM pulse. Ref. PCA9685 library setPWM() method.
  const int16_t jointMin[6] =                 {  95, 170,  95,  95,  95,  95};  //WARNING: Min and Max should be symetrical around centre 
  const int16_t jointCentre[6] =              { 300, 300, 300, 300, 300, 300}; 
  const int16_t jointMax[6] =                 { 505, 430, 505, 505, 505, 505};
  // Values are degrees of rotation matching the configuration space above.
  const int16_t jointMinDeg[6] =              { -90,-112, -90, -90, -90, -90};  //WARNING: Min and Max should be symetrical around centre 
  const int16_t jointCentreDeg[6] =           {   0,   0,   0,   0,   0,   0}; 
  const int16_t jointMaxDeg[6] =              {  90, 112,  90,  90,  90,  90};
  // Joint speeds and wait times
  const int16_t servoRotationSpeed[6] =  { 200,2100, 200, 200, 200, 200}; //For each joint, microseconds per ROTATION_SPEED_DEGREES of servo travel (including any gearing) from servo datasheet
  const int16_t startWait[6] =           { 250,5000, 250, 250, 250, 250}; //Wait time for joint to achieve start position
  const unsigned long diagWait = 30000; //Wait time between publishing diagnostics (milliseconds)

//Enumerations and Types
  enum jointIndex { // more easily reference indices of the configuration matrices by joint number
    joint1,         // n.b., joint index equals joint number -1
    joint2, 
    joint3, 
    joint4, 
    joint5, 
    joint6
  };
  enum armStateType { // States in the robot arm state machine
    arm_Idle,         //0
    arm_Moving        //1
  }; 
  enum jointStateType { // States in the per joint state machine
    joint_Idle,         //0
    joint_Moving        //1
  }; 
  enum diagStateType { // States in the diagnostics state machine
    diag_Waiting, 
    diag_Publishing
  }; 

//Variables
  jointStateType jointState[6] = {joint_Idle,joint_Idle,joint_Idle,joint_Idle,joint_Idle,joint_Idle};
  armStateType armState = arm_Idle; 
  diagStateType diagState = diag_Waiting;
  int16_t nextPose[6] ; //Queued next position the arm will move to after targetPose is reached
  int16_t targetPose[6] ; //Position the arm is moving to, composed of 6 joint angles
  int16_t nextStepPose[6] ; //The next step the arm is moving to on the path to TargetPose
  int16_t lastPose[6]; //Last position of arm
  unsigned long lastPoseTime; //Time in milliseconds that the arm's lastPose was reached
  float stepWait[6]; //Time to wait (milliseconds) for a joint to move one tick (/4095) of rotation. This is calculated from servoRotationSpeed in calcJointStepWait()
  unsigned long prevStepTime[6]; //The time the current joint stepped movement started
  unsigned long prevDiagTime; //The last time that diagnostics were published
  unsigned long currentTime; 
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;  
  static bool do_echo = true;
  // Test of Array use for recorded Poses
 
  array<array<int16_t, 6>, 1> recordedPose = {
    { 300, 300, 300, 300, 300, 300} // Standing Straight up
  };
//Compiler constants
  #define RTP_DEBUG true //Comment out to turn off debug mode
  //#define ROS_IO true.    //comment out to select Serial Monitor IO. define to select ROS IO
  #define OSC_FREQ 26315000 // Calibration for the Oscillator on this specific PCA9685 instance to achieve SERVO_FREQ. Checked on oscilloscope.
  #define SERVO_FREQ 50 // PWM Pulse frequency in Hz 
  #define SET_FREQ_WAIT 10 // Wait time for setPWMFreq function to complete
  #define ROTATION_SPEED_DEGREES 60 // Number of degrees of travel for servoRotationSpeed calculations
  //###TODO Should step size be a variable which can be adjusted from CMD line?
  #define STEP_SIZE 1 // Number of ticks per servo movement step
  #define BUF_LENGTH 128  // Buffer for the incoming command. 

//Library object instantiation
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
  Adafruit_MPU6050 mpu; // on Wire1 ref. mpu.begin()
  Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

void setup() { 
  setupArduinoDue();
  #ifdef ROS_IO
    setupROS();
  #else // Serial Monitor I/O
    setupSerialMonitor();
  #endif
  #ifdef RTP_DEBUG
    Serial.println("Debug is on");
  #endif
  setupPCA9685();
  setupRobotArm();  
  setupMPU6050();
} 

void loop() {
  currentTime = millis();
  checkInput();
  //updateArmState();
  publishDiagnostics();
} 

void setupArduinoDue() { 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  
  currentTime = millis();
  prevDiagTime = currentTime;
}

void setupROS() { // Setup for Robot Operating System (ROS) IO
    //###TODO
}

void setupSerialMonitor() { // Setup for Serial IO
  Serial.begin(115200); 
  while (!Serial)
    delay(10); 
  Serial.println(
    "+-----------------------------------------------+\r\n"
    "| RTP Gobilda Robot Arm Due PCA9685 Controller  |\r\n"
    "+-----------------------------------------------+\r\n"
    " NOTE: Serial Monitor must be set to CR only\r\n"
  );
}

void setupPCA9685 () {
  pwm.begin(); 
  pwm.setOscillatorFrequency(OSC_FREQ); 
  pwm.setPWMFreq(SERVO_FREQ);  
  delay(SET_FREQ_WAIT); 
}

void setupRobotArm() {
  #ifdef RTP_DEBUG
    //Serial.println("setupRobotArm()");
  #endif
  // Check joint config symmetry so that degree <-> tick mapping will work and calculate centre and step wait accurately.
  checkConfigSymmetry();
  //Calculate how long the servo takes in milliseconds to travel one tick (/4095) of rotation
  calcJointStepWait();
  // Move arm to starting pose
  setStartPose(); 

}

void setupMPU6050() {
  //###TODO seperate out serial IO from future ROSIO
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("WARNING: Failed to find MPU6050");
  } else {
    Serial.println("MPU6050 Found!");
    mpu_temp = mpu.getTemperatureSensor();
    mpu_temp->printSensorDetails();
    mpu_accel = mpu.getAccelerometerSensor();
    mpu_accel->printSensorDetails();
    mpu_gyro = mpu.getGyroSensor();
    mpu_gyro->printSensorDetails();
  }
}

void checkConfigSymmetry() { //Check balance of config so mapping algorithms work
  //###TODO - warning messages during ROS IO (search whole sketch)
  #ifdef RTP_DEBUG
    //Serial.println("checkConfigSymmetry()");
  #endif
  for (int8_t i = joint1; i <= joint6; i++) { 
    if ((jointCentre[i]-jointMin[i]) != (jointMax[i]-jointCentre[i])) {
      Serial.print("WARNING: Joint workspace config (ticks) asymetrical. Joint");
      Serial.println(i+1);
    }
    if ((jointCentreDeg[i]-jointMinDeg[i]) != (jointMaxDeg[i]-jointCentreDeg[i])) {
      Serial.print("WARNING: Joint workspace config (degrees) asymetrical. Joint");
      Serial.println(i+1);
    }
  }
}

void setStartPose() { // Set start pose and initialise pose variables
  // Move each joint individually to the vertical initial pose
  // Would prefer to move slowly, but problem is we do not know the starting point, so cant use step technique.
  // NB blocking code 
  #ifdef RTP_DEBUG
    //Serial.println("setStartPose()");
  #endif
  for (int8_t i = joint1; i <= joint6; i++) { 
    targetPose[i] = jointCentre[i];
    lastPose[i] = targetPose[i];
    nextStepPose[i] = targetPose[i];
    nextPose[i] = targetPose[i];
    setJoint(i,targetPose[i]);
    delay(startWait[i]);
  }
  //printPose("recordedPose",recordedPose[1]);  //DEBUG ###TODO trying to get array structure working row by row.
  //delay(max(max(max(max(max(startWait[joint1],startWait[joint2]),startWait[joint3]),startWait[joint4]),startWait[joint5]),startWait[joint6]));
}

void setJoint(int8_t joint, int16_t pulseEnd) { //Output the PWM signal for the selected Joint
  // Set a joint to a rotation using PWM servo signal   
  #ifdef RTP_DEBUG
    Serial.print("setJoint(");
    Serial.print(joint);
    Serial.print(", ");
    Serial.print(pulseEnd);
    Serial.println(")");
  #endif
  pwm.setPWM(pwmPort[joint],0,pulseEnd);
}

void printPose(char label[12], int16_t pose[6]) {
  #ifdef RTP_DEBUG
    //Serial.println("printPose()");
  #endif
  #ifdef ROS_IO
    //###TODO output pose via ROS message
  #else
    Serial.print("  ");
    Serial.print(label);
    Serial.print(" ");
    for (int8_t i = joint1; i <= joint6; i++) {
      Serial.print(pose[i]);
      Serial.print(" ");
    }
    Serial.println(" ");
  #endif
}

/*
void degreesToTicks(int16_t poseDeg[6], int16_t pose[6]) {
  #ifdef RTP_DEBUG
    Serial.println("degreesToTicks()");
  #endif
  for (int8_t i = joint1; i <= joint6; i++) { 
    pose[i] = map (poseDeg[i],jointMinDeg[i],jointMaxDeg[i],jointMin[i],jointMax[i]);
  }
}

void ticksToDegrees(int16_t pose[6], int16_t poseDeg[6]) {
  #ifdef RTP_DEBUG
    Serial.println("ticksToDegrees()");
  #endif
  for (int8_t i = joint1; i <= joint6; i++) { 
    poseDeg[i] = map (pose[i],jointMin[i],jointMax[i],jointMinDeg[i],jointMaxDeg[i]);
  }
}
*/

void calcJointStepWait() {
  // for each joint
  //   degrees_per_tick = #degrees_of_travel divided by #ticks_of_travel 
  //   ms_per_degree = servoRotationSpeed /  ROTATION_SPEED_DEGREES  
  //   ms_per_tick = degrees per tick * ms per degree
  //###TODO - these are based on no-load datasheet figures. Need a fudge factor to allow for load?
  #ifdef RTP_DEBUG
    //Serial.println("calcJointStepWait()");
  #endif
  for (int8_t i = joint1; i <= joint6; i++) { 
    stepWait[i] = (float)(jointMaxDeg[i]-jointMinDeg[i]) / (float)(jointMax[i]-jointMin[i]) *
      (float)servoRotationSpeed[i] / (float)ROTATION_SPEED_DEGREES;
  }
}

void updateArmState () {
  #ifdef RTP_DEBUG
    //Serial.println("updateArmState()");
  #endif
  switch (armState) {
    case arm_Idle:
      // Nothing to do here, wait for a new pose to arrive.
      break;
    case arm_Moving:
      //Assert: arm enters moving state by receiving a new pose command
      for (int8_t i = joint1; i <= joint6; i++) { 
        updateJointState(i);    
      }
      //###TODO if all joint states are idle, then arm state becomes idle.
      break;
  }
}

void updateJointState(int8_t joint) {
  #ifdef RTP_DEBUG
    //Serial.println("updateJointState()");
  #endif
  switch (jointState[joint]) {
    case joint_Idle:
       break;
    case joint_Moving:
      currentTime=millis();
      //Has the previous step had time to finish moving?
      if (currentTime - prevStepTime[joint] >= stepWait[joint]) {
        //Have we finished stepping to the next target pose?
        if ((targetPose[joint] - nextStepPose[joint]) > 0) {  //###TODO What if we're travelling in the other direction?
          // ###TODO Increment step - remember travel might be in either direction
          prevStepTime[joint]=millis();
          setJoint(joint,nextStepPose[joint]);
        } else {
          // ###TODO Joint has reached targetPose, joint state becomes idle
        }
      }  
      break;
  }
}

void publishDiagnostics() {
  currentTime = millis(); // Recheck time. This function can be called any time by checkInput(). Without this publishDiagnostics() gets run twice after command.
    switch (diagState) {
    case diag_Waiting:
      if (currentTime - prevDiagTime >= diagWait) {
        diagState = diag_Publishing;
      }
      break;
    case diag_Publishing:
      #ifdef RTP_DEBUG
        Serial.print("currentTime ");
        Serial.print(currentTime);
        Serial.println(" publishDiagnostics()");
        printPose("nextPose    ",nextPose);
        printPose("targetPose  ",targetPose);
        printPose("nextStepPose",nextStepPose);
        printPose("lastPose    ",lastPose);
        Serial.print("  jointState   ");
        for (int8_t i = joint1; i <= joint6; i++) { 
          Serial.print(jointState[i]);
          Serial.print(" ");    
        }
        Serial.println("");
        Serial.print("  armState     ");
        Serial.println(armState);
        //###TODO Publish interval might be too slow for useful MPU data. Averaging?
        printMPU();
        Serial.println(" ");
      #endif
      prevDiagTime = millis();
      diagState = diag_Waiting;
      break;
  }
}

void printMPU() { // Print all MPU6050 readings
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);
  //Temperature  
  Serial.print("  Temp ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");
  //Accelerometer
  Serial.print("  Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");
  //Gyro
  Serial.print("  Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  //Serial.println();
}

void checkInput() { //Check for command input
  #ifdef RTP_DEBUG
    //Serial.println("checkInput()");
  #endif
  //###TODO
  //If input available, then get new Pose vector into buffer. New input overwrites buffer.
  //If finished last command, and new pose is in queue, Update Pose target
  //###TODO manage input from serial monitor and ROS
  //###TODO seperate ROS from Serial input
  /* Process incoming commands. */
  while (Serial.available()) {
    static char buffer[BUF_LENGTH];
    static int length = 0;
    int data = Serial.read();
    if (data == '\b' || data == '\177') {  // BS and DEL
      if (length) {
        length--;
        if (do_echo) Serial.write("\b \b");
      }
    }
    else if (data == '\r') {
      if (do_echo) Serial.write("\r\n");    // output CRLF
      buffer[length] = '\0';
      if (length) exec(buffer);
      length = 0;
    }
    else if (length < BUF_LENGTH - 1) {
      buffer[length++] = data;
      if (do_echo) Serial.write(data);
    }
  }
}

static void exec(char *cmdline) {/* Execute a command. */
  //###TODO New comands for: poseDegrees; jointDegrees
  char *command = strsep(&cmdline, " ");
  if (strcmp(command, "help") == 0) {
    Serial.println(
      "+------------------------------------------+\r\n"
      "| help                                     |\r\n"
      "|                                          |\r\n"
      "| pose <j1>,<j2>,<j3>,<j4>,<j5>,<j6>       |\r\n"
      "|    - set each joint angle in ticks       |\r\n"
      "| joint <joint>,<ticks>                    |\r\n"
      "|    - set one joint angle in ticks        |\r\n"
      "| diagnose                                 |\r\n"
      "|    - force print diagnostics             |\r\n"
      "| [cartesian <x>,<y>,<z>]                  |\r\n"
      "|    - set x,y,z, pose. not yet impl       |\r\n"
      "| echo <value>                             |\r\n"
      "|    - set echo off (0) or on (1)          |\r\n"
      "+------------------------------------------+\r\n"
    );
  } else if (strcmp(command, "pose") == 0) {                  // Pose ***
    for (int8_t i = joint1; i <= joint6; i++) { 
      int16_t ticks = atoi(strsep(&cmdline, ","));
      setNextJoint(i,ticks);
    }
    #ifdef RTP_DEBUG
      printPose("nextPose  ",nextPose);           
    #endif
    //###TODO update state machines.
  } else if (strcmp(command, "joint") == 0) {                 //Joint ***
    int8_t joint = atoi(strsep(&cmdline, ","))-1;  
    int8_t ticks = atoi(cmdline);
    setNextJoint(joint,ticks);
    //###TODO What if the arm is already moving to a pose? Do the other joint targets remain as before?
    //###TODO What if the arm and this joint is already moving, and this command requires a change in direction?
    //###TODO update state machines.
  } else if (strcmp(command, "diagnose") == 0) {              //Diagnose ***
    diagState = diag_Publishing;
    publishDiagnostics();
  } else if (strcmp(command, "cartesian") == 0) {             //Cartesian ***
    int x = atoi(strsep(&cmdline, ","));
    int y = atoi(strsep(&cmdline, ","));
    int z = atoi(cmdline);
    Serial.print("  WARNING: cartesion command not yet implemented.");
  } else if (strcmp(command, "echo") == 0) {                  //Echo ***
    do_echo = atoi(cmdline);
  } else {
    Serial.print("Error: Unknown command: ");
    Serial.println(command);
  }
}

void setNextJoint (int16_t joint, int16_t ticks) { // Update a single joint in the next pose queued
  if ((ticks <= jointMax[joint]) && (ticks >=jointMin[joint])) {
    nextPose[joint] = ticks;
  } else {
    Serial.print("  ERROR: Joint setting out of bounds. Joint");
    Serial.print(joint+1);
    Serial.print(", ");
    Serial.println(ticks);
  }
}