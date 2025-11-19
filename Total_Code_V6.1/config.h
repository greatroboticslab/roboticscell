#ifndef CONFIG_H
#define CONFIG_H

#include <NativeEthernet.h>
#include <AccelStepper.h>
#include <math.h>
#include <Servo.h>

extern Servo myServo;

// Pin Assignments
const int Shoulder_STEP  = 0;
const int Shoulder_DIR   = 1;
const int Elbow_STEP     = 2;
const int Elbow_DIR      = 3;
const int Rail_STEP      = 4;
const int Rail_DIR       = 5;
const int Claw           = 6;
const int Limit_Switch   = 7;
const int COGNEX_PIN     = 8;

const int RESET_PIN      = 33;
const int STOP_PIN       = 34;
const int START_PIN      = 35;
const int ESTOP_PIN      = 36;
const int POWER_ON_LIGHT = 24;
const int RESET_LIGHT    = 25;
const int STOP_LIGHT     = 26;
const int START_LIGHT    = 27;


// Light Bits
const uint8_t LIGHT_START = 1 << 0;
const uint8_t LIGHT_STOP  = 1 << 1;
const uint8_t LIGHT_RESET = 1 << 2;
const uint8_t All_Lights = LIGHT_START | LIGHT_STOP | LIGHT_RESET;

// Arm Constants
const float shoulderGearRatio = 5.0;
const float elbowGearRatio = 2.0;
const float baseOffsetX = 0.0;
const float baseOffsetY = -5.25;
const double SHOULDER_LENGTH = 9.25;
const double ELBOW_LENGTH = 7.25;
const float baseAnglePerStep = 0.0140625;
const int Open_ANGL = 0;
const int Closed_ANGL = 35;

const float shoulderAnglePerStep = baseAnglePerStep / shoulderGearRatio;
const float elbowAnglePerStep = baseAnglePerStep / elbowGearRatio;

const float toleranceDeg = 0.5;
const unsigned long debounceDelay = 300;

// Networking
extern byte mac[];
extern IPAddress ip;
extern IPAddress gateway;
extern IPAddress subnet;
extern EthernetServer server;
extern EthernetClient client;

// Motors
extern AccelStepper shoulderMotor;
extern AccelStepper elbowMotor;
extern AccelStepper railMotor;

// Global Flags
extern bool newDataReceived;
extern bool ResetState;
extern bool stopRequest;
extern bool StartState;
extern bool estopFlashed;
extern volatile bool EstopState;
extern volatile bool stopButtonPressed;

// Timers
extern unsigned long lastResetPressTime;
extern unsigned long lastStartPressTime;
extern unsigned long lastStopPressTime;

// Data Buffers
extern String inputBuffer;
extern float shoulderAngle;
extern float elbowAngle;
extern double targetX;
extern double targetY;
extern bool newXYReceived;

// Function Prototypes
void setupSystem();
void handleEstop();
void handleStopButton();
void handleStopRequest(int code);
bool MoveToTarget(float targetShoulderAngle, float targetElbowAngle);
void WaitForValidAngleData();
void TriggerCognex();
void Get_Angle();
void MoveToHome();
void setLights(uint8_t lights);
void Get_TargetXY();
void WaitForValidTargetData();
void Reset_Rail();
bool inverseKinematics(double targetX, double targetY, float &TargetShoulderAngle, float &TargetElbowAngle);

const char* getStopMessage(int code);

#endif
