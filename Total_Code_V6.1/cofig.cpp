#include "config.h"

// Network
byte mac[] = { 0x04, 0xE9, 0xE5, 0x12, 0x34, 0x56 };
IPAddress ip(192, 168, 1, 40);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(8888);
EthernetClient client;

// Motors
AccelStepper shoulderMotor(AccelStepper::DRIVER, Shoulder_STEP, Shoulder_DIR);
AccelStepper elbowMotor(AccelStepper::DRIVER, Elbow_STEP, Elbow_DIR);
AccelStepper railMotor(AccelStepper::DRIVER, Rail_STEP, Rail_DIR);

// States
bool newDataReceived = false;
bool ResetState = false;
bool StartState = false;
bool estopFlashed = false;
bool stopRequest = false;
volatile bool EstopState = false;
volatile bool stopButtonPressed = false;

// Timers
unsigned long lastResetPressTime = 0;
unsigned long lastStartPressTime = 0;
unsigned long lastStopPressTime = 0;

// Data Buffers
String inputBuffer = "";
float shoulderAngle = 0.0;
float elbowAngle = 0.0;
double targetX = 0.0;
double targetY = 0.0;
bool newXYReceived = false;
