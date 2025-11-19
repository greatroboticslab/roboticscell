//Void Setup
// setupSystem.cpp

#include <Arduino.h>
#include <NativeEthernet.h>
#include "config.h"
#include <Servo.h>

Servo myServo;

void setupSystem() {
  Serial.begin(115200);
  delay(1);

  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(100);
    digitalWrite(LED_BUILTIN, LOW); delay(100);
  }

  Ethernet.begin(mac, ip, gateway, gateway, subnet);
  server.begin();

  Serial.println("=== TCP Server Started ===");
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.print("Listening on port: ");
  Serial.println(8888);
  Serial.println("==========================");

  pinMode(START_PIN, INPUT_PULLDOWN);
  pinMode(START_LIGHT, OUTPUT);
  pinMode(STOP_PIN, INPUT_PULLDOWN);
  pinMode(STOP_LIGHT, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLDOWN);
  pinMode(RESET_LIGHT, OUTPUT);
  pinMode(POWER_ON_LIGHT, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLDOWN); 
  pinMode(Claw, OUTPUT);
  pinMode(Limit_Switch, INPUT_PULLDOWN);

  delay(100); //prevent Interupts from triggering on startup

  myServo.attach(Claw);
  myServo.write(Open_ANGL);

  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), handleEstop, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), handleStopButton, HIGH);

  digitalWrite(START_LIGHT, LOW);
  digitalWrite(STOP_LIGHT, LOW);
  digitalWrite(RESET_LIGHT, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(POWER_ON_LIGHT, LOW);
  digitalWrite(COGNEX_PIN, LOW);
  //digitalWrite(ESTOP_PIN, HIGH);

  shoulderMotor.setMaxSpeed(10000);
  shoulderMotor.setAcceleration(3000);
  shoulderMotor.setPinsInverted(false, false, false);

  elbowMotor.setMaxSpeed(10000);
  elbowMotor.setAcceleration(3000);
  elbowMotor.setPinsInverted(true, false, false);

  railMotor.setMaxSpeed(3000);
  railMotor.setAcceleration(1000);
  railMotor.setPinsInverted(false, false, false);
  railMotor.setMinPulseWidth(5);

  setLights(LIGHT_START | LIGHT_RESET);

}
