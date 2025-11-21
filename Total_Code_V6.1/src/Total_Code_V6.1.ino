#include "config.h"

void setup() {
  setupSystem();
}

void loop() {
  delay(1);
  unsigned long currentMillis = millis();

  if (EstopState) {
    if (!estopFlashed) {
    Serial.println("EMERGENCY STOP TRIGGERED!");
      for (int i = 0; i < 5; i++) {
        setLights(All_Lights);
        delay(200);
        setLights(0);
        delay(200);
    }
      estopFlashed = true;
      Serial.println("System halted. ESTOP is active. Release ESTOP and Press RESET to clear.");
    }
    setLights(LIGHT_RESET);
    if (digitalRead(RESET_PIN) == HIGH && !ResetState && (millis() - lastResetPressTime > debounceDelay)) {
      lastResetPressTime = millis();
      ResetState = true;
      Serial.println("Reset after ESTOP...");
      EstopState = false;
      estopFlashed = false;
      Reset_Rail();
      WaitForValidAngleData();
      MoveToHome();
      setLights(LIGHT_START | LIGHT_RESET);
      ResetState = false;
    }

    return;
  }

  if (!client || !client.connected()) {
    client = server.available();
    inputBuffer = "";
  }

  if (stopButtonPressed) { 
    lastStopPressTime = currentMillis; 
    stopButtonPressed = false; 
    if (!stopRequest) { 
      stopRequest = true; 
      Serial.println("Stop activated! Press Stop to clear"); 
      shoulderMotor.stop(); 
      elbowMotor.stop();
      while (stopRequest){
        setLights(LIGHT_STOP);
        delay(250);
        setLights(0);
        delay(250);
        if (stopButtonPressed){
          stopRequest = false;
          stopButtonPressed = false;
          Serial.println("Stop cleared! Resuming operation...");
          setLights(LIGHT_START | LIGHT_RESET);
          return;
        }
      }
      return;
    } 
  }

  if (digitalRead(RESET_PIN) == HIGH && !ResetState && (currentMillis - lastResetPressTime > debounceDelay)) {
    lastResetPressTime = currentMillis;
    ResetState = true;
    if (stopButtonPressed) {handleStopRequest(1); return;
    } else {
      Serial.println("Reset button pressed");
    }
    setLights(LIGHT_STOP);
    Reset_Rail();
    TriggerCognex();
    Serial.println("Triggered Cognex, waiting for angle data...");
    WaitForValidAngleData();
     if (!stopButtonPressed && newXYReceived) {
      Serial.print("shoulder Angle: ");
      Serial.print(shoulderAngle);
      Serial.print(", Elbow Angle: ");
      Serial.println(elbowAngle);
    }
    if (stopButtonPressed) { handleStopRequest(2); return;}
    if (!newDataReceived) {
      Serial.println("Failed to receive valid angle data or Cognex not connected.");
      setLights(LIGHT_START | LIGHT_RESET);
      ResetState = false;
      return;
    }
    Serial.println("Starting homing sequence...");
    MoveToHome();
    setLights(LIGHT_START | LIGHT_RESET);
    ResetState = false;
  }

  if (digitalRead(START_PIN) == HIGH && !StartState && (currentMillis - lastStartPressTime > debounceDelay)) { 
    lastStartPressTime = currentMillis; 
    StartState = true; 
    Serial.println("Start button pressed"); 
    if (stopButtonPressed) { handleStopRequest(3); return; } 
    setLights(LIGHT_STOP); 
    TriggerCognex(); 
    Serial.println("Triggered Cognex, waiting for target data..."); 
    WaitForValidTargetData(); 
    if (!stopButtonPressed && newXYReceived) { 
      Serial.print("Target X: "); 
      Serial.print(targetX); 
      Serial.print(", Target Y: "); 
      Serial.println(targetY); 
      bool ikSuccess = inverseKinematics(targetX, targetY, shoulderAngle, elbowAngle); 
        if (ikSuccess) { Serial.print("IK Success! Shoulder: "); 
          Serial.print(shoulderAngle); 
          Serial.print(" deg, Elbow: "); 
          Serial.print(elbowAngle); 
          Serial.println(" deg"); 
          MoveToTarget(shoulderAngle, elbowAngle); 
          setLights(LIGHT_START | LIGHT_RESET); 
          // Ready 
        } else { 
          Serial.println("Inverse kinematics failed: Target unreachable."); 
        } 
    } else { 
      Serial.println("Failed to receive valid target data or Cognex not connected."); 
    } 
    StartState = false; 
    setLights(LIGHT_START | LIGHT_RESET); 
    return; 
  }
  
}

void handleEstop() {
  EstopState = true;
  shoulderMotor.stop();
  elbowMotor.stop();
  return;
}

void handleStopButton() {
    unsigned long now = millis();
  if (now - lastStopPressTime > debounceDelay) {
    stopButtonPressed = true;
    lastStopPressTime = now;
  }
}

void handleStopRequest(int code) {
  Serial.println(getStopMessage(code));
  setLights(LIGHT_START | LIGHT_RESET);
}

bool MoveToTarget(float targetShoulderAngle, float targetElbowAngle) {
  if (stopButtonPressed) { handleStopRequest(4); return false;}

  const int maxCorrectionAttempts = 1;
  int attempts = 0;
  bool success = false;
  int shoulderTargetSteps = round(targetShoulderAngle / shoulderAnglePerStep);
  int elbowTargetSteps = round(targetElbowAngle / elbowAnglePerStep);
  while (attempts < maxCorrectionAttempts) {
  if (stopButtonPressed) { handleStopRequest(16); return false; }
    Serial.print("MoveToTarget attempt ");
    Serial.println(attempts + 1);
    shoulderMotor.moveTo(shoulderTargetSteps);
    elbowMotor.moveTo(elbowTargetSteps);
    while (shoulderMotor.distanceToGo() != 0 || elbowMotor.distanceToGo() != 0) {
  if (stopButtonPressed) { handleStopRequest(17); return false; }
      shoulderMotor.run();
      elbowMotor.run();
    }
    newDataReceived = false;
    WaitForValidAngleData();
    if (!newDataReceived) {
      Serial.println("Warning: Did not receive valid angle data.");
      attempts++;
      continue;
    }
    float shoulderError = abs(targetShoulderAngle - shoulderAngle);
    float elbowError = abs(targetElbowAngle - elbowAngle);
    if (shoulderError <= toleranceDeg && elbowError <= toleranceDeg) {
      success = true;
      break;
    }
    int shoulderSteps = round(shoulderAngle / shoulderAnglePerStep);
    int elbowSteps = round(elbowAngle / elbowAnglePerStep);
    shoulderMotor.setCurrentPosition(shoulderSteps);
    elbowMotor.setCurrentPosition(elbowSteps);
    shoulderMotor.move(shoulderTargetSteps - shoulderSteps);
    elbowMotor.move(elbowTargetSteps - elbowSteps);
    attempts++;
  }
  if (success) {
    Serial.println("Target position reached successfully.");
  } else {
    Serial.println("Failed to reach target position after max attempts.");
  }
  return success;
}

void WaitForValidAngleData() {
  if (!client || !client.connected()) {
    Serial.println("No client connected. Skipping target data wait.");
    return;
  }
  int attempts = 0;
  const int maxAttempts = 20;
  newDataReceived = false;
  while (!newDataReceived && attempts < maxAttempts) {
    if (stopButtonPressed) { handleStopRequest(5); return; }
    Get_Angle();
    delay(100);
    attempts++;
  }
  if (!newDataReceived) {
    Serial.println("Warning: Did not receive valid angle data.");
  }
}

void TriggerCognex() {
  if (stopButtonPressed) { handleStopRequest(6); return; }
  digitalWrite(COGNEX_PIN, HIGH);
  delay(100);
  digitalWrite(COGNEX_PIN, LOW);
}

void Get_Angle() {
  if (stopButtonPressed) { handleStopRequest(7); return; }

  while (client.available()) {
    char c = client.read();

    if (c == '\n') {
      int commaIndex = inputBuffer.indexOf(',');

      if (commaIndex != -1) {
        shoulderAngle = inputBuffer.substring(0, commaIndex).toFloat();
        elbowAngle = inputBuffer.substring(commaIndex + 1).toFloat();
        newDataReceived = true;

        Serial.print("Parsed shoulder angle: ");
        Serial.println(shoulderAngle);
        Serial.print("Parsed elbow angle: ");
        Serial.println(elbowAngle);
      } else {
        Serial.println("Invalid angle data format, skipping.");
      }

      inputBuffer = "";
    } else {
      inputBuffer += c;

      // Optional: Prevent runaway buffer growth
      if (inputBuffer.length() > 100) {
        Serial.println("Input buffer too long. Resetting.");
        inputBuffer = "";
      }
    }
  }
}

void MoveToHome() {
  if (stopButtonPressed) { handleStopRequest(8); return; }
  Serial.println("Resetting Claw Rail");
  Reset_Rail();
  Serial.println("Moving to home position (0,0)");
  shoulderMotor.moveTo(0);
  elbowMotor.moveTo(0);
  while (shoulderMotor.distanceToGo() != 0 || elbowMotor.distanceToGo() != 0) {
    if (stopButtonPressed) { handleStopRequest(9); return; }
    shoulderMotor.run();
    elbowMotor.run();
  }
  if (stopButtonPressed) { handleStopRequest(10); return; }

  // Set positions to logical 0
  shoulderMotor.setCurrentPosition(0);
  elbowMotor.setCurrentPosition(0);
  Serial.println("Position set to logical zero (0,0)");

  // Check if the motors are at the correct position (0,0)
  if (!isAtHomePosition()) {
    Serial.println("Error: Motors not at home position (0,0).");
    handleStopRequest(11);  // Handle the error (e.g., stop or notify)
    return;
  }

  delay(1000);  // Short delay before moving to the next position
  if (stopButtonPressed) { handleStopRequest(18); return; }

  // Move to (60, -45) if the home position is verified
  Serial.println("Moving to (60, -45) to clear Cognex");
  bool success = MoveToTarget(60.0, -45.0); 
  if (stopButtonPressed) { handleStopRequest(12); return; }

  if (success) {
    Serial.println("Homing sequence complete and successful.");
  } else {
    Serial.println("Homing sequence complete but failed to verify position.");
  }
}

bool isAtHomePosition() {
  // Check if both motors are within a reasonable range of their home position
  float shoulderTolerance = 2.0;  // Tolerance in degrees or steps
  float elbowTolerance = 2.0;  // Tolerance in degrees or steps

  if (abs(shoulderMotor.currentPosition()) <= shoulderTolerance &&
      abs(elbowMotor.currentPosition()) <= elbowTolerance) {
    return true;
  }
  return false;
}

void setLights(uint8_t lights) {
  digitalWrite(START_LIGHT, lights & LIGHT_START ? LOW : HIGH);
  digitalWrite(STOP_LIGHT,  lights & LIGHT_STOP  ? LOW : HIGH);
  digitalWrite(RESET_LIGHT, lights & LIGHT_RESET ? LOW : HIGH);
}

void Get_TargetXY() {
  if (stopButtonPressed) { handleStopRequest(11); return; }

  while (client && client.available()) {
    char c = client.read();

    if (c == '\n') {
      int commaIndex = inputBuffer.indexOf(',');
      if (commaIndex != -1) {
        char tempStr[20];

        // Parse X
        inputBuffer.substring(0, commaIndex).toCharArray(tempStr, sizeof(tempStr));
        double inputX = atof(tempStr);

        // Parse Y
        inputBuffer.substring(commaIndex + 1).toCharArray(tempStr, sizeof(tempStr));
        double inputY = atof(tempStr);

        // Apply offsets
        targetX = inputX - baseOffsetX;
        targetY = inputY - baseOffsetY;
        newXYReceived = true;

        Serial.print("Received Target (inches): X = ");
        Serial.print(inputX); Serial.print(", Y = ");
        Serial.println(inputY);

        Serial.print("Relative to base offset: X = ");
        Serial.print(targetX, 6); Serial.print(", Y = ");
        Serial.println(targetY, 6);
      } else {
        Serial.println("Invalid target data format, skipping.");
      }

      inputBuffer = "";  // Always clear buffer after line read
    } else {
      inputBuffer += c;

      // Prevent runaway buffer growth
      if (inputBuffer.length() > 100) {
        Serial.println("Input buffer too long. Resetting.");
        inputBuffer = "";
      }
    }
  }
}

void WaitForValidTargetData() {
  if (!client || !client.connected()) {
    Serial.println("No client connected. Skipping target data wait.");
    return;
  }

  int attempts = 0;
  const int maxAttempts = 20;

  newXYReceived = false;

  while (!newXYReceived && attempts < maxAttempts) {
    if (stopButtonPressed) { handleStopRequest(12); return; }

    Get_TargetXY();
    delay(100);
    attempts++;
  }

  if (!newXYReceived) {
    Serial.println("Warning: Did not receive valid target XY data.");
  }
}

bool inverseKinematics(double targetX, double targetY, float &TargetShoulderAngle, float &TargetElbowAngle) {
  if (stopButtonPressed) { handleStopRequest(13); return false; }

  double L1 = SHOULDER_LENGTH;
  double L2 = ELBOW_LENGTH;

  double DistFromBase = sqrt(targetX * targetX + targetY * targetY);

  Serial.println("=== Inverse Kinematics Debug ===");
  Serial.print("Input Target X: "); Serial.println(targetX, 4);
  Serial.print("Input Target Y: "); Serial.println(targetY, 4);
  Serial.print("Distance from Base: "); Serial.println(DistFromBase, 4);

  if (DistFromBase > (L1 + L2) || DistFromBase < fabs(L1 - L2)) {
    Serial.println("[IK] Target unreachable: Out of range.");
    return false;
  }

  if (stopButtonPressed) { handleStopRequest(14); return false; }

  double ObjectAngleFromZero = (atan(abs(targetX)/ targetY)) * (180.0 / M_PI);
  double ShoulderBaseAngle = acos(((L1 * L1) + (DistFromBase * DistFromBase) - (L2 * L2)) / (2 * L1 * DistFromBase)) * (180.0 / M_PI);
  double ElbowShoulderAngle = acos(((L1 * L1) + (L2 * L2) - (DistFromBase * DistFromBase)) / (2 * L1 * L2)) * (180.0 / M_PI);
  double BaseElbowAngle = acos(((L2 * L2) + (DistFromBase * DistFromBase) - (L1 * L1)) / (2 * DistFromBase * L2)) * (180.0 / M_PI);

  double ShoulderAngleFromZero = ShoulderBaseAngle - ObjectAngleFromZero;
  double ElbowAngleFromZero = 180.0 - ElbowShoulderAngle;

  TargetShoulderAngle = (targetX < 0) ? abs(ShoulderAngleFromZero) : -abs(ShoulderAngleFromZero);
  TargetElbowAngle = (targetX < 0) ? -abs(ElbowAngleFromZero) : abs(ElbowAngleFromZero);

  if (stopButtonPressed) { handleStopRequest(15); return false; }

  Serial.print("ObjectAngleFromZero: "); Serial.println(ObjectAngleFromZero, 4);
  Serial.print("ShoulderBaseAngle: "); Serial.println(ShoulderBaseAngle, 4);
  Serial.print("ElbowShoulderAngle: "); Serial.println(ElbowShoulderAngle, 4);
  Serial.print("BaseElbowAngle: "); Serial.println(BaseElbowAngle, 4);
  Serial.print("Target Shoulder Angle: "); Serial.println(TargetShoulderAngle, 4);
  Serial.print("Target Elbow Angle: "); Serial.println(TargetElbowAngle, 4);
  Serial.println("=== IK Calculation Complete ===");

  return true;
}

void Reset_Rail(){
  digitalWrite(Rail_DIR, HIGH);
  railMotor.moveTo(10000000);
  while (digitalRead(Limit_Switch) == HIGH){
    railMotor.run();
  }
  railMotor.setSpeed(0);
  railMotor.setCurrentPosition(0);
  myServo.write(Open_ANGL);
}

void Pick_Up(){
  myServo.write(Open_ANGL);
}