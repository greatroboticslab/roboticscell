#ifndef STOP_MESSAGES_H
#define STOP_MESSAGES_H

const char* getStopMessage(int code) {
  switch (code) {
    case 0: return "Stop Requested: General Stop";
    case 1: return "Stop Requested During Beginning of RESET Sequence";
    case 2: return "Stop Requested Before Starting Homing Sequence";
    case 3: return "Stop Requested During Begining of START Sequence";
    case 4: return "Stop Requested During Begining of MoveToTarget Function";
    case 5: return "Stop Requested During WaitForValidAngleData Function";
    case 6: return "Stop Requested During TriggerCognex Function";
    case 7: return "Stop Requested During Get_Angle Function";
    case 8: return "Stop Requested During Begining of MoveToHome Function";
    case 9: return "Stop Requested While Moving to the Home Position";
    case 10: return "Stop Requested After Moving to Home Position";
    case 11: return "Stop Requested During Get_TargetXY Function";
    case 12: return "Stop Requested During WaitForValidTargetData Function";
    case 13: return "Stop Requested During Begining of InverseKinematics Function";
    case 14: return "Stop Requested Before Mathmatical Calculations of InverseKinematics Function";
    case 15: return "Stop Requested After Target Angles in InverseKinematics";
    case 16: return "Stop Requested In the Middle of MoveToTarget Function";
    case 17: return "Stop Requested After Attempts in MoveToTarget Function";
    case 18: return "Stop Requested After Position Set to Logical Zero in Move to Home Function";
    case 19: return "Stop Requested While Moving to 60/-45 in Move to Home Function";
    default: return "Stop Requested: Unknown Code";
  }
}

#endif
