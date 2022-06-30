// Reflow state machine based on the TinyReflowController from https://github.com/rocketscream/TinyReflowController/blob/master/TinyReflowController.ino.

#include <PID_v1.h>
#include <stdbool.h>
#include <Arduino.h>
#include "reflow_state_machine.h"

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
//#define PID_KP_PREHEAT 17
//#define PID_KI_PREHEAT 0.11
//#define PID_KD_PREHEAT 0
// 6, 230, 40
// double consKp=1, consKi=0.05, consKd=0.25;
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

reflowState_t reflowState;
reflowStatus_t reflowStatus;
reflowProfile_t reflowProfile;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;

unsigned long stateTransitionTimestamp;

// Create PID controller with "Proportional on Measurement" turned on.
//PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);

PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void initReflow(int windowSize) {
  reflowStatus = REFLOW_STATUS_OFF;
  reflowState = REFLOW_STATE_IDLE;

   // Tell the PID to range between 0 and the full window size
  reflowOvenPID.SetOutputLimits(0, windowSize);
  reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
}

void setReflowState(reflowState_t state) {
  //Serial.print("setReflowState: ");
  //Serial.println(state);
  reflowState = state;
  stateTransitionTimestamp = millis();
}

bool startReflow() {
    if (reflowStatus == REFLOW_STATUS_OFF) {
        reflowStatus = REFLOW_STATUS_ON;
        Serial.print("#kp: ");
        Serial.print(kp);
        Serial.print(", ki: ");
        Serial.print(ki);
        Serial.print(", kd: ");
        Serial.println(kd);
        Serial.print("# sp: ");
        Serial.println(setpoint);
        return true;
    }
    return false;
}

bool stopReflow() {
    if (reflowStatus == REFLOW_STATUS_ON) {
        reflowStatus = REFLOW_STATUS_OFF;
        setReflowState(REFLOW_STATE_IDLE);
        return true;
    }
    return false;
}

reflowState_t getReflowState() {
  return reflowState;
}

reflowStatus_t getReflowStatus() {
  return reflowStatus;
}

double processInput(double temp) {
    input = temp;
    //Serial.println(reflowState);

    switch (reflowState) {
    case REFLOW_STATE_IDLE:
        // If oven temperature is still above room temperature
        if (reflowStatus == REFLOW_STATUS_ON) {
            if (input >= TEMPERATURE_ROOM) {
                setReflowState(REFLOW_STATE_TOO_HOT);
            } else {
                // Turn the PID on
                reflowOvenPID.SetMode(AUTOMATIC);

                setReflowState(REFLOW_STATE_PREHEAT);
                setpoint = reflowProfile.preheatTemperature;
            }
        }
        break;

    case REFLOW_STATE_PREHEAT:
      // The PREHEAT state is for ramping up the temperature by 1-3 deg. C per second
      // up to a given temperature (soakTemperatureMin). When this temperature is reached
      // the current statue moves to SOAK.
      if (input >= reflowProfile.soakTemperatureMin) {
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        setpoint = reflowProfile.soakTemperatureMax;
        setReflowState(REFLOW_STATE_SOAK);
      }
      break;

    case REFLOW_STATE_SOAK:
        // In the SOAK state the temperature should be maintained at steady level for a given
        // duration. When the soak period is over, the next state is REFLOW.
        if (millis() - stateTransitionTimestamp > reflowProfile.soakTemperatureDuration) {
            // Set agressive PID parameters for reflow ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            setpoint = reflowProfile.reflowTemperatureMax;
            setReflowState(REFLOW_STATE_REFLOW_RAMP);
        }
        break;

    case REFLOW_STATE_REFLOW_RAMP:
        if (input >= reflowProfile.reflowTemperatureMin) {
            setReflowState(REFLOW_STATE_REFLOW);
        }
        break;

    case REFLOW_STATE_REFLOW:
        // We need to avoid hovering at peak temperature for too long
        // Crude method that works like a charm and safe for the components
        if (millis() - stateTransitionTimestamp > reflowProfile.reflowTemperatureDuration) {
            // Set PID parameters for cooling ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            setpoint = reflowProfile.coolTemperatureMin;
            setReflowState(REFLOW_STATE_COOL);
        }
        break;

    case REFLOW_STATE_COOL:
        // If minimum cool temperature is achieve
        if (input <= reflowProfile.coolTemperatureMin) {
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Proceed to reflow Completion state
            setReflowState(REFLOW_STATE_COMPLETE);
        }
        break;

    case REFLOW_STATE_COMPLETE:
        setReflowState(REFLOW_STATE_IDLE);
        break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM) {
          // Ready to reflow
          setReflowState(REFLOW_STATE_IDLE);
      }
      break;

    case REFLOW_STATE_ERROR:
        break;
    }

    // PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON) {
        bool result = reflowOvenPID.Compute();
        //Serial.print("Setpoint: ");
        //Serial.println(setpoint);
        //Serial.print("Output: ");
        //Serial.println(output);
        return output;
    }
    return -1;
}

void setReflowProfile(reflowProfile_t profile) {
    reflowProfile = profile;
}
