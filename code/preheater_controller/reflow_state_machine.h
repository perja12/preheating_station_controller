#ifndef REFLOW_STATE_MACHINE_H
#define REFLOW_STATE_MACHINE_H

#include <stdbool.h>

#define TEMPERATURE_ROOM 40

/*
 * The various states of a reflow process.
 */
typedef enum REFLOW_STATE {
    REFLOW_STATE_IDLE,
    REFLOW_STATE_PREHEAT,
    REFLOW_STATE_SOAK,
    REFLOW_STATE_REFLOW_RAMP,
    REFLOW_STATE_REFLOW,
    REFLOW_STATE_COOL,
    REFLOW_STATE_COMPLETE,
    REFLOW_STATE_TOO_HOT,
    REFLOW_STATE_ERROR
} reflowState_t;

/*
 * Overall status of the reflow process. On if the reflow has started
 * and off when process has finished.
 */
typedef enum REFLOW_STATUS {
    REFLOW_STATUS_OFF,
    REFLOW_STATUS_ON
} reflowStatus_t;


/*
 * Struct for holding parameters for a specific reflow profile.
 */
struct reflowProfile_t {
    int preheatTemperature;

    int soakTemperatureMin;
    int soakTemperatureMax;
    int soakTemperatureDuration;

    int reflowTemperatureMin;
    int reflowTemperatureMax;
    int reflowTemperatureDuration;

    int coolTemperatureMin;
};

typedef struct reflowProfile_t reflowProfile_t;

void initReflow(int windowSize);
bool startReflow();
bool stopReflow();
reflowStatus_t getReflowStatus();
reflowState_t getReflowState();
double processInput(double temp);
void setReflowProfile(reflowProfile_t profile);
void setRoomTemperature(float temp);

#endif /* REFLOW_STATE_MACHINE_H */
