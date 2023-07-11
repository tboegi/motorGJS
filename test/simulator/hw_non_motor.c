#include "max_axes.h"
#include "hw_non_motor.h"


typedef struct
{
  int      manualMode;
  unsigned manualStatusReasonAux32;
} analog_input_type;

static analog_input_type analog_input[MAX_ANALOG_INPUTS];

int getAnalogInputManualSimulatorMode(unsigned analog_input_no)
{
  if (analog_input_no < MAX_ANALOG_INPUTS) {
    return analog_input[analog_input_no].manualMode;
  }
  return 0;
}

void setAnalogInputManualSimulator(unsigned analog_input_no,
                              int manualMode)
{
  if (analog_input_no < MAX_ANALOG_INPUTS) {
    analog_input[analog_input_no].manualMode = manualMode;
  }
}

unsigned getAnalogInputManualStatusReasonAux32(unsigned analog_input_no)
{
  if (analog_input_no < MAX_ANALOG_INPUTS) {
    return analog_input[analog_input_no].manualStatusReasonAux32;
  }
  return 0;
}

void setAnalogInputManualStatusReasonAux32(unsigned analog_input_no,
                                           unsigned statusReasonAux32)
{
  if (analog_input_no < MAX_ANALOG_INPUTS) {
    analog_input[analog_input_no].manualStatusReasonAux32 = statusReasonAux32;
  }
}

