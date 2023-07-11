/*
FILENAME...   GJSDriver.h
USAGE...      Motor driver support for the gjs controller.
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define gjsMicroStepsString               "MicroSteps"
#define gjsRampString                     "Ramp"
#define gjsErrorCodeString                "ErrorCode"

class epicsShareClass GJSAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  GJSAxis(class GJSController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);

private:
  GJSController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  asynStatus sendDeltaStepCountAccelAndVelocity(double deltaStepCount,
                                                double accel, double velocity);

  friend class GJSController;
};

class epicsShareClass GJSController : public asynMotorController {
public:
  GJSController(const char *portName, const char *GJSPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  GJSAxis* getAxis(asynUser *pasynUser);
  GJSAxis* getAxis(int axisNo);

 private:
  int gjsMicroSteps_;
  int gjsRamp_;
  int gjsErrorCode_;

friend class GJSAxis;
};
