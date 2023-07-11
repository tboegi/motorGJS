/*
  FILENAME... GJSDriver.cpp
  USAGE...    Motor driver support for the GJS controller.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <epicsTypes.h>
#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "GJSDriver.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)
#define NINT64(f) (epicsInt64)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new GJSController object.
 * \param[in] portName          The name of the asyn port that will be created for this driver
 * \param[in] GJSPortName       The name of the drvAsynSerialPort that was created previously to connect to the GJS controller
 * \param[in] numAxes           The number of axes that this controller supports
 * \param[in] movingPollPeriod  The time between polls when any axis is moving
 * \param[in] idlePollPeriod    The time between polls when no axis is moving
 */
GJSController::GJSController(const char *portName, const char *GJSPortName, int numAxes,
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, 0, // Obsolete: Nnumber of additional asyn parameters
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  GJSAxis *pAxis;
  static const char *functionName = "GJSController::GJSController";

  /* Connect to GJS controller */
  status = pasynOctetSyncIO->connect(GJSPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: cannot connect to GJS controller\n",
              functionName);
  }
  /* Create asyn parameters, that connect records to data in the driver */
  createParam(gjsMicroStepsString, asynParamInt32,  &gjsMicroSteps_);
  createParam(gjsRampString,       asynParamInt32,  &gjsRamp_);
  createParam(gjsErrorCodeString,  asynParamInt32,  &gjsErrorCode_);
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new GJSAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new GJSController object.
 * Configuration command, called directly or from iocsh
 * \param[in] portName          The name of the asyn port that will be created for this driver
 * \param[in] GJSPortName       The name of the drvAsynIPPPort that was created previously to connect to the GJS controller
 * \param[in] numAxes           The number of axes that this controller supports
 * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
 * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
 */
extern "C" int GJSCreateController(const char *portName, const char *GJSPortName, int numAxes,
                                   int movingPollPeriod, int idlePollPeriod)
{
  GJSController *pGJSController
    = new GJSController(portName, GJSPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pGJSController = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information it calls asynMotorController::report()
 */
void GJSController::report(FILE *fp, int level)
{
  fprintf(fp, "GJS motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an GJSAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] pasynUser asynUser structure that encodes the axis index number. */
GJSAxis* GJSController::getAxis(asynUser *pasynUser)
{
  return static_cast<GJSAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an GJSAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] axisNo Axis index number. */
GJSAxis* GJSController::getAxis(int axisNo)
{
  return static_cast<GJSAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the GJSAxis methods

/** Creates a new GJSAxis object.
 * \param[in] pC Pointer to the GJSController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
 *
 * Initializes register numbers, etc.
 */
GJSAxis::GJSAxis(GJSController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
}

/** Reports on status of the axis
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * After printing device-specific information calls asynMotorAxis::report()
 */
void GJSAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus GJSAxis::sendDeltaStepCountAccelAndVelocity(double deltaStepCount,
                                                       double acceleration,
                                                       double velocity)
{
  const char * const functionName = "sendDeltaStepCountAccelAndVelocity";
  asynUser *pasynUser = pC_->pasynUserController_;
  int delta_step_count = NINT(deltaStepCount);
  asynStatus status;
  size_t nwrite = 0;
  size_t nread = 0;
  int eomReason = 0;
  struct {
    uint8_t cmd;
    uint8_t delta_step_count[5];
    uint8_t dir_micro_steps;
    uint8_t ramp;
    uint8_t frequency;
  } move_req;
  struct {
    uint8_t status_error;
  } move_rep;
  int negative = 0;
  memset (&move_req, 0, sizeof(move_req));
  memset (&move_rep, 0, sizeof(move_rep));
  move_req.cmd = 0x20 | axisNo_;
  {
    if (delta_step_count < 0) {
      negative = 1;
      delta_step_count = 0 - delta_step_count;
    }
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d): delta_step_count=%i 0x%x\n",
              functionName, axisNo_,
              delta_step_count, delta_step_count);
    for (size_t i = 0; i < sizeof(move_req.delta_step_count); i++) {
      move_req.delta_step_count[i] = delta_step_count & 0xF;
      delta_step_count = delta_step_count >> 4;
    }
  }
  {
    /* TODO: can we calculate ramp from acceleration ? */
    epicsInt32 ramp;
    pC_->getIntegerParam(axisNo_, pC_->gjsRamp_, &ramp);
    move_req.ramp = ramp & 0xFF;
  }
  {
    epicsInt32 micro_steps;
    pC_->getIntegerParam(axisNo_, pC_->gjsMicroSteps_, &micro_steps);
    move_req.dir_micro_steps = negative | micro_steps << 1;
  }
  {
    /* TODO: calculate fmove_requency from velocity  */
    move_req.frequency = 0;
  }

  status = pasynOctetSyncIO->write(pasynUser, (char*)&move_req, sizeof(move_req),
                                   DEFAULT_CONTROLLER_TIMEOUT,
                                   &nwrite);
  if (status || (nwrite != sizeof(move_req))) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d): deltaStepCount=%f velocity=%f, acceleration=%f status=%d nwrite=%u\n",
              functionName, axisNo_,
              deltaStepCount, velocity, acceleration,
              (int)status, (unsigned)nwrite);
    return asynError;
  };
  status = pasynOctetSyncIO->read(pasynUser,
                                  (char*)&move_rep, sizeof(move_rep),
                                  DEFAULT_CONTROLLER_TIMEOUT,
                                  &nread, &eomReason);
  if (status || (nread != sizeof(move_rep) || (move_rep.status_error != 0xFF))) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d): deltaStepCount=%f velocity=%f acceleration=%f status=%d nread=%u status_error=0x%02x\n",
              functionName, axisNo_,
              deltaStepCount, velocity, acceleration,
              (int)status, (unsigned)nread, move_rep.status_error);
    return asynError;
  }
  return status;
}

asynStatus GJSAxis::move(double position, int relative, double minVelocity,
                         double maxVelocity, double acceleration)
{
  const char * const functionName = "move";
  asynUser *pasynUser = pC_->pasynUserController_;
  double deltaStepCount;
  asynStatus status;

  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d): relative=%d position=%f maxVelocity=%f, acceleration=%f\n",
            functionName, axisNo_,
            relative, position,maxVelocity, acceleration);
  if (relative) {
    deltaStepCount = position;
  } else {
    double act_step_count;
    pC_->getDoubleParam(axisNo_, pC_->motorPosition_, &act_step_count);
    deltaStepCount = (position - act_step_count);
  }
  status = sendDeltaStepCountAccelAndVelocity(deltaStepCount,
                                              acceleration, maxVelocity);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%s(%d): deltaStepCount=%f maxVelocity=%f, acceleration=%f status1=%d\n",
              functionName, axisNo_,
              deltaStepCount, maxVelocity, acceleration,
            (int)status);

  if (status) return status;

  {
    size_t nwrite = 0;
    size_t nread = 0;
    int eomReason = 0;
    uint8_t start_req[2] = { 0x10 | axisNo_, 0x02 };
    status = pasynOctetSyncIO->write(pasynUser, (char*)&start_req, sizeof(start_req),
                                     DEFAULT_CONTROLLER_TIMEOUT,
                                     &nwrite);
    if (status || (nwrite != sizeof(start_req))) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s(%d): status=%d nwrite=%u\n",
                functionName, axisNo_, (int)status, (unsigned)nwrite);
      return asynError;
    }
    struct {
      uint8_t status_error;
    } start_rep;
    status = pasynOctetSyncIO->read(pasynUser,
                                    (char*)&start_rep, sizeof(start_rep),
                                    DEFAULT_CONTROLLER_TIMEOUT,
                                    &nread, &eomReason);
    if (status || (nread != sizeof(start_rep) || (start_rep.status_error != 0xFF))) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
                "%s(%d): status=%d nread=%u status_error=0x%02x\n",
                functionName, axisNo_,
                (int)status, (unsigned)nread, start_rep.status_error);
      return asynError;
    }
  }
  return status;
}

asynStatus GJSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynError;
  return status;
}

asynStatus GJSAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynError;
  return status;
}

asynStatus GJSAxis::stop(double acceleration )
{
  const char * const functionName = "move";
  asynUser *pasynUser = pC_->pasynUserController_;
  asynStatus status = asynError;
  size_t nwrite = 0;
  size_t nread = 0;
  int eomReason = 0;
  uint8_t stop_req[2] = { 0x10 | axisNo_, 0x03};
  status = pasynOctetSyncIO->write(pasynUser, (char*)&stop_req, sizeof(stop_req),
                                   DEFAULT_CONTROLLER_TIMEOUT,
                                   &nwrite);
  if (status || (nwrite != sizeof(stop_req))) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d): status=%d nwrite=%u\n",
              functionName, axisNo_, (int)status, (unsigned)nwrite);
    return asynError;
  }
  struct {
    uint8_t status_error;
  } stop_rep;
  status = pasynOctetSyncIO->read(pasynUser,
                                  (char*)&stop_rep, sizeof(stop_rep),
                                  DEFAULT_CONTROLLER_TIMEOUT,
                                  &nread, &eomReason);
  if (status || (nread != sizeof(stop_rep) || (stop_rep.status_error != 0xFF))) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "%s(%d): status=%d nread=%u status_error=0x%02x\n",
              functionName, axisNo_,
              (int)status, (unsigned)nread, stop_rep.status_error);
    return asynError;
  }
  return status;
}

/** Polls the axis.
 * This function reads the motor position, the limit status, the home status, the moving status,
 * and the drive power-on status.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus GJSAxis::poll(bool *moving)
{
  asynUser *pasynUser = pC_->pasynUserController_;
  asynStatus status;
  size_t nwrite = 0;
  size_t nread = 0;
  int eomReason = 0;

  uint8_t poll_req[2] = { 0x10 | axisNo_, 0x01 };
  union {
    struct {
      char    conter_asc[7];
      uint8_t lls_hls_sign_busy_ms_end_res_one;
      uint8_t status_error;
    } data;
    uint8_t bytes[9];
  } poll_rep;
  *moving = false;
  status = pasynOctetSyncIO->write(pasynUser, (char*)&poll_req, sizeof(poll_req),
                                   DEFAULT_CONTROLLER_TIMEOUT,
                                   &nwrite);
  if (status) goto skip;
  memset(&poll_rep, 0, sizeof(poll_rep));
  status = pasynOctetSyncIO->read(pasynUser,
                                  (char*)&poll_rep, sizeof(poll_rep),
                                  DEFAULT_CONTROLLER_TIMEOUT,
                                  &nread, &eomReason);

  if (status) goto skip;
  while (poll_rep.bytes[0] & 0x80) {
    /* Bytes with bit 7 set are status/error bytes.
       If there is one or more left in the TCP stream, we eat them here */
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) nwrite=%lu nread=%lu eomReson=%d conter_asc=%02x %02x %02x %02x %02x %02x %02x s=0x%02x err=0x%02x\n",
              axisNo_,
              (unsigned long)nwrite,
              (unsigned long)nread,
              eomReason, poll_rep.data.conter_asc[0],
              poll_rep.data.conter_asc[1], poll_rep.data.conter_asc[2],
              poll_rep.data.conter_asc[3], poll_rep.data.conter_asc[4],
              poll_rep.data.conter_asc[5], poll_rep.data.conter_asc[6],
              poll_rep.data.lls_hls_sign_busy_ms_end_res_one,
              poll_rep.data.status_error);
    /* "shift" everything one byte, read another byte */
    memmove(&poll_rep.bytes[0], &poll_rep.bytes[1], sizeof(poll_rep) - 1);
    status = pasynOctetSyncIO->read(pasynUser,
                                    (char*)&poll_rep.bytes[sizeof(poll_rep)-1], 1,
                                    DEFAULT_CONTROLLER_TIMEOUT,
                                    &nread, &eomReason);
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) nread2=%lu status=%d eomReson=%d\n",
              axisNo_,
              (unsigned long)nread,
              (int)status,
              eomReason);
    if (status) goto skip;
  }
  {
    int step_count = 0;
    for (size_t i = 0; i < sizeof(poll_rep.data.conter_asc); i++) {
      uint8_t n = poll_rep.data.conter_asc[i];
      if (!n) break;
      if ((n < '0') || (n > '9')) goto skip;
      step_count = step_count * 10 + n - '0';
    }
    if (poll_rep.data.lls_hls_sign_busy_ms_end_res_one & (1 << 2)) {
      /* negative */
      step_count = 0 - step_count;
    }
    /* Position of the motor. The motorRecord will translate steps into mm */
    setDoubleParam(pC_->motorPosition_, (double)step_count);
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "poll(%d) nwrite=%lu nread=%lu eomReson=%d conter_asc=%02x %02x %02x %02x %02x %02x %02x s=0x%02x err=0x%02x step_count=%d\n",
              axisNo_,
              (unsigned long)nwrite,
              (unsigned long)nread,
              eomReason, poll_rep.data.conter_asc[0],
              poll_rep.data.conter_asc[1], poll_rep.data.conter_asc[2],
              poll_rep.data.conter_asc[3], poll_rep.data.conter_asc[4],
              poll_rep.data.conter_asc[5], poll_rep.data.conter_asc[6],
              poll_rep.data.lls_hls_sign_busy_ms_end_res_one,
              poll_rep.data.status_error,
              step_count);

  }
  {
    /* done is the opposite of moving */
    int done = poll_rep.data.lls_hls_sign_busy_ms_end_res_one & (1 << 3) ? 0 : 1;
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, !done);

  }
  {
    int limit = poll_rep.data.lls_hls_sign_busy_ms_end_res_one & (1 << 0) ? 1 : 0;
    setIntegerParam(pC_->motorStatusHighLimit_, limit);
    limit = poll_rep.data.lls_hls_sign_busy_ms_end_res_one & (1 << 1) ? 1 : 0;
    setIntegerParam(pC_->motorStatusLowLimit_, limit);
    /* TODO: Bit 5: What does it mean ? */
  }
  setIntegerParam(pC_->gjsErrorCode_, poll_rep.data.status_error);


 skip:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1:0);
  callParamCallbacks();
  return status ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg GJSCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg GJSCreateControllerArg1 = {"GJS port name", iocshArgString};
static const iocshArg GJSCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg GJSCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg GJSCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const GJSCreateControllerArgs[] = {&GJSCreateControllerArg0,
                                                           &GJSCreateControllerArg1,
                                                           &GJSCreateControllerArg2,
                                                           &GJSCreateControllerArg3,
                                                           &GJSCreateControllerArg4};
static const iocshFuncDef GJSCreateControllerDef = {"GJSCreateController", 5, GJSCreateControllerArgs};
static void GJSCreateContollerCallFunc(const iocshArgBuf *args)
{
  GJSCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void GJSRegister(void)
{
  iocshRegister(&GJSCreateControllerDef, GJSCreateContollerCallFunc);
}

extern "C" {
  epicsExportRegistrar(GJSRegister);
}
