#include <inttypes.h>
#include <string.h>
#include "sock-util.h"
#include "logerr_info.h"
#include "hw_motor.h"
#include "sock-gjs.h"

#define NUM_GJS_MOTORS 6

const static double UREV = 60.0; /* mm/revolution */
const static double SREV = 12800; /* steps/revolution */
const static int MAX_COUNTER = 9999999;

static int    init_done[NUM_GJS_MOTORS];
static double nxtRelPositionEGU[NUM_GJS_MOTORS];




static void init_axis(int axis_no)
{
  struct motor_init_values motor_init_values;
  double valueLow = -1.0 * MAX_COUNTER * SREV / UREV;
  double valueHigh = 1.0 * MAX_COUNTER * SREV / UREV;
  memset(&motor_init_values, 0, sizeof(motor_init_values));
  motor_init_values.ReverseERES = 1.0;
  motor_init_values.ParkingPos = 0;
  motor_init_values.MaxHomeVelocityAbs = 5.0;
  motor_init_values.lowHardLimitPos = valueLow;
  motor_init_values.highHardLimitPos = valueHigh;
  motor_init_values.hWlowPos = valueLow;
  motor_init_values.hWhighPos = valueHigh;

  hw_motor_init(axis_no,
                &motor_init_values,
                sizeof(motor_init_values));

  setMaxVelocity(axis_no, 50);
  setMRES_23(axis_no, UREV);
  setMRES_24(axis_no, SREV);

  setAmplifierPercent(axis_no, 100);
  init_done[axis_no] = 1;
}


size_t handle_gjs_request(int fd, uint8_t *buf, size_t len, size_t buff_len_max)
{
  uint8_t byte0 = buf[0];
  uint8_t byte1 = buf[1];
  const static uint8_t status_error_OK = 0xFF;
  LOGINFO7("%s/%s:%d fd=%d buffer[0]=0x%02x buffer[1]=0x%02x\n",
         __FILE__, __FUNCTION__, __LINE__, fd,
         byte0, byte1);

  if (byte0 <= 0x15) {
    int axis_no = 1 + (byte0 & 0x0F); /* axes are 0-counting in GJS, 1-conting here */
    unsigned cmd = byte1;
    if (axis_no >= NUM_GJS_MOTORS) {
      LOGERR("%s/%s:%d fd=%d NOT GJS buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
      return 0;
    }
    if (!init_done[axis_no]) {
      init_axis(axis_no);
      init_done[axis_no] = 1;
    }
    switch (cmd) {
    case 0:
      LOGERR("%s/%s:%d fd=%d NOT GJS buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
      return 0; /* not defined */
    case 1: /* read counter and status */
      {
        struct {
          char    conter_asc[7];
          uint8_t lls_hls_sign_busy_ms_end_res_one;
          uint8_t status_error;
        } reply;
        memset(&reply, 0, sizeof(reply));
        double fActPos = getMotorPos(axis_no);
        double fActPosSteps = (fActPos * SREV) / UREV ;
        int iActPosStepsAbs = abs((int)(fActPosSteps + 0.5));
        LOGINFO7("%s/%s:%d fActPos=%f fActPosSteps=%f iActPosStepsAbs=%d\n",
           __FILE__,__FUNCTION__, __LINE__,
                 fActPos, fActPosSteps, iActPosStepsAbs);
        snprintf(reply.conter_asc, sizeof(reply.conter_asc), "%d", iActPosStepsAbs);
        if (getNegLimitSwitch(axis_no)) reply.lls_hls_sign_busy_ms_end_res_one |= 0x01;
        if (getPosLimitSwitch(axis_no)) reply.lls_hls_sign_busy_ms_end_res_one |= 0x02;
        if (fActPosSteps < 0.0) reply.lls_hls_sign_busy_ms_end_res_one |= 0x04;
        if (isMotorMoving(axis_no)) reply.lls_hls_sign_busy_ms_end_res_one |= 0x08;
        reply.lls_hls_sign_busy_ms_end_res_one |= 0x80; /* transport bit, always 1 */
        reply.status_error = 0xFF;
        send_to_socket(fd, &reply, sizeof(reply));
      }
      break;
    case 2:
      {
        double position = nxtRelPositionEGU[axis_no];
        int relative = 1;
        double max_velocity = 2.0; /* TODO */
        double acceleration = 1.0;
        LOGINFO3("%s/%s:%d movePosition position=%f relative=%d max_velocity=%f acceleration=%f\n",
                 __FILE__,__FUNCTION__, __LINE__,
                 position, relative, max_velocity, acceleration);
        movePosition(axis_no, position,
                     relative, max_velocity, acceleration);
        send_to_socket(fd, &status_error_OK, sizeof(status_error_OK));
      }
      break;
    case 3:
      {
        StopInternal(axis_no);
        send_to_socket(fd, &status_error_OK, sizeof(status_error_OK));
      }
      break;
    case 4:
      LOGERR("%s/%s:%d fd=%d NOT IMPL buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
      /* reset */
      break;
    case 5:
      LOGERR("%s/%s:%d fd=%d NOT IMPL buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
      /* move to left limit switch */
      break;
    case 6:
      LOGERR("%s/%s:%d fd=%d NOT IMPL buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
      /* move to right limit switch */
      break;
  default:
    LOGERR("%s/%s:%d fd=%d NOT IMPL buffer[0]=%u buffer[1]=%u\n",
             __FILE__, __FUNCTION__, __LINE__, fd,
             byte0, byte1);
    }
    return len;
  } else if ((byte0 >= 0x20) && (byte0 <= 0x25)) {
    int axis_no = 1 + (byte0 & 0x0F); /* axes are 0-counting in GJS, 1-conting here */
    if (axis_no >= NUM_GJS_MOTORS) {
     LOGERR("%s/%s:%d fd=%d NOT GJS buffer[0]=%u buffer[1]=%u\n",
            __FILE__, __FUNCTION__, __LINE__, fd,
            byte0, byte1);
      return 0;
    }
    {
      double nxtRelPositionTmp = 0;
      unsigned i;
      uint8_t nibble;
      for (i = 0; i < 5; i++) {
        nibble = buf[1+i];
        if (nibble > 0x0F) {
          LOGERR("%s/%s:%d fd=%d nibble[%i]=0x%02x\n",
                   __FILE__, __FUNCTION__, __LINE__, fd,
                   i, nibble);
          return 0;
          LOGINFO3("%s/%s:%d fd=%d nibble[%i]=0x%02x\n",
                   __FILE__, __FUNCTION__, __LINE__, fd,
                   i, nibble);
        }
      }
      nxtRelPositionTmp = ((unsigned)buf[1]);
      nxtRelPositionTmp += ((unsigned)buf[2]) << 4;
      nxtRelPositionTmp += ((unsigned)buf[3]) << 8;
      nxtRelPositionTmp += ((unsigned)buf[4]) << 16;
      nxtRelPositionTmp += ((unsigned)buf[5]) << 20;
      LOGINFO3("%s/%s:%d fd=%d nxtRelPositionTmp=%f\n",
               __FILE__, __FUNCTION__, __LINE__, fd,
              nxtRelPositionTmp);
      {
        uint8_t dir_micro_steps = buf[6];
        if (dir_micro_steps & 1) {
          nxtRelPositionTmp = 0 - nxtRelPositionTmp;
        }
      }
      nxtRelPositionEGU[axis_no] = (nxtRelPositionTmp * UREV) / SREV ;;
      LOGINFO5("%s/%s:%d fd=%d buffer[0]=0x%02x nxtRelPositionTmp=%f\n",
               __FILE__, __FUNCTION__, __LINE__, fd,
               byte0, nxtRelPositionTmp);
      /* Ignore ramp/frequency */
        send_to_socket(fd, &status_error_OK, sizeof(status_error_OK));
    }
    return len;
  } else {
     LOGERR("%s/%s:%d fd=%d NOT GJS buffer[0]=%u buffer[1]=%u\n",
            __FILE__, __FUNCTION__, __LINE__, fd,
            byte0, byte1);
     return 0; // len;
  }
}
