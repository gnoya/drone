#ifndef _ROS_drone_Errors_h
#define _ROS_drone_Errors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drone
{

  class Errors : public ros::Msg
  {
    public:
      typedef float _yaw_error_type;
      _yaw_error_type yaw_error;
      typedef float _pitch_error_type;
      _pitch_error_type pitch_error;
      typedef float _roll_error_type;
      _roll_error_type roll_error;

    Errors():
      yaw_error(0),
      pitch_error(0),
      roll_error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_yaw_error;
      u_yaw_error.real = this->yaw_error;
      *(outbuffer + offset + 0) = (u_yaw_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_error);
      union {
        float real;
        uint32_t base;
      } u_pitch_error;
      u_pitch_error.real = this->pitch_error;
      *(outbuffer + offset + 0) = (u_pitch_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_error);
      union {
        float real;
        uint32_t base;
      } u_roll_error;
      u_roll_error.real = this->roll_error;
      *(outbuffer + offset + 0) = (u_roll_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_error.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_yaw_error;
      u_yaw_error.base = 0;
      u_yaw_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_error = u_yaw_error.real;
      offset += sizeof(this->yaw_error);
      union {
        float real;
        uint32_t base;
      } u_pitch_error;
      u_pitch_error.base = 0;
      u_pitch_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_error = u_pitch_error.real;
      offset += sizeof(this->pitch_error);
      union {
        float real;
        uint32_t base;
      } u_roll_error;
      u_roll_error.base = 0;
      u_roll_error.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_error.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_error.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_error.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_error = u_roll_error.real;
      offset += sizeof(this->roll_error);
     return offset;
    }

    const char * getType(){ return "drone/Errors"; };
    const char * getMD5(){ return "582ad2dbd0e654101ec424532ea4ad55"; };

  };

}
#endif