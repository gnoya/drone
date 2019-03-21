#ifndef _ROS_drone_control_rc_controller_h
#define _ROS_drone_control_rc_controller_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drone_control
{

  class rc_controller : public ros::Msg
  {
    public:
      typedef float _set_yaw_type;
      _set_yaw_type set_yaw;
      typedef float _set_pitch_type;
      _set_pitch_type set_pitch;
      typedef float _set_roll_type;
      _set_roll_type set_roll;
      typedef float _set_throttle_type;
      _set_throttle_type set_throttle;

    rc_controller():
      set_yaw(0),
      set_pitch(0),
      set_roll(0),
      set_throttle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_set_yaw;
      u_set_yaw.real = this->set_yaw;
      *(outbuffer + offset + 0) = (u_set_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_yaw);
      union {
        float real;
        uint32_t base;
      } u_set_pitch;
      u_set_pitch.real = this->set_pitch;
      *(outbuffer + offset + 0) = (u_set_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_pitch);
      union {
        float real;
        uint32_t base;
      } u_set_roll;
      u_set_roll.real = this->set_roll;
      *(outbuffer + offset + 0) = (u_set_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_roll);
      union {
        float real;
        uint32_t base;
      } u_set_throttle;
      u_set_throttle.real = this->set_throttle;
      *(outbuffer + offset + 0) = (u_set_throttle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_set_throttle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_set_throttle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_set_throttle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->set_throttle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_set_yaw;
      u_set_yaw.base = 0;
      u_set_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_yaw = u_set_yaw.real;
      offset += sizeof(this->set_yaw);
      union {
        float real;
        uint32_t base;
      } u_set_pitch;
      u_set_pitch.base = 0;
      u_set_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_pitch = u_set_pitch.real;
      offset += sizeof(this->set_pitch);
      union {
        float real;
        uint32_t base;
      } u_set_roll;
      u_set_roll.base = 0;
      u_set_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_roll = u_set_roll.real;
      offset += sizeof(this->set_roll);
      union {
        float real;
        uint32_t base;
      } u_set_throttle;
      u_set_throttle.base = 0;
      u_set_throttle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_set_throttle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_set_throttle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_set_throttle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->set_throttle = u_set_throttle.real;
      offset += sizeof(this->set_throttle);
     return offset;
    }

    const char * getType(){ return "drone_control/rc_controller"; };
    const char * getMD5(){ return "74e3616c3cce2ca8d389abd9581c79f8"; };

  };

}
#endif