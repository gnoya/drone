#ifndef _ROS_drone_RCValues_h
#define _ROS_drone_RCValues_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drone
{

  class RCValues : public ros::Msg
  {
    public:
      typedef uint16_t _rc_throttle_type;
      _rc_throttle_type rc_throttle;
      typedef uint16_t _rc_yaw_type;
      _rc_yaw_type rc_yaw;
      typedef uint16_t _rc_pitch_type;
      _rc_pitch_type rc_pitch;
      typedef uint16_t _rc_roll_type;
      _rc_roll_type rc_roll;

    RCValues():
      rc_throttle(0),
      rc_yaw(0),
      rc_pitch(0),
      rc_roll(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->rc_throttle >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_throttle >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_throttle);
      *(outbuffer + offset + 0) = (this->rc_yaw >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_yaw >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_yaw);
      *(outbuffer + offset + 0) = (this->rc_pitch >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_pitch >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_pitch);
      *(outbuffer + offset + 0) = (this->rc_roll >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_roll >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_roll);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->rc_throttle =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_throttle |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_throttle);
      this->rc_yaw =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_yaw |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_yaw);
      this->rc_pitch =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_pitch |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_pitch);
      this->rc_roll =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_roll |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_roll);
     return offset;
    }

    const char * getType(){ return "drone/RCValues"; };
    const char * getMD5(){ return "ee59341fa4e2ca788add5425941e5f23"; };

  };

}
#endif