#ifndef _ROS_drone_Motors_h
#define _ROS_drone_Motors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drone
{

  class Motors : public ros::Msg
  {
    public:
      typedef uint16_t _front_left_type;
      _front_left_type front_left;
      typedef uint16_t _front_right_type;
      _front_right_type front_right;
      typedef uint16_t _back_left_type;
      _back_left_type back_left;
      typedef uint16_t _back_right_type;
      _back_right_type back_right;

    Motors():
      front_left(0),
      front_right(0),
      back_left(0),
      back_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->front_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->front_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->front_left);
      *(outbuffer + offset + 0) = (this->front_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->front_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->front_right);
      *(outbuffer + offset + 0) = (this->back_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->back_left >> (8 * 1)) & 0xFF;
      offset += sizeof(this->back_left);
      *(outbuffer + offset + 0) = (this->back_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->back_right >> (8 * 1)) & 0xFF;
      offset += sizeof(this->back_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->front_left =  ((uint16_t) (*(inbuffer + offset)));
      this->front_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->front_left);
      this->front_right =  ((uint16_t) (*(inbuffer + offset)));
      this->front_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->front_right);
      this->back_left =  ((uint16_t) (*(inbuffer + offset)));
      this->back_left |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->back_left);
      this->back_right =  ((uint16_t) (*(inbuffer + offset)));
      this->back_right |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->back_right);
     return offset;
    }

    const char * getType(){ return "drone/Motors"; };
    const char * getMD5(){ return "13f79d296469ce879ee28320d808839b"; };

  };

}
#endif