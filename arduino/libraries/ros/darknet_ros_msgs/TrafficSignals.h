#ifndef _ROS_darknet_ros_msgs_TrafficSignals_h
#define _ROS_darknet_ros_msgs_TrafficSignals_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "darknet_ros_msgs/TrafficSignal.h"

namespace darknet_ros_msgs
{

  class TrafficSignals : public ros::Msg
  {
    public:
      uint32_t traffic_signals_length;
      typedef darknet_ros_msgs::TrafficSignal _traffic_signals_type;
      _traffic_signals_type st_traffic_signals;
      _traffic_signals_type * traffic_signals;

    TrafficSignals():
      traffic_signals_length(0), traffic_signals(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->traffic_signals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->traffic_signals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->traffic_signals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->traffic_signals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->traffic_signals_length);
      for( uint32_t i = 0; i < traffic_signals_length; i++){
      offset += this->traffic_signals[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t traffic_signals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      traffic_signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      traffic_signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      traffic_signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->traffic_signals_length);
      if(traffic_signals_lengthT > traffic_signals_length)
        this->traffic_signals = (darknet_ros_msgs::TrafficSignal*)realloc(this->traffic_signals, traffic_signals_lengthT * sizeof(darknet_ros_msgs::TrafficSignal));
      traffic_signals_length = traffic_signals_lengthT;
      for( uint32_t i = 0; i < traffic_signals_length; i++){
      offset += this->st_traffic_signals.deserialize(inbuffer + offset);
        memcpy( &(this->traffic_signals[i]), &(this->st_traffic_signals), sizeof(darknet_ros_msgs::TrafficSignal));
      }
     return offset;
    }

    const char * getType(){ return "darknet_ros_msgs/TrafficSignals"; };
    const char * getMD5(){ return "1e10022783e6ee115ff74621ed4f32a3"; };

  };

}
#endif