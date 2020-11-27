#ifndef _ROS_quadrotor_msgs_StatusData_h
#define _ROS_quadrotor_msgs_StatusData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace quadrotor_msgs
{

  class StatusData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _loop_rate_type;
      _loop_rate_type loop_rate;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef uint8_t _seq_type;
      _seq_type seq;

    StatusData():
      header(),
      loop_rate(0),
      voltage(0),
      seq(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->loop_rate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loop_rate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->loop_rate);
      offset += serializeAvrFloat64(outbuffer + offset, this->voltage);
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      offset += sizeof(this->seq);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->loop_rate =  ((uint16_t) (*(inbuffer + offset)));
      this->loop_rate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->loop_rate);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->voltage));
      this->seq =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->seq);
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/StatusData"; };
    virtual const char * getMD5() override { return "c70a4ec4725273263ae176ad30f89553"; };

  };

}
#endif
