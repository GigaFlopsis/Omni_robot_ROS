#ifndef _ROS_drone_msgs_Goal_h
#define _ROS_drone_msgs_Goal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "drone_msgs/DronePose.h"

namespace drone_msgs
{

  class Goal : public ros::Msg
  {
    public:
      typedef int8_t _ctr_type_type;
      _ctr_type_type ctr_type;
      typedef drone_msgs::DronePose _pose_type;
      _pose_type pose;
      enum { POSE = 0 };
      enum { VEL = 1 };

    Goal():
      ctr_type(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ctr_type;
      u_ctr_type.real = this->ctr_type;
      *(outbuffer + offset + 0) = (u_ctr_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ctr_type);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ctr_type;
      u_ctr_type.base = 0;
      u_ctr_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ctr_type = u_ctr_type.real;
      offset += sizeof(this->ctr_type);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "drone_msgs/Goal"; };
    virtual const char * getMD5() override { return "bf6e29cec64ab1c71dda19cb2e5b5f60"; };

  };

}
#endif
