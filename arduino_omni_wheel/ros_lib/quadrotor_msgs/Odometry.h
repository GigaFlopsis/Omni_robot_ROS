#ifndef _ROS_quadrotor_msgs_Odometry_h
#define _ROS_quadrotor_msgs_Odometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Odometry.h"

namespace quadrotor_msgs
{

  class Odometry : public ros::Msg
  {
    public:
      typedef nav_msgs::Odometry _curodom_type;
      _curodom_type curodom;
      typedef nav_msgs::Odometry _kfodom_type;
      _kfodom_type kfodom;
      typedef uint32_t _kfid_type;
      _kfid_type kfid;
      typedef uint8_t _status_type;
      _status_type status;
      enum { STATUS_ODOM_VALID = 0 };
      enum { STATUS_ODOM_INVALID = 1 };
      enum { STATUS_ODOM_LOOPCLOSURE = 2 };

    Odometry():
      curodom(),
      kfodom(),
      kfid(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->curodom.serialize(outbuffer + offset);
      offset += this->kfodom.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->kfid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->kfid >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->kfid >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->kfid >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kfid);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->curodom.deserialize(inbuffer + offset);
      offset += this->kfodom.deserialize(inbuffer + offset);
      this->kfid =  ((uint32_t) (*(inbuffer + offset)));
      this->kfid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->kfid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->kfid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->kfid);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/Odometry"; };
    virtual const char * getMD5() override { return "94d99f86002b25504a5d3354fa1ad709"; };

  };

}
#endif
