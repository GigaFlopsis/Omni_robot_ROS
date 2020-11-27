#ifndef _ROS_quadrotor_msgs_Gains_h
#define _ROS_quadrotor_msgs_Gains_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace quadrotor_msgs
{

  class Gains : public ros::Msg
  {
    public:
      typedef float _Kp_type;
      _Kp_type Kp;
      typedef float _Kd_type;
      _Kd_type Kd;
      typedef float _Kp_yaw_type;
      _Kp_yaw_type Kp_yaw;
      typedef float _Kd_yaw_type;
      _Kd_yaw_type Kd_yaw;

    Gains():
      Kp(0),
      Kd(0),
      Kp_yaw(0),
      Kd_yaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->Kp);
      offset += serializeAvrFloat64(outbuffer + offset, this->Kd);
      offset += serializeAvrFloat64(outbuffer + offset, this->Kp_yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->Kd_yaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Kp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Kd));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Kp_yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->Kd_yaw));
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/Gains"; };
    virtual const char * getMD5() override { return "b82763b9f24e5595e2a816aa779c9461"; };

  };

}
#endif
