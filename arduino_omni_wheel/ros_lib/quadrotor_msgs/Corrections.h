#ifndef _ROS_quadrotor_msgs_Corrections_h
#define _ROS_quadrotor_msgs_Corrections_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace quadrotor_msgs
{

  class Corrections : public ros::Msg
  {
    public:
      typedef float _kf_correction_type;
      _kf_correction_type kf_correction;
      float angle_corrections[2];

    Corrections():
      kf_correction(0),
      angle_corrections()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->kf_correction);
      for( uint32_t i = 0; i < 2; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angle_corrections[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kf_correction));
      for( uint32_t i = 0; i < 2; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle_corrections[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/Corrections"; };
    virtual const char * getMD5() override { return "61e86887a75fe520847d3256306360f5"; };

  };

}
#endif
