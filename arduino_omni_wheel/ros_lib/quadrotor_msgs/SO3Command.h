#ifndef _ROS_quadrotor_msgs_SO3Command_h
#define _ROS_quadrotor_msgs_SO3Command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "quadrotor_msgs/AuxCommand.h"

namespace quadrotor_msgs
{

  class SO3Command : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _force_type;
      _force_type force;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      float kR[3];
      float kOm[3];
      typedef quadrotor_msgs::AuxCommand _aux_type;
      _aux_type aux;

    SO3Command():
      header(),
      force(),
      orientation(),
      kR(),
      kOm(),
      aux()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->force.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->kR[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->kOm[i]);
      }
      offset += this->aux.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->force.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kR[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kOm[i]));
      }
      offset += this->aux.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "quadrotor_msgs/SO3Command"; };
    virtual const char * getMD5() override { return "a466650b2633e768513aa3bf62383c86"; };

  };

}
#endif
