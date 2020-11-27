#ifndef _ROS_ego_planner_DataDisp_h
#define _ROS_ego_planner_DataDisp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ego_planner
{

  class DataDisp : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _a_type;
      _a_type a;
      typedef float _b_type;
      _b_type b;
      typedef float _c_type;
      _c_type c;
      typedef float _d_type;
      _d_type d;
      typedef float _e_type;
      _e_type e;

    DataDisp():
      header(),
      a(0),
      b(0),
      c(0),
      d(0),
      e(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->a);
      offset += serializeAvrFloat64(outbuffer + offset, this->b);
      offset += serializeAvrFloat64(outbuffer + offset, this->c);
      offset += serializeAvrFloat64(outbuffer + offset, this->d);
      offset += serializeAvrFloat64(outbuffer + offset, this->e);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->a));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->b));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->c));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->e));
     return offset;
    }

    virtual const char * getType() override { return "ego_planner/DataDisp"; };
    virtual const char * getMD5() override { return "8cc3ee9eb5e91a9074c1e70ea8f247fa"; };

  };

}
#endif
