#ifndef _ROS_mav_planning_msgs_PolygonWithHolesStamped_h
#define _ROS_mav_planning_msgs_PolygonWithHolesStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mav_planning_msgs/PolygonWithHoles.h"

namespace mav_planning_msgs
{

  class PolygonWithHolesStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _altitude_type;
      _altitude_type altitude;
      typedef mav_planning_msgs::PolygonWithHoles _polygon_type;
      _polygon_type polygon;

    PolygonWithHolesStamped():
      header(),
      altitude(0),
      polygon()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += this->polygon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += this->polygon.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mav_planning_msgs/PolygonWithHolesStamped"; };
    virtual const char * getMD5() override { return "75e2ac63509c016edab7c5a5ed67059b"; };

  };

}
#endif
