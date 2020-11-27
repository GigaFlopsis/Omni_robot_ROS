#ifndef _ROS_SERVICE_PolygonService_h
#define _ROS_SERVICE_PolygonService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mav_planning_msgs/PolygonWithHolesStamped.h"

namespace mav_planning_msgs
{

static const char POLYGONSERVICE[] = "mav_planning_msgs/PolygonService";

  class PolygonServiceRequest : public ros::Msg
  {
    public:
      typedef mav_planning_msgs::PolygonWithHolesStamped _polygon_type;
      _polygon_type polygon;

    PolygonServiceRequest():
      polygon()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->polygon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->polygon.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return POLYGONSERVICE; };
    virtual const char * getMD5() override { return "b72bf7542ebf0f998ff6de9ed6f90873"; };

  };

  class PolygonServiceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    PolygonServiceResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return POLYGONSERVICE; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class PolygonService {
    public:
    typedef PolygonServiceRequest Request;
    typedef PolygonServiceResponse Response;
  };

}
#endif
