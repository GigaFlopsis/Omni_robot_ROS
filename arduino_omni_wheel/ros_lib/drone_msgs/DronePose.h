#ifndef _ROS_drone_msgs_DronePose_h
#define _ROS_drone_msgs_DronePose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace drone_msgs
{

  class DronePose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _point_type;
      _point_type point;
      typedef float _course_type;
      _course_type course;

    DronePose():
      point(),
      course(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->point.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_course;
      u_course.real = this->course;
      *(outbuffer + offset + 0) = (u_course.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_course.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_course.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_course.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->course);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->point.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_course;
      u_course.base = 0;
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_course.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->course = u_course.real;
      offset += sizeof(this->course);
     return offset;
    }

    virtual const char * getType() override { return "drone_msgs/DronePose"; };
    virtual const char * getMD5() override { return "c3922f772bbf0305ac9710fa392aba5a"; };

  };

}
#endif
